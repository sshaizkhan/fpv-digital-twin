#include "fdt/msp_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace fdt::msp {
namespace {

int16_t readInt16(const std::vector<uint8_t>& p, size_t offset) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[offset]) |
                              (static_cast<uint16_t>(p[offset + 1]) << 8));
}

uint16_t readUint16(const std::vector<uint8_t>& p, size_t offset) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[offset]) |
                               (static_cast<uint16_t>(p[offset + 1]) << 8));
}

/// Bytes to discard to get past a garbled header: everything up to the next
/// '$', or the whole buffer if there is no later '$' in it. Always >= 1, so a
/// caller that drops this much is guaranteed to make progress.
size_t bytesToResync(const std::vector<uint8_t>& bytes) {
  for (size_t i = 1; i < bytes.size(); ++i) {
    if (bytes[i] == '$') return i;
  }
  return bytes.size();
}

timeval toTimeval(std::chrono::microseconds d) {
  timeval tv{};
  if (d.count() > 0) {
    tv.tv_sec = static_cast<time_t>(d.count() / 1000000);
    tv.tv_usec = static_cast<suseconds_t>(d.count() % 1000000);
  }
  return tv;
}

}  // namespace

uint8_t checksum(uint8_t length, uint8_t command, const uint8_t* payload, size_t size) {
  uint8_t crc = static_cast<uint8_t>(length ^ command);
  for (size_t i = 0; i < size; ++i) crc = static_cast<uint8_t>(crc ^ payload[i]);
  return crc;
}

std::vector<uint8_t> encodeRequest(Command command, const std::vector<uint8_t>& payload) {
  if (payload.size() > 255) throw MspError("MSP v1 payloads are limited to 255 bytes");

  const uint8_t length = static_cast<uint8_t>(payload.size());
  const uint8_t cmd = static_cast<uint8_t>(command);

  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 6);
  frame.push_back('$');
  frame.push_back('M');
  frame.push_back('<');
  frame.push_back(length);
  frame.push_back(cmd);
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame.push_back(checksum(length, cmd, payload.data(), payload.size()));
  return frame;
}

bool decodeReply(const std::vector<uint8_t>& bytes, Reply& out, size_t& consumed) {
  // `consumed` is set on EVERY path that throws as well as on success. A
  // throwing decode used to leave it at zero, so the caller kept the offending
  // bytes at the head of its buffer and every later request re-parsed them and
  // rethrew the same error against the wrong command -- one bad byte poisoned
  // the client for good. Callers drop `consumed` bytes either way.
  consumed = 0;
  if (bytes.size() < 6) return false;

  if (bytes[0] != '$' || bytes[1] != 'M') {
    consumed = bytesToResync(bytes);
    throw MspError("not an MSP frame: bad preamble");
  }

  const uint8_t length = bytes[3];
  const uint8_t command = bytes[4];
  const size_t total = static_cast<size_t>(length) + 6;

  if (bytes[2] == '!') {
    // Betaflight's rejection carries a real length byte, so wait for the whole
    // frame before dropping it -- otherwise the tail desyncs the next parse.
    if (bytes.size() < total) return false;
    consumed = total;
    throw MspError("Betaflight rejected the command (MSP error reply)");
  }
  if (bytes[2] != '>') {
    // The direction byte is garbage, so the length byte cannot be trusted
    // either. Resync rather than believing it.
    consumed = bytesToResync(bytes);
    throw MspError(std::string("unexpected MSP direction byte: '") + static_cast<char>(bytes[2]) + "'");
  }

  if (bytes.size() < total) return false;  // frame not complete yet

  const uint8_t expected = checksum(length, command, bytes.data() + 5, length);
  const uint8_t actual = bytes[total - 1];
  if (expected != actual) {
    consumed = total;
    throw MspError("MSP checksum mismatch");
  }

  out.command = static_cast<Command>(command);
  out.payload.assign(bytes.begin() + 5, bytes.begin() + 5 + static_cast<long>(length));
  consumed = total;
  return true;
}

MspClient::MspClient(const std::string& host, uint16_t port, std::chrono::milliseconds timeout)
    : timeout_(timeout) {
  fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd_ < 0) throw MspError(std::string("cannot create socket: ") + std::strerror(errno));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    ::close(fd_);
    fd_ = -1;
    throw MspError("not a dotted IPv4 address: " + host);
  }

  const timeval tv = toTimeval(std::chrono::duration_cast<std::chrono::microseconds>(timeout));
  ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  const std::string where = host + ":" + std::to_string(port);
  const std::string hint = " (is ./tools/run_sitl.sh running?)";

  // SO_RCVTIMEO/SO_SNDTIMEO do NOT bound a blocking connect() on macOS, BSD or
  // Linux -- they only apply to an established socket. A container whose port
  // is unmapped drops the SYN instead of answering RST, which is exactly the
  // failure this error message is written for, and a blocking connect() would
  // sit there for the OS timeout of roughly 75 s while the probe printed
  // nothing at all. Drive connect() through select() on the caller's deadline.
  const int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    const std::string err = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw MspError("cannot set the MSP socket non-blocking: " + err);
  }

  int rc = ::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  if (rc < 0 && (errno == EINPROGRESS || errno == EALREADY)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
      const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
          deadline - std::chrono::steady_clock::now());
      if (remaining.count() <= 0) {
        ::close(fd_);
        fd_ = -1;
        throw MspError("timed out connecting to SITL at " + where + " after " +
                       std::to_string(timeout.count()) + " ms" + hint);
      }

      timeval wait = toTimeval(remaining);
      fd_set wfds;
      FD_ZERO(&wfds);
      FD_SET(fd_, &wfds);

      const int ready = ::select(fd_ + 1, nullptr, &wfds, nullptr, &wait);
      if (ready < 0) {
        if (errno == EINTR) continue;
        const std::string err = std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        throw MspError("select while connecting to " + where + " failed: " + err);
      }
      if (ready == 0) continue;  // re-check the deadline

      int so_error = 0;
      socklen_t len = sizeof(so_error);
      if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0) so_error = errno;
      if (so_error != 0) {
        const std::string err = std::strerror(so_error);
        ::close(fd_);
        fd_ = -1;
        throw MspError("cannot reach SITL at " + where + ": " + err + hint);
      }
      rc = 0;
      break;
    }
  }

  if (rc < 0) {
    const std::string err = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw MspError("cannot reach SITL at " + where + ": " + err + hint);
  }

  // Back to blocking: request() bounds every recv with select() itself.
  if (::fcntl(fd_, F_SETFL, flags) < 0) {
    const std::string err = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw MspError("cannot restore the MSP socket to blocking: " + err);
  }

  int one = 1;
  ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
}

MspClient::~MspClient() {
  if (fd_ >= 0) ::close(fd_);
}

std::vector<uint8_t> MspClient::request(Command command, const std::vector<uint8_t>& payload) {
  using namespace std::chrono;

  const std::vector<uint8_t> frame = encodeRequest(command, payload);
  size_t sent = 0;
  while (sent < frame.size()) {
    const ssize_t n = ::send(fd_, frame.data() + sent, frame.size() - sent, 0);
    if (n <= 0) throw MspError(std::string("MSP send failed: ") + std::strerror(errno));
    sent += static_cast<size_t>(n);
  }

  // Betaflight answers in order, so discard anything already buffered that is
  // not the reply we want rather than mixing up two commands' payloads.
  const auto deadline = steady_clock::now() + timeout_;
  for (;;) {
    Reply reply;
    size_t consumed = 0;
    bool complete = false;
    try {
      complete = decodeReply(buffer_, reply, consumed);
    } catch (...) {
      // Drop the bytes decodeReply rejected before propagating, so the next
      // request starts on a clean buffer instead of rethrowing this forever.
      dropFromBuffer(consumed);
      throw;
    }
    if (complete) {
      dropFromBuffer(consumed);
      if (reply.command == command) return reply.payload;
      continue;  // stale reply to an earlier request
    }

    // Bound this recv by the time LEFT, not by timeout_. SO_RCVTIMEO is also
    // timeout_, so checking the deadline and then letting recv block for its
    // own full timeout allowed a single request to take up to 2 x timeout_.
    const auto remaining = duration_cast<microseconds>(deadline - steady_clock::now());
    if (remaining.count() <= 0) {
      throw MspError("timed out waiting for a reply to MSP command " +
                     std::to_string(static_cast<int>(command)));
    }

    timeval wait = toTimeval(remaining);
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);

    const int ready = ::select(fd_ + 1, &rfds, nullptr, nullptr, &wait);
    if (ready < 0) {
      if (errno == EINTR) continue;
      throw MspError(std::string("select on the MSP socket failed: ") + std::strerror(errno));
    }
    if (ready == 0) continue;  // deadline check at the top of the loop reports it

    uint8_t chunk[512];
    const ssize_t n = ::recv(fd_, chunk, sizeof(chunk), 0);
    if (n > 0) {
      buffer_.insert(buffer_.end(), chunk, chunk + n);
    } else if (n == 0) {
      throw MspError("SITL closed the MSP connection");
    } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      throw MspError(std::string("MSP receive failed: ") + std::strerror(errno));
    }
  }
}

void MspClient::dropFromBuffer(size_t consumed) {
  if (consumed >= buffer_.size()) {
    buffer_.clear();
  } else if (consumed > 0) {
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(consumed));
  }
}

std::string MspClient::fcVariant() {
  const auto p = request(Command::FcVariant);
  return std::string(p.begin(), p.end());
}

std::string MspClient::fcVersion() {
  const auto p = request(Command::FcVersion);
  if (p.size() < 3) throw MspError("short MSP_FC_VERSION reply");
  return std::to_string(p[0]) + "." + std::to_string(p[1]) + "." + std::to_string(p[2]);
}

RawImu MspClient::rawImu() {
  const auto p = request(Command::RawImu);
  if (p.size() < 18) throw MspError("short MSP_RAW_IMU reply");
  RawImu imu;
  for (size_t i = 0; i < 3; ++i) {
    imu.acc[i] = readInt16(p, i * 2);
    imu.gyro[i] = readInt16(p, 6 + i * 2);
    imu.mag[i] = readInt16(p, 12 + i * 2);
  }
  return imu;
}

Attitude MspClient::attitude() {
  const auto p = request(Command::Attitude);
  if (p.size() < 6) throw MspError("short MSP_ATTITUDE reply");
  Attitude a;
  a.roll_deg = readInt16(p, 0) / 10.0;   // decidegrees on the wire
  a.pitch_deg = readInt16(p, 2) / 10.0;  // decidegrees on the wire
  a.yaw_deg = readInt16(p, 4);           // whole degrees
  return a;
}

uint32_t armingDisableFlagsFromStatus(const std::vector<uint8_t>& p) {
  // MSP_STATUS layout (msp.c:1086-1115):
  //   u16 taskDelta | u16 i2cErrors | u16 sensors | u32 flightModeFlags
  //   u8 pidProfile | u16 systemLoad | u16 gyroCycleTime | u8 byteCount
  //   byteCount bytes of extra mode flags | u8 flagCount | u32 armingDisableFlags
  constexpr size_t kByteCountOffset = 15;
  if (p.size() <= kByteCountOffset) throw MspError("short MSP_STATUS reply");

  // "Lowest 4 bits contain number of bytes that follow / header is emited even
  // when all bits fit into 32 bits to allow future extension" (msp.c:1104-1105).
  // Betaflight constrains byteCount to 0..15 before writing it, so today the
  // whole byte is the count -- but the upper nibble is explicitly reserved, so
  // mask it off rather than walking a wild offset if it is ever used.
  const size_t extra = p[kByteCountOffset] & 0x0Fu;
  const size_t flags_offset = kByteCountOffset + 1 + extra + 1;  // skip extra flags and flagCount
  if (p.size() < flags_offset + 4) throw MspError("MSP_STATUS reply has no arming flags");

  return static_cast<uint32_t>(p[flags_offset]) | (static_cast<uint32_t>(p[flags_offset + 1]) << 8) |
         (static_cast<uint32_t>(p[flags_offset + 2]) << 16) |
         (static_cast<uint32_t>(p[flags_offset + 3]) << 24);
}

uint32_t MspClient::armingDisableFlags() {
  return armingDisableFlagsFromStatus(request(Command::Status));
}

bool MspClient::isCalibrating() {
  return (armingDisableFlags() & kArmingDisabledCalibrating) != 0;
}

std::array<uint16_t, 8> MspClient::motors() {
  const auto p = request(Command::Motor);
  if (p.size() < 16) throw MspError("short MSP_MOTOR reply");
  std::array<uint16_t, 8> out{};
  for (size_t i = 0; i < 8; ++i) out[i] = readUint16(p, i * 2);
  return out;
}

}  // namespace fdt::msp
