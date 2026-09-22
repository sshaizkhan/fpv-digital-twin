#include "fdt/msp_client.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
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
  if (bytes.size() < 6) return false;
  if (bytes[0] != '$' || bytes[1] != 'M') {
    throw MspError("not an MSP frame: bad preamble");
  }
  if (bytes[2] == '!') {
    throw MspError("Betaflight rejected the command (MSP error reply)");
  }
  if (bytes[2] != '>') {
    throw MspError(std::string("unexpected MSP direction byte: '") + static_cast<char>(bytes[2]) + "'");
  }

  const uint8_t length = bytes[3];
  const uint8_t command = bytes[4];
  const size_t total = static_cast<size_t>(length) + 6;
  if (bytes.size() < total) return false;  // frame not complete yet

  const uint8_t expected = checksum(length, command, bytes.data() + 5, length);
  const uint8_t actual = bytes[total - 1];
  if (expected != actual) throw MspError("MSP checksum mismatch");

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
    throw MspError("not a dotted IPv4 address: " + host);
  }

  timeval tv{};
  tv.tv_sec = static_cast<time_t>(timeout.count() / 1000);
  tv.tv_usec = static_cast<suseconds_t>((timeout.count() % 1000) * 1000);
  ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    const std::string err = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw MspError("cannot reach SITL at " + host + ":" + std::to_string(port) + ": " + err +
                   " (is ./tools/run_sitl.sh running?)");
  }

  int one = 1;
  ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
}

MspClient::~MspClient() {
  if (fd_ >= 0) ::close(fd_);
}

std::vector<uint8_t> MspClient::request(Command command, const std::vector<uint8_t>& payload) {
  const std::vector<uint8_t> frame = encodeRequest(command, payload);
  size_t sent = 0;
  while (sent < frame.size()) {
    const ssize_t n = ::send(fd_, frame.data() + sent, frame.size() - sent, 0);
    if (n <= 0) throw MspError(std::string("MSP send failed: ") + std::strerror(errno));
    sent += static_cast<size_t>(n);
  }

  // Betaflight answers in order, so discard anything already buffered that is
  // not the reply we want rather than mixing up two commands' payloads.
  const auto deadline = std::chrono::steady_clock::now() + timeout_;
  for (;;) {
    Reply reply;
    size_t consumed = 0;
    if (decodeReply(buffer_, reply, consumed)) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(consumed));
      if (reply.command == command) return reply.payload;
      continue;  // stale reply to an earlier request
    }

    if (std::chrono::steady_clock::now() > deadline) {
      throw MspError("timed out waiting for a reply to MSP command " +
                     std::to_string(static_cast<int>(command)));
    }

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

uint32_t MspClient::armingDisableFlags() {
  // MSP_STATUS layout (msp.c:1086-1115):
  //   u16 taskDelta | u16 i2cErrors | u16 sensors | u32 flightModeFlags
  //   u8 pidProfile | u16 systemLoad | u16 gyroCycleTime | u8 byteCount
  //   byteCount bytes of extra mode flags | u8 flagCount | u32 armingDisableFlags
  const auto p = request(Command::Status);
  constexpr size_t kByteCountOffset = 15;
  if (p.size() <= kByteCountOffset) throw MspError("short MSP_STATUS reply");

  const size_t extra = p[kByteCountOffset];
  const size_t flags_offset = kByteCountOffset + 1 + extra + 1;  // skip extra flags and flagCount
  if (p.size() < flags_offset + 4) throw MspError("MSP_STATUS reply has no arming flags");

  return static_cast<uint32_t>(p[flags_offset]) | (static_cast<uint32_t>(p[flags_offset + 1]) << 8) |
         (static_cast<uint32_t>(p[flags_offset + 2]) << 16) |
         (static_cast<uint32_t>(p[flags_offset + 3]) << 24);
}

bool MspClient::isCalibrating() {
  constexpr uint32_t kArmingDisabledCalibrating = 1u << 12;  // runtime_config.h:55
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
