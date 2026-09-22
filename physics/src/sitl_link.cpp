#include "fdt/sitl_link.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace fdt::sitl {
namespace {

static_assert(sizeof(sockaddr_in) <= 28, "opaque address storage is too small");

sockaddr_in* asAddr(unsigned char* storage) { return reinterpret_cast<sockaddr_in*>(storage); }

/// SITL itself uses inet_addr (udplink.c:36) and cannot resolve names, so we
/// match that: a numeric address only, with a clear error instead of a silent
/// INADDR_NONE.
void fillAddress(unsigned char* storage, const std::string& host, uint16_t port) {
  auto* addr = asAddr(storage);
  std::memset(addr, 0, sizeof(sockaddr_in));
  addr->sin_family = AF_INET;
  addr->sin_port = htons(port);
  if (::inet_pton(AF_INET, host.c_str(), &addr->sin_addr) != 1) {
    throw SitlError("not a dotted IPv4 address: '" + host +
                    "' (SITL uses inet_addr and cannot resolve names either)");
  }
}

[[noreturn]] void failErrno(const std::string& what) {
  throw SitlError(what + ": " + std::strerror(errno));
}

}  // namespace

SitlLink::SitlLink(const Endpoints& endpoints) {
  fillAddress(state_addr_, endpoints.sitl_host, endpoints.state_port);
  fillAddress(rc_addr_, endpoints.sitl_host, endpoints.rc_port);

  motor_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (motor_fd_ < 0) failErrno("cannot create the motor socket");

  int one = 1;
  ::setsockopt(motor_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in bind_addr{};
  bind_addr.sin_family = AF_INET;
  bind_addr.sin_port = htons(endpoints.motor_port);
  bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(motor_fd_, reinterpret_cast<const sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
    const std::string port = std::to_string(endpoints.motor_port);
    closeSockets();
    throw SitlError("cannot bind the motor port " + port + ": " + std::strerror(errno) +
                    " (is another simulator or a second SITL already running?)");
  }

  // Report the port actually assigned, which matters when 0 was requested.
  sockaddr_in actual{};
  socklen_t len = sizeof(actual);
  if (::getsockname(motor_fd_, reinterpret_cast<sockaddr*>(&actual), &len) == 0) {
    bound_motor_port_ = ntohs(actual.sin_port);
  } else {
    bound_motor_port_ = endpoints.motor_port;
  }

  send_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (send_fd_ < 0) {
    closeSockets();
    failErrno("cannot create the send socket");
  }
}

SitlLink::~SitlLink() { closeSockets(); }

void SitlLink::closeSockets() noexcept {
  if (motor_fd_ >= 0) ::close(motor_fd_);
  if (send_fd_ >= 0) ::close(send_fd_);
  motor_fd_ = -1;
  send_fd_ = -1;
}

SitlLink::SitlLink(SitlLink&& other) noexcept
    : motor_fd_(other.motor_fd_),
      send_fd_(other.send_fd_),
      bound_motor_port_(other.bound_motor_port_),
      malformed_(other.malformed_) {
  std::memcpy(state_addr_, other.state_addr_, sizeof(state_addr_));
  std::memcpy(rc_addr_, other.rc_addr_, sizeof(rc_addr_));
  other.motor_fd_ = -1;
  other.send_fd_ = -1;
}

SitlLink& SitlLink::operator=(SitlLink&& other) noexcept {
  if (this != &other) {
    closeSockets();
    motor_fd_ = other.motor_fd_;
    send_fd_ = other.send_fd_;
    bound_motor_port_ = other.bound_motor_port_;
    malformed_ = other.malformed_;
    std::memcpy(state_addr_, other.state_addr_, sizeof(state_addr_));
    std::memcpy(rc_addr_, other.rc_addr_, sizeof(rc_addr_));
    other.motor_fd_ = -1;
    other.send_fd_ = -1;
  }
  return *this;
}

void SitlLink::sendState(const FdmPacket& packet) {
  const ssize_t n = ::sendto(send_fd_, &packet, sizeof(packet), 0,
                             reinterpret_cast<const sockaddr*>(asAddr(state_addr_)), sizeof(sockaddr_in));
  if (n != static_cast<ssize_t>(sizeof(packet))) failErrno("sending the state packet failed");
}

void SitlLink::sendRc(const RcPacket& packet) {
  const ssize_t n = ::sendto(send_fd_, &packet, sizeof(packet), 0,
                             reinterpret_cast<const sockaddr*>(asAddr(rc_addr_)), sizeof(sockaddr_in));
  if (n != static_cast<ssize_t>(sizeof(packet))) failErrno("sending the RC packet failed");
}

bool SitlLink::receiveMotors(ServoPacket& out, std::chrono::microseconds timeout) {
  using namespace std::chrono;
  const auto deadline = steady_clock::now() + timeout;

  for (;;) {
    const auto remaining = duration_cast<microseconds>(deadline - steady_clock::now());
    timeval tv{};
    if (remaining.count() > 0) {
      tv.tv_sec = static_cast<time_t>(remaining.count() / 1000000);
      tv.tv_usec = static_cast<suseconds_t>(remaining.count() % 1000000);
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(motor_fd_, &fds);

    const int ready = ::select(motor_fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ready < 0) {
      if (errno == EINTR) continue;
      failErrno("select on the motor socket failed");
    }
    if (ready == 0) return false;

    // Read into a buffer larger than any packet we expect. Handing recvfrom
    // a 16-byte buffer would TRUNCATE an oversized datagram and return 16,
    // making a wrong-sized packet indistinguishable from a good one.
    alignas(8) unsigned char buffer[512];
    const ssize_t n = ::recvfrom(motor_fd_, buffer, sizeof(buffer), 0, nullptr, nullptr);
    if (n < 0) {
      if (errno == EINTR) continue;
      failErrno("receiving a motor packet failed");
    }
    if (n != static_cast<ssize_t>(sizeof(ServoPacket))) {
      // Most likely SITL's 68-byte servo_packet_raw arriving on the wrong
      // port, or a layout change. Count it and keep waiting.
      ++malformed_;
      if (steady_clock::now() >= deadline) return false;
      continue;
    }
    std::memcpy(&out, buffer, sizeof(ServoPacket));
    return true;
  }
}

size_t SitlLink::drainMotors() {
  size_t dropped = 0;
  alignas(8) unsigned char scratch[512];
  for (;;) {
    const ssize_t n = ::recvfrom(motor_fd_, scratch, sizeof(scratch), MSG_DONTWAIT, nullptr, nullptr);
    if (n < 0) break;  // EAGAIN: nothing left
    ++dropped;
  }
  return dropped;
}

}  // namespace fdt::sitl
