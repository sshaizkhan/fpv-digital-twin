#include "fdt/pose_publisher.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace fdt {
namespace {
sockaddr_in* asAddr(unsigned char* storage) { return reinterpret_cast<sockaddr_in*>(storage); }
}  // namespace

PosePacket makePosePacket(const Multirotor& quad, bool armed, double throttle) {
  const State& s = quad.state();
  const Telemetry& t = quad.telemetry();

  PosePacket p;
  p.flags = static_cast<uint16_t>((armed ? kPoseArmed : 0) |
                                  (t.in_contact ? kPoseInContact : 0));
  p.time_s = quad.time();
  for (int i = 0; i < 3; ++i) {
    p.position_ned[i] = s.position[i];
    p.velocity_ned[i] = s.velocity[i];
  }
  // Scalar first, as docs/coordinate_frames.md requires on the wire. Eigen's
  // internal storage is (x, y, z, w), so name the components rather than
  // copying the block.
  p.orientation[0] = s.orientation.w();
  p.orientation[1] = s.orientation.x();
  p.orientation[2] = s.orientation.y();
  p.orientation[3] = s.orientation.z();

  for (size_t i = 0; i < 4; ++i) p.motor_rpm[i] = static_cast<float>(t.motor_rpm[i]);
  p.battery_v = static_cast<float>(t.battery_voltage);
  p.throttle = static_cast<float>(throttle);
  return p;
}

PosePublisher::PosePublisher(const std::string& host, uint16_t port) {
  auto* addr = asAddr(addr_);
  std::memset(addr, 0, sizeof(sockaddr_in));
  addr->sin_family = AF_INET;
  addr->sin_port = htons(port);
  if (::inet_pton(AF_INET, host.c_str(), &addr->sin_addr) != 1) {
    throw ConfigError("viewer host is not a dotted IPv4 address: " + host);
  }

  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) throw ConfigError(std::string("cannot create the pose socket: ") + std::strerror(errno));
}

PosePublisher::~PosePublisher() {
  if (fd_ >= 0) ::close(fd_);
}

void PosePublisher::send(const PosePacket& packet) {
  const ssize_t n = ::sendto(fd_, &packet, sizeof(packet), 0,
                             reinterpret_cast<const sockaddr*>(asAddr(addr_)), sizeof(sockaddr_in));
  // Nobody listening is the normal case for a spectator, and must never stall
  // or fail the physics. Count it so it is visible if it matters.
  if (n == static_cast<ssize_t>(sizeof(packet))) {
    ++sent_;
  } else {
    ++failed_;
  }
}

void PosePublisher::publish(const Multirotor& quad, bool armed, double throttle) {
  send(makePosePacket(quad, armed, throttle));
}

bool PosePublisher::publishThrottled(const Multirotor& quad, bool armed, double throttle,
                                     double rate_hz) {
  if (!(rate_hz > 0.0)) return false;
  const double now = quad.time();
  const double interval = 1.0 / rate_hz;
  if (last_sent_time_ >= 0.0 && (now - last_sent_time_) < interval) return false;
  last_sent_time_ = now;
  publish(quad, armed, throttle);
  return true;
}

}  // namespace fdt
