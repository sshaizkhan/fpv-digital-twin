// UDP transport to Betaflight SITL.
//
// Plumbing only: this moves the three packet types over the wire and nothing
// else. It deliberately contains NO axis conversions, NO unit conversions and
// NO motor reordering -- those depend on conventions that are still unverified
// (docs/sitl_interface.md section 5), and a guess buried in the transport
// layer would be the hardest kind of bug to find later.
//
// Directions, all verified at sitl.c:80-83 / :310-320:
//   state  us -> SITL  udp 9003   SITL binds, we send
//   rc     us -> SITL  udp 9004   SITL binds, we send
//   motors SITL -> us  udp 9002   we bind, SITL sends
#pragma once

#include "fdt/sitl_packets.hpp"

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace fdt::sitl {

class SitlError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class SitlLink {
 public:
  struct Endpoints {
    std::string sitl_host = "127.0.0.1";  ///< where SITL is listening
    uint16_t state_port = kPortState;     ///< 9003, we send here
    uint16_t rc_port = kPortRc;           ///< 9004, we send here
    uint16_t motor_port = kPortPwm;       ///< 9002, we bind and listen here
  };

  /// Binds the motor port and prepares the outbound sockets. Throws SitlError.
  /// A `motor_port` of 0 asks the OS for an ephemeral port, which is what the
  /// tests use; boundMotorPort() then reports what was actually assigned.
  explicit SitlLink(const Endpoints& endpoints);
  ~SitlLink();

  SitlLink(const SitlLink&) = delete;
  SitlLink& operator=(const SitlLink&) = delete;
  SitlLink(SitlLink&& other) noexcept;
  SitlLink& operator=(SitlLink&& other) noexcept;

  void sendState(const FdmPacket& packet);
  void sendRc(const RcPacket& packet);

  /// Waits up to `timeout` for one motor packet. False on timeout; throws
  /// SitlError on a real socket failure. A datagram of the wrong size is
  /// counted and skipped rather than returned.
  bool receiveMotors(ServoPacket& out, std::chrono::microseconds timeout);

  /// Drop any queued motor packets. Use before a step so the packet read is
  /// the freshest one rather than a backlog.
  size_t drainMotors();

  uint16_t boundMotorPort() const { return bound_motor_port_; }

  /// Datagrams received on the motor port whose size did not match
  /// sizeof(ServoPacket) -- a non-zero count means something else is talking
  /// to that port, or SITL's packet layout changed.
  uint64_t malformedMotorPackets() const { return malformed_; }

 private:
  void closeSockets() noexcept;

  int motor_fd_ = -1;  ///< bound, receives from SITL
  int send_fd_ = -1;   ///< unbound, sends state and RC
  uint16_t bound_motor_port_ = 0;
  uint64_t malformed_ = 0;

  // sockaddr_in, kept opaque so the header does not drag in <netinet/in.h>.
  alignas(8) unsigned char state_addr_[28]{};
  alignas(8) unsigned char rc_addr_[28]{};
};

}  // namespace fdt::sitl
