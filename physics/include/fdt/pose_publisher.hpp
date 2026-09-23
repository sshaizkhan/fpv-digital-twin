// Sends PosePackets to the viewer over UDP.
//
// Fire and forget: the viewer is a spectator and must never be able to stall
// the physics. A send failure is counted, not thrown -- a closed viewer is the
// normal case, not an error.
#pragma once

#include "fdt/multirotor.hpp"
#include "fdt/pose_packet.hpp"

#include <cstdint>
#include <string>

namespace fdt {

class PosePublisher {
 public:
  /// Throws ConfigError only if the address is unusable; a viewer that is not
  /// listening is fine and produces no error at all (UDP).
  PosePublisher(const std::string& host, uint16_t port);
  ~PosePublisher();

  PosePublisher(const PosePublisher&) = delete;
  PosePublisher& operator=(const PosePublisher&) = delete;

  /// Build a packet from the vehicle's current state and send it.
  void publish(const Multirotor& quad, bool armed, double throttle);

  void send(const PosePacket& packet);

  /// Send at most `rate_hz`; returns true if this call actually sent.
  /// CLAUDE.md asks for 120 Hz to the viewer, well below the physics rate.
  bool publishThrottled(const Multirotor& quad, bool armed, double throttle, double rate_hz);

  uint64_t sent() const { return sent_; }
  uint64_t failed() const { return failed_; }

 private:
  int fd_ = -1;
  alignas(8) unsigned char addr_[28]{};
  uint64_t sent_ = 0;
  uint64_t failed_ = 0;
  double last_sent_time_ = -1.0;
};

/// Fill a packet from a vehicle state. Separated from the socket so the
/// conversion can be tested without one.
PosePacket makePosePacket(const Multirotor& quad, bool armed, double throttle);

}  // namespace fdt
