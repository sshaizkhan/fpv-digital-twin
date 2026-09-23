#include "fdt/sitl_bridge.hpp"

#include <algorithm>

namespace fdt::sitl {

// --- RC ---------------------------------------------------------------------

RcChannels RcChannels::neutral() {
  RcChannels rc;
  rc.us.fill(1000);
  rc.us[kRoll] = 1500;
  rc.us[kPitch] = 1500;
  rc.us[kYaw] = 1500;
  rc.us[kThrottle] = 1000;
  return rc;
}

void RcChannels::setAetr(uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw) {
  us[kRoll] = roll;
  us[kPitch] = pitch;
  us[kThrottle] = throttle;
  us[kYaw] = yaw;
}

void RcChannels::setAux(size_t aux_index, uint16_t value_us) {
  const size_t channel = kFirstAux + aux_index;
  if (channel < kMaxRcChannels) us[channel] = value_us;
}

uint16_t RcChannels::aux(size_t aux_index) const {
  const size_t channel = kFirstAux + aux_index;
  return (channel < kMaxRcChannels) ? us[channel] : 0;
}

// --- conversions ------------------------------------------------------------

Eigen::Vector3d gyroToSitl(const Eigen::Vector3d& body_rates) {
  // SITL negates Y and Z (sitl.c:142-144). Pre-negate so Betaflight ends up
  // with our true FRD rates rather than a pitch/yaw-mirrored world that
  // disagrees with the attitude quaternion we also send.
  return {body_rates.x(), -body_rates.y(), -body_rates.z()};
}

Eigen::Vector3d accelToSitl(const Eigen::Vector3d& specific_force_body) {
  // Sent unchanged. SITL negates all three (sitl.c:136-138), which turns our
  // at-rest [0, 0, -g] into Betaflight's expected +256 on Z. Measured.
  return specific_force_body;
}

std::array<double, 4> attitudeToSitl(const Eigen::Quaterniond& q_wb) {
  // Scalar first, and the y component negated: measured, +30 deg about our
  // body y otherwise arrives as -30 deg of Betaflight pitch (imu.c:317 defines
  // its pitch as nose-up positive, the same sense as ours).
  return {q_wb.w(), q_wb.x(), -q_wb.y(), q_wb.z()};
}

FdmPacket toFdmPacket(const State& state, const Eigen::Vector3d& specific_force_body,
                      double timestamp_s, double pressure_pa) {
  FdmPacket packet{};
  packet.timestamp = timestamp_s;

  const Eigen::Vector3d gyro = gyroToSitl(state.angular_velocity);
  const Eigen::Vector3d accel = accelToSitl(specific_force_body);
  const std::array<double, 4> quat = attitudeToSitl(state.orientation);

  for (int i = 0; i < 3; ++i) {
    packet.imu_angular_velocity_rpy[i] = gyro[i];
    packet.imu_linear_acceleration_xyz[i] = accel[i];
    // Position and velocity are already NED, which is what the packet wants
    // (target.h:260-261) and what our world frame is. No conversion.
    packet.velocity_xyz[i] = state.velocity[i];
    packet.position_xyz[i] = state.position[i];
  }
  for (size_t i = 0; i < 4; ++i) packet.imu_orientation_quat[i] = quat[i];

  packet.pressure = pressure_pa;
  return packet;
}

RcPacket toRcPacket(const RcChannels& channels, double timestamp_s) {
  RcPacket packet{};
  packet.timestamp = timestamp_s;
  for (size_t i = 0; i < kMaxRcChannels; ++i) packet.channels[i] = channels.us[i];
  return packet;
}

std::array<double, 4> motorCommands(const ServoPacket& packet, const Motors& config) {
  std::array<double, 4> out{};
  for (size_t slot = 0; slot < 4; ++slot) {
    // Undo SITL's reordering, then Betaflight's index -> physical mapping.
    const size_t bf_index = betaflightMotorForPacketSlot(slot);
    const MotorId motor = config.motorForBetaflightIndex(static_cast<int>(bf_index) + 1);

    // Already throttle-above-idle normalised: SITL subtracted idlePulse
    // (sitl.c:557) and divided by 1000 (sitl.c:587). Do not re-apply either.
    const double value = static_cast<double>(packet.motor_speed[slot]);
    out[static_cast<size_t>(motor)] = std::clamp(value, 0.0, 1.0);
  }
  return out;
}

// --- the link ---------------------------------------------------------------

SitlBridge::SitlBridge(const QuadConfig& config, const SitlLink::Endpoints& endpoints)
    : config_(config), link_(endpoints) {}

void SitlBridge::sendState(const State& state, const Eigen::Vector3d& specific_force_body,
                           double timestamp_s) {
  link_.sendState(toFdmPacket(state, specific_force_body, timestamp_s));
}

void SitlBridge::sendRc(const RcChannels& channels, double timestamp_s) {
  link_.sendRc(toRcPacket(channels, timestamp_s));
}

bool SitlBridge::receiveMotors(std::array<double, 4>& out, std::chrono::microseconds timeout) {
  ServoPacket packet{};
  if (!link_.receiveMotors(packet, timeout)) return false;
  out = motorCommands(packet, config_.motors);
  return true;
}

}  // namespace fdt::sitl
