#include "fdt/sitl_bridge.hpp"

#include <algorithm>

namespace fdt::sitl {

// --- RC ---------------------------------------------------------------------

RcChannels RcChannels::neutral(const ArmSwitch& arm) {
  RcChannels rc;
  rc.us.fill(1000);
  rc.us[kRoll] = 1500;
  rc.us[kPitch] = 1500;
  rc.us[kYaw] = 1500;
  rc.us[kThrottle] = 1000;
  rc.setArmed(arm, false);
  return rc;
}

void RcChannels::setArmed(const ArmSwitch& arm, bool armed) {
  setAux(static_cast<size_t>(arm.aux - 1), armed ? arm.armed_us : arm.disarmed_us);
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
  // Sent unchanged. SITL's own Y/Z negation (sitl.c:142-144) is the FRD ->
  // FLU conversion into Betaflight's body frame, where +Y is nose DOWN and +Z
  // is yaw LEFT (rc.c:691-709: forward pitch stick and left yaw stick both
  // produce positive setpoints). Cancelling it would invert both rate loops.
  return body_rates;
}

Eigen::Vector3d accelToSitl(const Eigen::Vector3d& specific_force_body) {
  // Betaflight's accel shares its gyro's FLU frame, so it wants
  // (fx, -fy, -fz) of our FRD specific force. SITL negates all three
  // (sitl.c:136-138), so pre-negate X only. At rest [0, 0, -g] still arrives
  // as +256 on Z; a nose-down tilt now reads negative on X, as on a real FC.
  return {-specific_force_body.x(), specific_force_body.y(), specific_force_body.z()};
}

std::array<double, 4> attitudeToSitl(const Eigen::Quaterniond& q_wb) {
  // Sent unchanged, scalar first. Under SITL imuComputeRotationMatrix patches
  // rMat[1][0] and rMat[2][0] (imu.c:162-165), so our NED/FRD q_wb comes out
  // as roll = ours, pitch = -ours (Betaflight's pitch is nose-DOWN positive:
  // pid.c:387-395 drives it toward a positive target on forward stick), and
  // yaw = our heading. That is already Betaflight's convention.
  return {q_wb.w(), q_wb.x(), q_wb.y(), q_wb.z()};
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
