// The physics <-> Betaflight SITL boundary.
//
// This is where every frame and sign conversion lives, and nowhere else. Each
// one is derived from a MEASUREMENT against a running SITL, recorded in
// docs/sitl_interface.md section 5a, not from reading the source alone -- the
// source says what SITL does to our numbers, which is only half the question.
// The other half is what Betaflight considers correct, and that had to be
// observed.
//
// The conversions are free functions, separate from the socket, so every axis
// and every motor gets a test that fails on a flipped sign or a swapped index.
// CLAUDE.md is explicit that this boundary is where the bugs will be.
#pragma once

#include "fdt/multirotor.hpp"
#include "fdt/quad_config.hpp"
#include "fdt/sitl_link.hpp"
#include "fdt/types.hpp"

#include <array>
#include <cstdint>

namespace fdt::sitl {

/// Standard sea-level pressure, fed to SITL's virtual baro. SITL's baro is the
/// only sensor that actually calibrates at boot, so this must be present and
/// steady or ARMING_DISABLED_CALIBRATING never clears.
inline constexpr double kSeaLevelPressurePa = 101325.0;

// --- RC --------------------------------------------------------------------

/// Channel values in microseconds, in the order SITL hands to Betaflight.
/// sitl.c:228-232 returns rcPkt.channels[channel] unscaled, and the debug
/// print at sitl.c:249-251 labels 0-3 as AETR and 4-7 as AUX1-4.
struct RcChannels {
  std::array<uint16_t, kMaxRcChannels> us{};

  /// Sticks centred, throttle low, every AUX low. Note this is NOT armed.
  static RcChannels neutral();

  void setAetr(uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw);

  /// `aux_index` 0 is AUX1, which is RC channel index 4.
  void setAux(size_t aux_index, uint16_t value_us);
  uint16_t aux(size_t aux_index) const;

  static constexpr size_t kRoll = 0;
  static constexpr size_t kPitch = 1;
  static constexpr size_t kThrottle = 2;
  static constexpr size_t kYaw = 3;
  static constexpr size_t kFirstAux = 4;
};

// --- conversions -----------------------------------------------------------
//
// MEASURED behaviour of SITL, per docs/sitl_interface.md section 5a:
//
//   gyro      fdm rpy[0] -> BF X same sign, rpy[1] -> BF Y NEGATED,
//             rpy[2] -> BF Z NEGATED                        (sitl.c:142-144)
//   accel     all three axes NEGATED                        (sitl.c:136-138)
//   attitude  +30 deg about fdm x -> BF roll  +30
//             +30 deg about fdm y -> BF pitch -30   <-- inverted
//             +30 deg about fdm z -> BF yaw   +30
//
// Betaflight's own conventions, so we know what "correct" is:
//
//   attitude.values.pitch = asin(-rMat[2][0]) (imu.c:317), i.e. positive is
//   NOSE UP, the aerospace sense, matching ours.
//
// Therefore:
//
//   * Accelerometer: send our FRD specific force unchanged. At rest ours is
//     [0, 0, -9.80665]; SITL negates it to +9.80665 on Z and scales by
//     256/9.80665, so Betaflight reads +256 on Z -- exactly what a real level
//     FC reads. VERIFIED by measurement.
//
//   * Gyro: negate pitch and yaw before sending, cancelling SITL's negation so
//     Betaflight receives our true FRD body rates. Sent raw, Betaflight would
//     see pitch and yaw rates inverted relative to the attitude it is given.
//
//   * Attitude: negate the quaternion's y component. Roll and yaw arrive
//     correct; only pitch inverts, and this is the minimal correction that
//     fixes it without disturbing the other two.
//
// The pitch and yaw corrections are the load-bearing ones. If either is wrong
// the quad will diverge the instant a closed loop runs, which is precisely what
// the Phase 2 scripted hover is for.

/// Body rates (rad/s, our FRD frame) as SITL's `imu_angular_velocity_rpy`.
Eigen::Vector3d gyroToSitl(const Eigen::Vector3d& body_rates);

/// Specific force (m/s^2, our FRD frame) as SITL's `imu_linear_acceleration_xyz`.
Eigen::Vector3d accelToSitl(const Eigen::Vector3d& specific_force_body);

/// Attitude as SITL's `imu_orientation_quat`, scalar first.
std::array<double, 4> attitudeToSitl(const Eigen::Quaterniond& q_wb);

/// Full state packet.
FdmPacket toFdmPacket(const State& state, const Eigen::Vector3d& specific_force_body,
                      double timestamp_s, double pressure_pa = kSeaLevelPressurePa);

RcPacket toRcPacket(const RcChannels& channels, double timestamp_s);

/// Motor packet -> throttle per PHYSICAL motor, indexed by MotorId.
///
/// Two separate permutations apply and both must be right:
///   1. SITL reorders before sending (sitl.c:592-595), so packet slot i holds
///      Betaflight motor index (i + 1) mod 4. VERIFIED from source.
///   2. Betaflight motor index -> physical position, from
///      `motors.betaflight_order` in quad.yaml. STILL UNVERIFIED -- check
///      `config.motors.order_verified` before trusting flight results.
std::array<double, 4> motorCommands(const ServoPacket& packet, const Motors& config);

// --- the link ---------------------------------------------------------------

class SitlBridge {
 public:
  explicit SitlBridge(const QuadConfig& config, const SitlLink::Endpoints& endpoints = {});

  void sendState(const State& state, const Eigen::Vector3d& specific_force_body, double timestamp_s);
  void sendRc(const RcChannels& channels, double timestamp_s);

  /// One motor packet, already converted to physical-motor throttles.
  /// False on timeout, which is not fatal -- SITL only emits a packet per
  /// completed PID cycle (sitl.c:597).
  bool receiveMotors(std::array<double, 4>& out, std::chrono::microseconds timeout);

  size_t drainMotors() { return link_.drainMotors(); }
  uint64_t malformedMotorPackets() const { return link_.malformedMotorPackets(); }
  uint16_t boundMotorPort() const { return link_.boundMotorPort(); }

  /// True once `motors.betaflight_order` has been checked against the real
  /// quad. Flight results before that are provisional.
  bool motorOrderVerified() const { return config_.motors.order_verified; }

 private:
  QuadConfig config_;
  SitlLink link_;
};

}  // namespace fdt::sitl
