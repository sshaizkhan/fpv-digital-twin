// Wire format for the Betaflight SITL UDP links.
//
// The structs below are transcribed VERBATIM from the Betaflight source at tag
// 4.5.1 (commit 77d01ba3b), src/main/target/SITL/target.h:255-277 -- the same
// commit the real FC runs. They are not re-typed from a table, and the
// static_asserts pin every size and offset so a silent layout change cannot
// slip through.
//
// Full write-up with citations: docs/sitl_interface.md
//
// NOTE ON SIGNS: this header deliberately contains NO axis conversions. SITL
// negates gyro Y and Z but all three accelerometer axes (sitl.c:136-144), and
// those two patterns cannot both be a pure frame rotation. Until that is
// settled empirically against a running SITL, any conversion here would be a
// guess. See docs/sitl_interface.md section 5.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace fdt::sitl {

// --- ports (sitl.c:80-83, serial_tcp.c:41,113) -----------------------------

inline constexpr uint16_t kPortPwmRaw = 9001;  ///< SITL -> us, RealFlight format
inline constexpr uint16_t kPortPwm = 9002;     ///< SITL -> us, motor outputs
inline constexpr uint16_t kPortState = 9003;   ///< us -> SITL, SITL binds
inline constexpr uint16_t kPortRc = 9004;      ///< us -> SITL, SITL binds
inline constexpr uint16_t kPortConfiguratorTcp = 5761;  ///< BASE_PORT(5760) + UART1

// --- limits (target.h:238-239) ---------------------------------------------

inline constexpr size_t kMaxRcChannels = 16;
inline constexpr size_t kMaxPwmChannels = 16;

// --- scaling constants (sitl.c:104-106) ------------------------------------
// Recorded for reference; SITL applies these itself, we send SI units.

inline constexpr double kSitlGyroScale = 16.4;              ///< LSB per deg/s
inline constexpr double kSitlAccScale = 256.0 / 9.80665;    ///< LSB per m/s^2, 1 g = 256
inline constexpr double kMotorOutScale = 1000.0;            ///< sitl.c:587 (500.0 with FEATURE_3D)

// --- packets ---------------------------------------------------------------

/// us -> SITL on 9003. target.h:255-263.
struct FdmPacket {
  double timestamp;                        ///< seconds
  double imu_angular_velocity_rpy[3];      ///< rad/s
  double imu_linear_acceleration_xyz[3];   ///< m/s^2, body frame
  double imu_orientation_quat[4];          ///< w, x, y, z -- SCALAR FIRST
  double velocity_xyz[3];                  ///< m/s, earth frame
  double position_xyz[3];                  ///< m, NED from origin
  double pressure;                         ///< Pa
};

static_assert(sizeof(FdmPacket) == 144, "fdm_packet must be 144 bytes");
static_assert(offsetof(FdmPacket, timestamp) == 0);
static_assert(offsetof(FdmPacket, imu_angular_velocity_rpy) == 8);
static_assert(offsetof(FdmPacket, imu_linear_acceleration_xyz) == 32);
static_assert(offsetof(FdmPacket, imu_orientation_quat) == 56);
static_assert(offsetof(FdmPacket, velocity_xyz) == 88);
static_assert(offsetof(FdmPacket, position_xyz) == 112);
static_assert(offsetof(FdmPacket, pressure) == 136);

/// us -> SITL on 9004. target.h:265-268.
/// Channels are RAW MICROSECONDS: readRCSITL returns them unscaled
/// (sitl.c:228-232), so 1000-2000 with 1500 centre.
struct RcPacket {
  double timestamp;                        ///< seconds
  uint16_t channels[kMaxRcChannels];       ///< microseconds
};

static_assert(sizeof(RcPacket) == 40, "rc_packet must be 40 bytes");
static_assert(offsetof(RcPacket, timestamp) == 0);
static_assert(offsetof(RcPacket, channels) == 8);

/// SITL -> us on 9002. target.h:270-272.
/// Throttle ABOVE IDLE, normalised: SITL already subtracted idlePulse
/// (sitl.c:557) and divided by 1000 (sitl.c:587). Do not re-apply the idle
/// offset.
struct ServoPacket {
  float motor_speed[4];  ///< [0, 1] normally; [-1, 1] with FEATURE_3D
};

static_assert(sizeof(ServoPacket) == 16, "servo_packet must be 16 bytes");
static_assert(offsetof(ServoPacket, motor_speed) == 0);

/// SITL -> us on 9001, the RealFlight bridge format. target.h:274-277.
/// Unused by us, defined so the port is not mistaken for something else.
/// Note the 2 bytes of padding after motorCount.
struct ServoPacketRaw {
  uint16_t motorCount;
  float pwm_output_raw[kMaxPwmChannels];  ///< raw PWM, 1100-1900
};

static_assert(sizeof(ServoPacketRaw) == 68, "servo_packet_raw must be 68 bytes");
static_assert(offsetof(ServoPacketRaw, motorCount) == 0);
static_assert(offsetof(ServoPacketRaw, pwm_output_raw) == 4, "expected 2 bytes of padding");

// --- motor order permutation (sitl.c:592-595) ------------------------------
//
//     pwmPkt.motor_speed[3] = motorsPwm[0];   // BF motor index 0 -> slot 3
//     pwmPkt.motor_speed[0] = motorsPwm[1];   // BF motor index 1 -> slot 0
//     pwmPkt.motor_speed[1] = motorsPwm[2];   // BF motor index 2 -> slot 1
//     pwmPkt.motor_speed[2] = motorsPwm[3];   // BF motor index 3 -> slot 2
//
// SITL permutes the motors before sending, "for gazebo8 ArduCopterPlugin"
// (sitl.c:585). The packet is NOT in Betaflight motor order. This is on top of
// Betaflight's own motor index -> physical position mapping, which is a
// separate, still-unverified question (docs/sitl_interface.md section 5).

/// packet slot -> Betaflight motor index (both 0-based).
inline constexpr std::array<size_t, 4> kPacketSlotToBetaflightMotor{1, 2, 3, 0};

/// Betaflight motor index -> packet slot (both 0-based).
inline constexpr std::array<size_t, 4> kBetaflightMotorToPacketSlot{3, 0, 1, 2};

namespace detail {
constexpr bool permutationsAreInverse() {
  for (size_t slot = 0; slot < 4; ++slot) {
    if (kBetaflightMotorToPacketSlot[kPacketSlotToBetaflightMotor[slot]] != slot) return false;
  }
  return true;
}
}  // namespace detail

static_assert(detail::permutationsAreInverse(), "the two motor maps must be inverses");

/// Betaflight motor index (0-based) carried by `slot` of ServoPacket.
constexpr size_t betaflightMotorForPacketSlot(size_t slot) { return kPacketSlotToBetaflightMotor[slot]; }

/// ServoPacket slot carrying Betaflight motor index `motor` (0-based).
constexpr size_t packetSlotForBetaflightMotor(size_t motor) { return kBetaflightMotorToPacketSlot[motor]; }

}  // namespace fdt::sitl
