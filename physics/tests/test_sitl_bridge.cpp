// Boundary conversion tests.
//
// CLAUDE.md: "Most bugs will be here: wrong axis sign, wrong motor order,
// wrong spin direction. Write a test for each." So there is one per axis and
// one per motor, and each asserts against the value MEASURED from a running
// SITL (docs/sitl_interface.md section 5a) rather than against a re-derivation
// of the same reasoning the code uses.

#include "fdt/imu.hpp"
#include "fdt/sitl_bridge.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace {

fdt::QuadConfig config() { return fdt::loadQuadConfig(std::string(FDT_REPO_ROOT) + "/config/quad.yaml"); }

/// What SITL does to a gyro value, transcribed from sitl.c:142-144.
Eigen::Vector3d sitlGyroTransform(const Eigen::Vector3d& sent) {
  return {sent.x(), -sent.y(), -sent.z()};
}

/// sitl.c:136-138 -- all three negated.
Eigen::Vector3d sitlAccelTransform(const Eigen::Vector3d& sent) { return -sent; }

/// Betaflight attitude in degrees, as MSP_ATTITUDE would report it.
struct BfAttitude {
  double roll, pitch, yaw;
};

/// What Betaflight does with the packet quaternion under SITL, transcribed
/// from imuSetAttitudeQuat (imu.c:777), imuComputeRotationMatrix including its
/// SIMULATOR_BUILD patch (imu.c:146-165), and imuUpdateEulerAngles
/// (imu.c:310-323). Independent of the bridge's reasoning: it is Betaflight's
/// code, so the bridge output run through it must yield Betaflight's own
/// convention for the attitude we mean.
BfAttitude betaflightEuler(const std::array<double, 4>& q) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double r00 = 1.0 - 2.0 * y * y - 2.0 * z * z;
  const double r10 = -2.0 * (x * y + w * z);  // SIMULATOR_BUILD patch
  const double r20 = -2.0 * (x * z - w * y);  // SIMULATOR_BUILD patch
  const double r21 = 2.0 * (y * z + w * x);
  const double r22 = 1.0 - 2.0 * x * x - 2.0 * y * y;
  constexpr double kDeg = 180.0 / M_PI;
  BfAttitude a{};
  a.roll = std::atan2(r21, r22) * kDeg;
  a.pitch = (M_PI / 2.0 - std::acos(-r20)) * kDeg;
  a.yaw = -std::atan2(r10, r00) * kDeg;
  if (a.yaw < 0.0) a.yaw += 360.0;
  return a;
}

/// Our NED/FRD attitude from ZYX Euler angles (yaw, then pitch, then roll).
Eigen::Quaterniond fromEuler(double roll_deg, double pitch_deg, double yaw_deg) {
  const double d = M_PI / 180.0;
  return Eigen::AngleAxisd(yaw_deg * d, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch_deg * d, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll_deg * d, Eigen::Vector3d::UnitX());
}

constexpr double kGyroCountsPerRadPerSec = 16.4 * 180.0 / M_PI;  // ~939.7
constexpr double kAccCountsPerG = 256.0;

Eigen::Vector3d sentGyro(const fdt::sitl::FdmPacket& p) {
  return {p.imu_angular_velocity_rpy[0], p.imu_angular_velocity_rpy[1], p.imu_angular_velocity_rpy[2]};
}

Eigen::Vector3d sentAccel(const fdt::sitl::FdmPacket& p) {
  return {p.imu_linear_acceleration_xyz[0], p.imu_linear_acceleration_xyz[1],
          p.imu_linear_acceleration_xyz[2]};
}

}  // namespace

// --- gyro ------------------------------------------------------------------
//
// Betaflight's body frame is FLU. The rate PID drives the gyro toward the
// setpoint, so the sign each stick produces fixes the sign of each axis:
// forward pitch stick -> positive rcCommand (rc.c:698) and forward stick means
// nose down, so +Y is nose DOWN; right yaw stick -> negative rcCommand
// (rc.c:705), so +Z is yaw LEFT.

TEST(SitlBridge, GyroIsSentUnchanged) {
  const Eigen::Vector3d rates(1.5, -2.5, 0.75);
  EXPECT_TRUE(fdt::sitl::gyroToSitl(rates).isApprox(rates, 1e-15))
      << "SITL's own Y/Z negation is the FRD -> FLU conversion; do not cancel it";
}

TEST(SitlBridge, GyroPerAxisSignsMatchTheMeasurement) {
  // Measured: sending +1 rad/s on rpy[0] read +939 at Betaflight, rpy[1] read
  // -939, rpy[2] read -939.
  struct Case {
    int axis;
    double measured_counts;
    const char* description;
  };
  const Case cases[] = {
      {0, +939.0, "fdm x -> BF X, same sign"},
      {1, -939.0, "fdm y -> BF Y, negated"},
      {2, -939.0, "fdm z -> BF Z, negated"},
  };

  for (const auto& c : cases) {
    Eigen::Vector3d raw = Eigen::Vector3d::Zero();
    raw[c.axis] = 1.0;  // what the probe put on the wire
    const Eigen::Vector3d at_bf = sitlGyroTransform(raw) * kGyroCountsPerRadPerSec;
    EXPECT_NEAR(at_bf[c.axis], c.measured_counts, 1.0) << c.description;
  }
}

TEST(SitlBridge, RollRightArrivesPositiveAtBetaflight) {
  // p > 0 is roll right (docs/coordinate_frames.md §4); right roll stick is a
  // positive setpoint in Betaflight.
  fdt::State s;
  s.angular_velocity = Eigen::Vector3d(4.0, 0.0, 0.0);
  const auto packet = fdt::sitl::toFdmPacket(s, Eigen::Vector3d(0, 0, -fdt::kGravity), 1.0);
  EXPECT_GT(sitlGyroTransform(sentGyro(packet)).x(), 0.0);
}

TEST(SitlBridge, NoseUpArrivesNegativeAtBetaflight) {
  // q > 0 is nose up for us. Betaflight's +Y is nose DOWN (forward stick is a
  // positive setpoint), so nose up must arrive negative or the pitch loop is
  // positive feedback.
  fdt::State s;
  s.angular_velocity = Eigen::Vector3d(0.0, 3.0, 0.0);
  const auto packet = fdt::sitl::toFdmPacket(s, Eigen::Vector3d(0, 0, -fdt::kGravity), 1.0);
  EXPECT_LT(sitlGyroTransform(sentGyro(packet)).y(), 0.0);
}

TEST(SitlBridge, YawRightArrivesNegativeAtBetaflight) {
  // r > 0 is yaw right for us. Right yaw stick is a NEGATIVE setpoint in
  // Betaflight (rc.c:705), so yaw right must arrive negative.
  fdt::State s;
  s.angular_velocity = Eigen::Vector3d(0.0, 0.0, 2.0);
  const auto packet = fdt::sitl::toFdmPacket(s, Eigen::Vector3d(0, 0, -fdt::kGravity), 1.0);
  EXPECT_LT(sitlGyroTransform(sentGyro(packet)).z(), 0.0);
}

// --- accelerometer ---------------------------------------------------------
//
// Betaflight's accel shares its gyro's FLU frame: it expects (fx, -fy, -fz)
// of our FRD specific force.

TEST(SitlBridge, RestingQuadReadsPlusOneGOnZAtBetaflight) {
  // Measured: a level FC reads +256 counts on Z.
  fdt::State s;
  const auto packet = fdt::sitl::toFdmPacket(s, Eigen::Vector3d(0.0, 0.0, -fdt::kGravity), 1.0);
  const Eigen::Vector3d counts = sitlAccelTransform(sentAccel(packet)) * (kAccCountsPerG / fdt::kGravity);

  EXPECT_NEAR(counts.z(), +256.0, 0.5) << "a level FC reads +1 g on Z";
  EXPECT_NEAR(counts.x(), 0.0, 0.5);
  EXPECT_NEAR(counts.y(), 0.0, 0.5);
}

TEST(SitlBridge, NoseDownAtRestReadsNegativeXAtBetaflight) {
  // Tilted nose down by 20 deg, at rest. Gravity then has a component along
  // the nose, and in FLU the specific force is (-g sin, 0, g cos): X negative.
  fdt::State s;
  s.orientation = fromEuler(0.0, -20.0, 0.0);
  const Eigen::Vector3d f = fdt::specificForceBody(s, Eigen::Vector3d::Zero());
  ASSERT_LT(f.x(), 0.0) << "our FRD specific force is also -g sin on X";

  const auto packet = fdt::sitl::toFdmPacket(s, f, 1.0);
  const Eigen::Vector3d counts = sitlAccelTransform(sentAccel(packet)) * (kAccCountsPerG / fdt::kGravity);
  const double expected = -256.0 * std::sin(20.0 * M_PI / 180.0);
  EXPECT_NEAR(counts.x(), expected, 0.5) << "nose down must read negative X, as on a real FC";
  EXPECT_GT(counts.z(), 0.0);
}

TEST(SitlBridge, RightWingDownAtRestReadsPositiveYAtBetaflight) {
  // Rolled right 20 deg, at rest. Specific force points world-up, which now
  // leans toward the raised left wing: +Y in FLU, (0, +g sin, g cos).
  fdt::State s;
  s.orientation = fromEuler(20.0, 0.0, 0.0);
  const Eigen::Vector3d f = fdt::specificForceBody(s, Eigen::Vector3d::Zero());

  const auto packet = fdt::sitl::toFdmPacket(s, f, 1.0);
  const Eigen::Vector3d counts = sitlAccelTransform(sentAccel(packet)) * (kAccCountsPerG / fdt::kGravity);
  EXPECT_NEAR(counts.y(), +256.0 * std::sin(20.0 * M_PI / 180.0), 0.5);
}

TEST(SitlBridge, FreeFallReadsZeroAtBetaflight) {
  fdt::State s;
  const auto packet = fdt::sitl::toFdmPacket(s, Eigen::Vector3d::Zero(), 1.0);
  for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(packet.imu_linear_acceleration_xyz[i], 0.0);
}

// --- attitude --------------------------------------------------------------
//
// Betaflight reports roll right positive, pitch nose DOWN positive (angle mode
// drives attitude toward a positive target on forward stick, pid.c:387-395),
// and yaw as a 0-360 compass heading.

TEST(SitlBridge, AttitudeIsSentUnchangedScalarFirst) {
  const Eigen::Quaterniond q = fromEuler(10.0, -20.0, 30.0);
  const auto out = fdt::sitl::attitudeToSitl(q);
  EXPECT_DOUBLE_EQ(out[0], q.w()) << "target.h:259 says w, x, y, z";
  EXPECT_DOUBLE_EQ(out[1], q.x());
  EXPECT_DOUBLE_EQ(out[2], q.y());
  EXPECT_DOUBLE_EQ(out[3], q.z());
}

TEST(SitlBridge, SingleAxisAttitudesMatchTheMeasurement) {
  // Measured: +30 deg about fdm x -> BF roll +30; about y -> BF pitch -30;
  // about z -> BF yaw +30. Checks the imu.c transcription against reality.
  const BfAttitude roll = betaflightEuler(fdt::sitl::attitudeToSitl(fromEuler(30.0, 0.0, 0.0)));
  const BfAttitude pitch = betaflightEuler(fdt::sitl::attitudeToSitl(fromEuler(0.0, 30.0, 0.0)));
  const BfAttitude yaw = betaflightEuler(fdt::sitl::attitudeToSitl(fromEuler(0.0, 0.0, 30.0)));
  EXPECT_NEAR(roll.roll, +30.0, 1e-9);
  EXPECT_NEAR(pitch.pitch, -30.0, 1e-9) << "nose up is negative in Betaflight";
  EXPECT_NEAR(yaw.yaw, +30.0, 1e-9);
}

TEST(SitlBridge, CombinedAttitudeArrivesInBetaflightConvention) {
  // Single-axis cases cannot tell a real frame change from a component hack.
  // Roll right 25, nose up 15, heading 250.
  const BfAttitude a = betaflightEuler(fdt::sitl::attitudeToSitl(fromEuler(25.0, 15.0, 250.0)));
  EXPECT_NEAR(a.roll, 25.0, 1e-9);
  EXPECT_NEAR(a.pitch, -15.0, 1e-9) << "nose up is negative in Betaflight";
  EXPECT_NEAR(a.yaw, 250.0, 1e-9);
}

TEST(SitlBridge, AttitudeStaysAUnitQuaternion) {
  Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()));
  const auto out = fdt::sitl::attitudeToSitl(q);
  const double norm = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
  EXPECT_NEAR(norm, 1.0, 1e-12);
}

// --- position and velocity -------------------------------------------------

TEST(SitlBridge, PositionAndVelocityPassThroughBecauseBothAreNed) {
  fdt::State s;
  s.position = Eigen::Vector3d(10.0, -5.0, -30.0);  // 30 m up, NED
  s.velocity = Eigen::Vector3d(1.0, 2.0, -3.0);
  const auto p = fdt::sitl::toFdmPacket(s, Eigen::Vector3d(0, 0, -fdt::kGravity), 2.5);

  EXPECT_DOUBLE_EQ(p.position_xyz[2], -30.0) << "target.h:261 says NED, as is ours";
  EXPECT_DOUBLE_EQ(p.velocity_xyz[1], 2.0);
  EXPECT_DOUBLE_EQ(p.timestamp, 2.5);
  EXPECT_DOUBLE_EQ(p.pressure, fdt::sitl::kSeaLevelPressurePa) << "baro must be fed or arming never clears";
}

// --- motors ----------------------------------------------------------------

TEST(SitlBridge, EachPacketSlotReachesTheRightPhysicalMotor) {
  const auto cfg = config();
  for (size_t slot = 0; slot < 4; ++slot) {
    fdt::sitl::ServoPacket packet{};
    packet.motor_speed[slot] = 0.5f;

    const auto commands = fdt::sitl::motorCommands(packet, cfg.motors);

    // Exactly one motor spun up, and it is the one both permutations name.
    const size_t bf_index = fdt::sitl::betaflightMotorForPacketSlot(slot);
    const fdt::MotorId expected = cfg.motors.motorForBetaflightIndex(static_cast<int>(bf_index) + 1);

    int spinning = 0;
    for (size_t i = 0; i < 4; ++i) {
      if (commands[i] > 0.0) ++spinning;
    }
    EXPECT_EQ(spinning, 1) << "slot " << slot << " should drive exactly one motor";
    EXPECT_NEAR(commands[static_cast<size_t>(expected)], 0.5, 1e-6)
        << "slot " << slot << " -> BF motor " << bf_index + 1 << " -> " << fdt::toString(expected);
  }
}

TEST(SitlBridge, TheFourMotorsAreAPermutationNotACollision) {
  const auto cfg = config();
  fdt::sitl::ServoPacket packet{};
  packet.motor_speed[0] = 0.1f;
  packet.motor_speed[1] = 0.2f;
  packet.motor_speed[2] = 0.3f;
  packet.motor_speed[3] = 0.4f;

  const auto commands = fdt::sitl::motorCommands(packet, cfg.motors);

  std::array<double, 4> sorted = commands;
  std::sort(sorted.begin(), sorted.end());
  // motor_speed is float on the wire, so compare at float precision.
  EXPECT_NEAR(sorted[0], 0.1, 1e-6);
  EXPECT_NEAR(sorted[1], 0.2, 1e-6);
  EXPECT_NEAR(sorted[2], 0.3, 1e-6);
  EXPECT_NEAR(sorted[3], 0.4, 1e-6);
}

TEST(SitlBridge, MotorCommandsAreClampedToTheValidRange) {
  const auto cfg = config();
  fdt::sitl::ServoPacket packet{};
  packet.motor_speed[0] = 1.5f;   // saturated mixer output
  packet.motor_speed[1] = -0.5f;  // 3D mode, which we do not fly
  const auto commands = fdt::sitl::motorCommands(packet, cfg.motors);
  for (double c : commands) {
    EXPECT_GE(c, 0.0);
    EXPECT_LE(c, 1.0);
  }
}

// --- RC --------------------------------------------------------------------

TEST(SitlBridge, NeutralRcIsCentredSticksAndLowThrottle) {
  const auto rc = fdt::sitl::RcChannels::neutral();
  EXPECT_EQ(rc.us[fdt::sitl::RcChannels::kRoll], 1500);
  EXPECT_EQ(rc.us[fdt::sitl::RcChannels::kPitch], 1500);
  EXPECT_EQ(rc.us[fdt::sitl::RcChannels::kYaw], 1500);
  EXPECT_EQ(rc.us[fdt::sitl::RcChannels::kThrottle], 1000) << "neutral must never be armed-and-throttled";
  for (size_t aux = 0; aux < 4; ++aux) EXPECT_EQ(rc.aux(aux), 1000);
}

TEST(SitlBridge, AuxOneIsChannelFour) {
  auto rc = fdt::sitl::RcChannels::neutral();
  rc.setAux(0, 1800);
  EXPECT_EQ(rc.us[4], 1800) << "sitl.c:249-251 labels channels 4-7 as AUX1-4";
  EXPECT_EQ(rc.aux(0), 1800);
}

TEST(SitlBridge, RcPacketCarriesMicrosecondsUnscaled) {
  auto rc = fdt::sitl::RcChannels::neutral();
  rc.setAetr(1600, 1400, 1250, 1500);
  const auto packet = fdt::sitl::toRcPacket(rc, 4.0);

  EXPECT_DOUBLE_EQ(packet.timestamp, 4.0);
  EXPECT_EQ(packet.channels[0], 1600) << "readRCSITL returns these unscaled (sitl.c:228-232)";
  EXPECT_EQ(packet.channels[1], 1400);
  EXPECT_EQ(packet.channels[2], 1250);
  EXPECT_EQ(packet.channels[3], 1500);
}

TEST(SitlBridge, OutOfRangeAuxIndexIsIgnoredNotUndefined) {
  auto rc = fdt::sitl::RcChannels::neutral();
  rc.setAux(99, 2000);  // must not write past the array
  for (size_t i = 0; i < fdt::sitl::kMaxRcChannels; ++i) EXPECT_LE(rc.us[i], 1500);
}
