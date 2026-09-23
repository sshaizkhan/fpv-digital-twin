// Boundary conversion tests.
//
// CLAUDE.md: "Most bugs will be here: wrong axis sign, wrong motor order,
// wrong spin direction. Write a test for each." So there is one per axis and
// one per motor, and each asserts against the value MEASURED from a running
// SITL (docs/sitl_interface.md section 5a) rather than against a re-derivation
// of the same reasoning the code uses.

#include "fdt/sitl_bridge.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace {

fdt::QuadConfig config() { return fdt::loadQuadConfig(std::string(FDT_REPO_ROOT) + "/config/quad.yaml"); }

/// What SITL does to a gyro value, transcribed from sitl.c:142-144. Applying
/// this to what we send must reproduce what Betaflight was measured to read.
Eigen::Vector3d sitlGyroTransform(const Eigen::Vector3d& sent) {
  return {sent.x(), -sent.y(), -sent.z()};
}

/// sitl.c:136-138 -- all three negated.
Eigen::Vector3d sitlAccelTransform(const Eigen::Vector3d& sent) { return -sent; }

constexpr double kGyroCountsPerRadPerSec = 16.4 * 180.0 / M_PI;  // ~939.7
constexpr double kAccCountsPerG = 256.0;

}  // namespace

// --- gyro ------------------------------------------------------------------

TEST(SitlBridge, GyroReachesBetaflightAsOurTrueBodyRates) {
  // The whole point of the pre-negation: after SITL's transform, Betaflight
  // must see exactly the rates we have, not a mirrored version.
  const Eigen::Vector3d rates(1.5, -2.5, 0.75);
  const Eigen::Vector3d at_betaflight = sitlGyroTransform(fdt::sitl::gyroToSitl(rates));
  EXPECT_TRUE(at_betaflight.isApprox(rates, 1e-15))
      << "sent " << fdt::sitl::gyroToSitl(rates).transpose() << ", Betaflight would see "
      << at_betaflight.transpose() << ", we have " << rates.transpose();
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

TEST(SitlBridge, PositiveRollRateArrivesPositiveAtBetaflight) {
  // p > 0 is roll right (docs/coordinate_frames.md §4). Betaflight must agree,
  // or the roll PID drives the wrong way.
  fdt::State s;
  s.angular_velocity = Eigen::Vector3d(4.0, 0.0, 0.0);
  const auto packet = fdt::sitl::toFdmPacket(s, Eigen::Vector3d(0, 0, -fdt::kGravity), 1.0);

  const Eigen::Vector3d sent(packet.imu_angular_velocity_rpy[0], packet.imu_angular_velocity_rpy[1],
                             packet.imu_angular_velocity_rpy[2]);
  EXPECT_GT(sitlGyroTransform(sent).x(), 0.0) << "right roll must reach Betaflight as positive X";
}

TEST(SitlBridge, NoseUpPitchRateArrivesPositiveAtBetaflight) {
  // q > 0 is nose up for us, and Betaflight's pitch is nose-up positive too
  // (imu.c:317). Sent raw this arrives inverted, which is why gyroToSitl
  // negates it.
  fdt::State s;
  s.angular_velocity = Eigen::Vector3d(0.0, 3.0, 0.0);
  const auto packet = fdt::sitl::toFdmPacket(s, Eigen::Vector3d(0, 0, -fdt::kGravity), 1.0);

  const Eigen::Vector3d sent(packet.imu_angular_velocity_rpy[0], packet.imu_angular_velocity_rpy[1],
                             packet.imu_angular_velocity_rpy[2]);
  EXPECT_LT(sent.y(), 0.0) << "must be pre-negated on the wire";
  EXPECT_GT(sitlGyroTransform(sent).y(), 0.0) << "so Betaflight sees nose-up as positive";
}

TEST(SitlBridge, YawRightRateArrivesPositiveAtBetaflight) {
  fdt::State s;
  s.angular_velocity = Eigen::Vector3d(0.0, 0.0, 2.0);  // r > 0 is yaw right
  const auto packet = fdt::sitl::toFdmPacket(s, Eigen::Vector3d(0, 0, -fdt::kGravity), 1.0);

  const Eigen::Vector3d sent(packet.imu_angular_velocity_rpy[0], packet.imu_angular_velocity_rpy[1],
                             packet.imu_angular_velocity_rpy[2]);
  EXPECT_LT(sent.z(), 0.0);
  EXPECT_GT(sitlGyroTransform(sent).z(), 0.0) << "Betaflight must see yaw-right as positive Z";
}

// --- accelerometer ---------------------------------------------------------

TEST(SitlBridge, RestingQuadReadsPlusOneGOnZAtBetaflight) {
  // The single most checkable fact at this boundary, and it was measured:
  // our [0, 0, -9.80665] arrives as +256 counts on Z, which is what a real
  // level FC reads.
  fdt::State s;
  const Eigen::Vector3d at_rest(0.0, 0.0, -fdt::kGravity);
  const auto packet = fdt::sitl::toFdmPacket(s, at_rest, 1.0);

  const Eigen::Vector3d sent(packet.imu_linear_acceleration_xyz[0], packet.imu_linear_acceleration_xyz[1],
                             packet.imu_linear_acceleration_xyz[2]);
  const Eigen::Vector3d counts = sitlAccelTransform(sent) * (kAccCountsPerG / fdt::kGravity);

  EXPECT_NEAR(counts.z(), +256.0, 0.5) << "a level FC reads +1 g on Z";
  EXPECT_NEAR(counts.x(), 0.0, 0.5);
  EXPECT_NEAR(counts.y(), 0.0, 0.5);
}

TEST(SitlBridge, AccelerometerIsSentUnchanged) {
  const Eigen::Vector3d f(1.0, -2.0, -9.0);
  EXPECT_TRUE(fdt::sitl::accelToSitl(f).isApprox(f, 1e-15))
      << "SITL's own negation is what produces Betaflight's expected reading";
}

TEST(SitlBridge, FreeFallReadsZeroAtBetaflight) {
  fdt::State s;
  const auto packet = fdt::sitl::toFdmPacket(s, Eigen::Vector3d::Zero(), 1.0);
  for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(packet.imu_linear_acceleration_xyz[i], 0.0);
}

// --- attitude --------------------------------------------------------------

TEST(SitlBridge, AttitudeIsScalarFirst) {
  Eigen::Quaterniond q(0.5, 0.5, 0.5, 0.5);  // Eigen ctor is (w, x, y, z)
  const auto out = fdt::sitl::attitudeToSitl(q);
  EXPECT_DOUBLE_EQ(out[0], q.w()) << "target.h:259 says w, x, y, z";
}

TEST(SitlBridge, PitchIsNegatedButRollAndYawAreNot) {
  // Measured: +30 deg about fdm x -> BF roll +30; about y -> BF pitch -30;
  // about z -> BF yaw +30. Only pitch needs correcting.
  const double a = 30.0 * M_PI / 180.0;
  const double s = std::sin(a / 2.0);

  const auto roll = fdt::sitl::attitudeToSitl(Eigen::Quaterniond(std::cos(a / 2), s, 0, 0));
  EXPECT_NEAR(roll[1], +s, 1e-15) << "roll passes through";

  const auto pitch = fdt::sitl::attitudeToSitl(Eigen::Quaterniond(std::cos(a / 2), 0, s, 0));
  EXPECT_NEAR(pitch[2], -s, 1e-15) << "pitch must be negated or Betaflight reads it upside down";

  const auto yaw = fdt::sitl::attitudeToSitl(Eigen::Quaterniond(std::cos(a / 2), 0, 0, s));
  EXPECT_NEAR(yaw[3], +s, 1e-15) << "yaw passes through";
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
