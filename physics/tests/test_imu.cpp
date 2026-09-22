// IMU synthesis tests.

#include "fdt/imu.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

fdt::QuadConfig config() { return fdt::loadQuadConfig(std::string(FDT_REPO_ROOT) + "/config/quad.yaml"); }

/// Same IMU, all noise and bias switched off, so the plumbing can be tested
/// without statistics.
fdt::Imu noiseless(fdt::Imu imu) {
  imu.gyro_noise_density.value = 0.0;
  imu.accel_noise_density.value = 0.0;
  imu.gyro_bias_walk.value = 0.0;
  imu.gyro_bias.value.setZero();
  imu.accel_bias.value.setZero();
  return imu;
}

}  // namespace

// --- specific force -------------------------------------------------------

TEST(Imu, AtRestOnTheBenchTheAccelerometerReadsOneGUp) {
  fdt::State s;  // level, motionless, at the origin
  const Eigen::Vector3d f = fdt::specificForceBody(s, Eigen::Vector3d::Zero());

  EXPECT_NEAR(f.x(), 0.0, 1e-15);
  EXPECT_NEAR(f.y(), 0.0, 1e-15);
  EXPECT_NEAR(f.z(), -fdt::kGravity, 1e-12) << "FRD: up is -z, so a level quad at rest reads -g";
}

TEST(Imu, InFreeFallTheAccelerometerReadsZero) {
  fdt::State s;
  const Eigen::Vector3d f = fdt::specificForceBody(s, fdt::gravityWorld());
  EXPECT_NEAR(f.norm(), 0.0, 1e-12) << "free fall is weightless";
}

TEST(Imu, SpecificForceRotatesWithTheBody) {
  // An accelerometer at rest reads +1 g along whichever BODY axis is pointing
  // up. Level, that axis is -z, giving the familiar [0, 0, -g].
  //
  // Rolled 90 degrees right, the right wing (body +y) is pointing at the
  // ground and body -y is pointing at the sky, so the reading moves to
  // [0, -g, 0]. Rolling the other way must put it at [0, +g, 0].
  fdt::State right;
  right.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
  const Eigen::Vector3d f = fdt::specificForceBody(right, Eigen::Vector3d::Zero());

  EXPECT_NEAR(f.x(), 0.0, 1e-12);
  EXPECT_NEAR(f.y(), -fdt::kGravity, 1e-12);
  EXPECT_NEAR(f.z(), 0.0, 1e-12);
  EXPECT_NEAR(f.norm(), fdt::kGravity, 1e-12) << "magnitude must be preserved by a rotation";

  fdt::State left;
  left.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX()));
  EXPECT_NEAR(fdt::specificForceBody(left, Eigen::Vector3d::Zero()).y(), fdt::kGravity, 1e-12);

  // Nose up 90 degrees puts it on the body +x axis.
  fdt::State nose_up;
  nose_up.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
  EXPECT_NEAR(fdt::specificForceBody(nose_up, Eigen::Vector3d::Zero()).x(), fdt::kGravity, 1e-12);
}

TEST(Imu, UpwardThrustShowsAsIncreasedLoadFactor) {
  // Climbing at 2 g: the accelerometer must read 2 g, not 1 g and not 3 g.
  fdt::State s;
  const Eigen::Vector3d a_world(0.0, 0.0, -fdt::kGravity);  // accelerating upward at 1 g
  const Eigen::Vector3d f = fdt::specificForceBody(s, a_world);
  EXPECT_NEAR(f.z(), -2.0 * fdt::kGravity, 1e-12);
}

// --- plumbing -------------------------------------------------------------

TEST(Imu, NoiselessImuPassesTheTruthThrough) {
  fdt::ImuModel imu(noiseless(config().imu), 1);
  const Eigen::Vector3d omega(1.5, -2.5, 0.75);
  const Eigen::Vector3d f(0.1, -0.2, -9.7);

  const fdt::ImuSample s = imu.sample(omega, f, 1.0 / 8000.0);
  EXPECT_TRUE(s.gyro.isApprox(omega, 1e-15));
  EXPECT_TRUE(s.accel.isApprox(f, 1e-15));
}

TEST(Imu, ConfiguredBiasIsAddedToEveryReading) {
  fdt::Imu cfg = noiseless(config().imu);
  cfg.gyro_bias.value = Eigen::Vector3d(0.01, -0.02, 0.03);
  cfg.accel_bias.value = Eigen::Vector3d(0.1, 0.2, -0.3);

  fdt::ImuModel imu(cfg, 1);
  const fdt::ImuSample s = imu.sample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1.0 / 8000.0);
  EXPECT_TRUE(s.gyro.isApprox(cfg.gyro_bias.value, 1e-15));
  EXPECT_TRUE(s.accel.isApprox(cfg.accel_bias.value, 1e-15));
}

// --- noise ----------------------------------------------------------------

TEST(Imu, NoiseIsBitIdenticalForTheSameSeed) {
  const auto cfg = config();
  fdt::ImuModel a(cfg.imu, 12345), b(cfg.imu, 12345);
  const double dt = 1.0 / 8000.0;

  for (int i = 0; i < 2000; ++i) {
    const fdt::ImuSample sa = a.sample(Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero(), dt);
    const fdt::ImuSample sb = b.sample(Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero(), dt);
    ASSERT_EQ(sa.gyro.x(), sb.gyro.x()) << "at sample " << i;
    ASSERT_EQ(sa.accel.z(), sb.accel.z()) << "at sample " << i;
  }
}

TEST(Imu, DifferentSeedsGiveDifferentNoise) {
  const auto cfg = config();
  fdt::ImuModel a(cfg.imu, 1), b(cfg.imu, 2);
  const double dt = 1.0 / 8000.0;

  bool differed = false;
  for (int i = 0; i < 100 && !differed; ++i) {
    const fdt::ImuSample sa = a.sample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);
    const fdt::ImuSample sb = b.sample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);
    differed = (sa.gyro - sb.gyro).norm() > 0.0;
  }
  EXPECT_TRUE(differed);
}

TEST(Imu, NoiseHasTheConfiguredStandardDeviation) {
  auto cfg = config();
  cfg.imu.gyro_bias_walk.value = 0.0;  // isolate the white noise
  cfg.imu.gyro_bias.value.setZero();

  fdt::ImuModel imu(cfg.imu, 7);
  const double dt = 1.0 / 8000.0;
  const int n = 200000;

  double sum = 0.0, sum_sq = 0.0;
  for (int i = 0; i < n; ++i) {
    const double g = imu.sample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt).gyro.x();
    sum += g;
    sum_sq += g * g;
  }
  const double mean = sum / n;
  const double stdev = std::sqrt(sum_sq / n - mean * mean);
  const double expected = cfg.imu.gyro_noise_density.value / std::sqrt(dt);

  EXPECT_NEAR(mean, 0.0, 5.0 * expected / std::sqrt(static_cast<double>(n)));
  EXPECT_NEAR(stdev / expected, 1.0, 0.02) << "expected sigma " << expected << ", got " << stdev;
}

TEST(Imu, BiasRandomWalkStartsAtTheConfiguredBiasAndDrifts) {
  auto cfg = config();
  cfg.imu.gyro_bias.value = Eigen::Vector3d(0.05, 0.0, 0.0);
  cfg.imu.gyro_bias_walk.value = 1e-3;  // exaggerated so the drift is visible
  cfg.imu.gyro_noise_density.value = 0.0;

  fdt::ImuModel imu(cfg.imu, 3);
  EXPECT_TRUE(imu.gyroBias().isApprox(cfg.imu.gyro_bias.value, 1e-15));

  const double dt = 1.0 / 8000.0;
  for (int i = 0; i < 80000; ++i) imu.sample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);

  EXPECT_NE(imu.gyroBias().x(), cfg.imu.gyro_bias.value.x()) << "bias must random-walk";
  // sigma over 10 s is walk*sqrt(10) = 3.2e-3; a 20x excursion means a bug.
  EXPECT_LT(std::abs(imu.gyroBias().x() - cfg.imu.gyro_bias.value.x()), 20.0 * 1e-3 * std::sqrt(10.0));
}

TEST(Imu, ResetRestoresTheInitialBiasAndNoiseStream) {
  const auto cfg = config();
  fdt::ImuModel imu(cfg.imu, 99);
  const double dt = 1.0 / 8000.0;

  const fdt::ImuSample first = imu.sample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);
  for (int i = 0; i < 1000; ++i) imu.sample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);

  imu.reset();
  const fdt::ImuSample again = imu.sample(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);
  EXPECT_EQ(first.gyro.x(), again.gyro.x()) << "reset must rewind the noise stream";
}
