// IMU synthesis: what the gyro and accelerometer hand to Betaflight.
//
// The accelerometer reports SPECIFIC FORCE, not acceleration: everything
// except gravity, divided by mass, in the body frame. A quad sitting on the
// bench reads [0, 0, -9.80665] in FRD (the bench pushes up, and up is -z), and
// a quad in free fall reads zero. Getting this backwards is one of the
// standard ways to make a sim that looks right and flies wrong.
//
// Noise is drawn from a seeded mt19937_64 so a replay is reproducible bit for
// bit -- the project needs deterministic headless runs for Phase 4.
#pragma once

#include "fdt/quad_config.hpp"
#include "fdt/types.hpp"

#include <cstdint>
#include <random>

namespace fdt {

struct ImuSample {
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();   ///< rad/s, body frame
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();  ///< m/s^2 specific force, body frame
};

/// Specific force in the body frame, given the world-frame acceleration of the
/// CG and the attitude. `f = R_wb^T * (a_world - g_world)`.
Eigen::Vector3d specificForceBody(const State& state, const Eigen::Vector3d& acceleration_world);

class ImuModel {
 public:
  ImuModel(const Imu& config, uint64_t seed = 0);

  /// Corrupt the true values with bias and noise, and advance the bias random
  /// walk by dt. Discrete noise sigma is `density / sqrt(dt)`, which is the
  /// usual continuous-to-discrete conversion for a noise density.
  ImuSample sample(const Eigen::Vector3d& angular_velocity_body,
                   const Eigen::Vector3d& specific_force_body, double dt);

  void reset();

  const Eigen::Vector3d& gyroBias() const { return gyro_bias_; }
  const Eigen::Vector3d& accelBias() const { return accel_bias_; }

 private:
  Imu config_;
  uint64_t seed_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
};

}  // namespace fdt
