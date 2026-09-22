#include "fdt/imu.hpp"

#include <cmath>

namespace fdt {

Eigen::Vector3d specificForceBody(const State& state, const Eigen::Vector3d& acceleration_world) {
  // An accelerometer cannot feel gravity: it measures everything else.
  return state.orientation.conjugate() * (acceleration_world - gravityWorld());
}

ImuModel::ImuModel(const Imu& config, uint64_t seed) : config_(config), seed_(seed) { reset(); }

void ImuModel::reset() {
  rng_.seed(seed_);
  normal_.reset();
  gyro_bias_ = config_.gyro_bias.value;
  accel_bias_ = config_.accel_bias.value;
}

ImuSample ImuModel::sample(const Eigen::Vector3d& angular_velocity_body,
                           const Eigen::Vector3d& specific_force_body, double dt) {
  // Continuous noise density -> per-sample sigma.
  const double root_dt = (dt > 0.0) ? std::sqrt(dt) : 1.0;
  const double gyro_sigma = config_.gyro_noise_density.value / root_dt;
  const double accel_sigma = config_.accel_noise_density.value / root_dt;
  const double walk_sigma = config_.gyro_bias_walk.value * root_dt;

  ImuSample out;
  for (int i = 0; i < 3; ++i) {
    gyro_bias_[i] += walk_sigma * normal_(rng_);
    out.gyro[i] = angular_velocity_body[i] + gyro_bias_[i] + gyro_sigma * normal_(rng_);
    out.accel[i] = specific_force_body[i] + accel_bias_[i] + accel_sigma * normal_(rng_);
  }
  return out;
}

}  // namespace fdt
