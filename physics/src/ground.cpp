#include "fdt/ground.hpp"

#include <algorithm>
#include <limits>

namespace fdt {
namespace {

/// Softening speed for the Coulomb friction direction, m/s. Purely numerical:
/// without it the friction direction flips every step as the tangential speed
/// crosses zero and a parked quad buzzes. Not a physical parameter, which is
/// why it does not live in quad.yaml.
constexpr double kTangentialRegularisation = 0.05;

}  // namespace

GroundModel::GroundModel(const Ground& config, const Motors& motors) : config_(config) {
  for (const auto id : kAllMotorIds) {
    const size_t i = static_cast<size_t>(id);
    // Body +z is down, so the feet are at positive z relative to the CG.
    contact_points_[i] = motors.at(id).position.value + Eigen::Vector3d(0.0, 0.0, config_.stand_height.value);
  }
}

double GroundModel::penetration(const State& state) const {
  const Eigen::Matrix3d R = state.bodyToWorld();
  double deepest = -std::numeric_limits<double>::infinity();
  for (const auto& r : contact_points_) {
    const double z = state.position.z() + (R * r).z();
    deepest = std::max(deepest, z - config_.height.value);
  }
  return deepest;
}

bool GroundModel::inContact(const State& state) const { return penetration(state) > 0.0; }

Wrench GroundModel::wrench(const State& state) const {
  const Eigen::Matrix3d R = state.bodyToWorld();
  const Eigen::Matrix3d Rt = R.transpose();

  Wrench w;
  for (const auto& r_body : contact_points_) {
    const Eigen::Vector3d r_world = R * r_body;
    const double pen = state.position.z() + r_world.z() - config_.height.value;
    if (pen <= 0.0) continue;

    // Velocity of this contact point in the world frame.
    const Eigen::Vector3d v_point = state.velocity + R * state.angular_velocity.cross(r_body);

    // Penalty spring plus damper, clamped at zero: the ground pushes, it never
    // pulls, however fast the quad is leaving.
    const double normal = std::max(
        0.0, config_.contact_stiffness.value * pen + config_.contact_damping.value * v_point.z());
    if (normal <= 0.0) continue;

    Eigen::Vector3d force_world(0.0, 0.0, -normal);  // up is -z

    const Eigen::Vector3d v_tangent(v_point.x(), v_point.y(), 0.0);
    const double speed = v_tangent.norm();
    if (speed > 0.0) {
      force_world -= config_.friction.value * normal * v_tangent / (speed + kTangentialRegularisation);
    }

    const Eigen::Vector3d force_body = Rt * force_world;
    w.force += force_body;
    w.torque += r_body.cross(force_body);
  }
  return w;
}

}  // namespace fdt
