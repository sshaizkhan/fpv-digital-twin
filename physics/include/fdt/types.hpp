// Core state and wrench types.
//
// Frames and signs: docs/coordinate_frames.md
//   world NED (x North, y East, z Down), body FRD (x fwd, y right, z down),
//   body origin at the CG. SI units throughout.
#pragma once

#include <Eigen/Geometry>

namespace fdt {

/// Standard gravity, m/s^2.
inline constexpr double kGravity = 9.80665;

/// Gravity in the world frame. NED means it points along +z.
inline Eigen::Vector3d gravityWorld() { return Eigen::Vector3d(0.0, 0.0, kGravity); }

/// Rigid-body state. 13 numbers: 3 position, 3 velocity, 4 quaternion, 3 rates.
struct State {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();     ///< world NED, m
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();     ///< world NED, m/s
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  ///< q_wb, body -> world
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();       ///< body frame, rad/s

  /// Height above the world origin plane. NED z is down, so altitude is -z.
  double altitude() const { return -position.z(); }

  /// Velocity expressed in the body frame.
  Eigen::Vector3d velocityBody() const { return orientation.conjugate() * velocity; }

  Eigen::Matrix3d bodyToWorld() const { return orientation.toRotationMatrix(); }
};

/// Force and torque acting on the body. **Both are expressed in the BODY
/// frame** — gravity is the one exception and is added by the integrator in
/// the world frame, so that models never have to know the attitude.
struct Wrench {
  Eigen::Vector3d force = Eigen::Vector3d::Zero();   ///< body frame, N
  Eigen::Vector3d torque = Eigen::Vector3d::Zero();  ///< body frame, N*m

  Wrench& operator+=(const Wrench& o) {
    force += o.force;
    torque += o.torque;
    return *this;
  }
  friend Wrench operator+(Wrench a, const Wrench& b) { return a += b; }
};

/// Mass and inertia about the CG, body axes.
struct InertiaProperties {
  double mass = 1.0;                                          ///< kg
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();      ///< kg*m^2
  Eigen::Matrix3d inertia_inv = Eigen::Matrix3d::Identity();

  static InertiaProperties fromMassAndInertia(double mass, const Eigen::Matrix3d& I) {
    InertiaProperties p;
    p.mass = mass;
    p.inertia = I;
    p.inertia_inv = I.inverse();
    return p;
  }
};

}  // namespace fdt
