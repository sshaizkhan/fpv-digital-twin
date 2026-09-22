#include "fdt/rigid_body.hpp"

namespace fdt {
namespace {

/// Derivative of the 13-element state. The quaternion derivative is carried as
/// a plain 4-vector (w, x, y, z) because it is not itself a rotation.
struct Derivative {
  Eigen::Vector3d dp = Eigen::Vector3d::Zero();
  Eigen::Vector3d dv = Eigen::Vector3d::Zero();
  Eigen::Vector4d dq = Eigen::Vector4d::Zero();
  Eigen::Vector3d dw = Eigen::Vector3d::Zero();
};

Eigen::Vector4d asVector(const Eigen::Quaterniond& q) { return {q.w(), q.x(), q.y(), q.z()}; }

Eigen::Quaterniond asQuaternion(const Eigen::Vector4d& v) { return {v(0), v(1), v(2), v(3)}; }

/// dq/dt = 0.5 * q (x) [0, omega], with omega in the BODY frame, so the pure
/// quaternion multiplies q from the right.
Eigen::Vector4d quaternionDerivative(const Eigen::Quaterniond& q, const Eigen::Vector3d& omega) {
  const double w = q.w(), x = q.x(), y = q.y(), z = q.z();
  const double p = omega.x(), qr = omega.y(), r = omega.z();
  return 0.5 * Eigen::Vector4d(-x * p - y * qr - z * r,
                                w * p + y * r - z * qr,
                                w * qr + z * p - x * r,
                                w * r + x * qr - y * p);
}

Derivative evaluate(const State& s, double t_offset, const InertiaProperties& inertia, const WrenchFn& fn) {
  // The RK4 stages hold un-normalised quaternions. Models are handed a
  // normalised copy so that rotating a vector through it is meaningful, while
  // the quaternion ODE itself uses the raw stage value to keep RK4 exact.
  State normalised = s;
  normalised.orientation.normalize();

  const Wrench w = fn(normalised, t_offset);

  Derivative d;
  d.dp = s.velocity;
  d.dv = normalised.orientation * (w.force / inertia.mass) + gravityWorld();
  d.dq = quaternionDerivative(s.orientation, s.angular_velocity);
  const Eigen::Vector3d& omega = s.angular_velocity;
  d.dw = inertia.inertia_inv * (w.torque - omega.cross(inertia.inertia * omega));
  return d;
}

State advance(const State& s, const Derivative& d, double h) {
  State out;
  out.position = s.position + h * d.dp;
  out.velocity = s.velocity + h * d.dv;
  out.orientation = asQuaternion(asVector(s.orientation) + h * d.dq);
  out.angular_velocity = s.angular_velocity + h * d.dw;
  return out;
}

}  // namespace

State integrateRK4(const State& state, double dt, const InertiaProperties& inertia, const WrenchFn& wrench) {
  const Derivative k1 = evaluate(state, 0.0, inertia, wrench);
  const Derivative k2 = evaluate(advance(state, k1, dt * 0.5), dt * 0.5, inertia, wrench);
  const Derivative k3 = evaluate(advance(state, k2, dt * 0.5), dt * 0.5, inertia, wrench);
  const Derivative k4 = evaluate(advance(state, k3, dt), dt, inertia, wrench);

  Derivative sum;
  sum.dp = (k1.dp + 2.0 * k2.dp + 2.0 * k3.dp + k4.dp) / 6.0;
  sum.dv = (k1.dv + 2.0 * k2.dv + 2.0 * k3.dv + k4.dv) / 6.0;
  sum.dq = (k1.dq + 2.0 * k2.dq + 2.0 * k3.dq + k4.dq) / 6.0;
  sum.dw = (k1.dw + 2.0 * k2.dw + 2.0 * k3.dw + k4.dw) / 6.0;

  State out = advance(state, sum, dt);
  out.orientation.normalize();
  return out;
}

State integrateSemiImplicitEuler(const State& state, double dt, const InertiaProperties& inertia,
                                 const WrenchFn& wrench) {
  const Derivative k = evaluate(state, 0.0, inertia, wrench);

  State out = state;
  // Velocities first, then positions from the UPDATED velocities: that is what
  // makes it symplectic and well behaved against a stiff contact spring.
  out.velocity = state.velocity + dt * k.dv;
  out.angular_velocity = state.angular_velocity + dt * k.dw;
  out.position = state.position + dt * out.velocity;
  out.orientation = asQuaternion(asVector(state.orientation) +
                                 dt * quaternionDerivative(state.orientation, out.angular_velocity));
  out.orientation.normalize();
  return out;
}

double mechanicalEnergy(const State& state, const InertiaProperties& inertia) {
  const double translational = 0.5 * inertia.mass * state.velocity.squaredNorm();
  const double rotational = 0.5 * state.angular_velocity.dot(inertia.inertia * state.angular_velocity);
  // Potential is measured against the world origin plane; NED z is down, so
  // altitude is -z and a body below the origin has negative potential energy.
  const double potential = inertia.mass * kGravity * state.altitude();
  return translational + rotational + potential;
}

Eigen::Vector3d angularMomentumWorld(const State& state, const InertiaProperties& inertia) {
  return state.orientation * (inertia.inertia * state.angular_velocity);
}

}  // namespace fdt
