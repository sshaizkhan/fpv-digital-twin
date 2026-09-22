#include "fdt/motor.hpp"

#include <algorithm>
#include <cmath>

namespace fdt {
namespace {
constexpr double kRpmToRadPerSec = 2.0 * 3.14159265358979323846 / 60.0;
}

MotorBank::MotorBank(const Motors& config) : config_(config) {
  kv_rad_ = config_.model.kv.value * kRpmToRadPerSec * config_.model.load_factor.value;
  omega_max_ = config_.model.max_rpm_safety.value * kRpmToRadPerSec;
  reset();
}

void MotorBank::reset() {
  commands_.fill(0.0);
  omega_.fill(0.0);
}

void MotorBank::setCommands(const std::array<double, 4>& commands) {
  for (size_t i = 0; i < 4; ++i) commands_[i] = std::clamp(commands[i], 0.0, 1.0);
}

void MotorBank::setSupplyVoltage(double volts) { supply_volts_ = std::max(0.0, volts); }

double MotorBank::targetSpeed(size_t motor) const {
  return std::min(commands_[motor] * kv_rad_ * supply_volts_, omega_max_);
}

MotorSample MotorBank::sampleAt(double t_offset) const {
  const double tau = config_.model.time_constant.value;
  const double decay = std::exp(-t_offset / tau);

  MotorSample s;
  for (size_t i = 0; i < 4; ++i) {
    const double target = targetSpeed(i);
    s.omega[i] = target + (omega_[i] - target) * decay;
    s.omega_dot[i] = (target - s.omega[i]) / tau;
  }
  return s;
}

void MotorBank::advance(double dt) { omega_ = sampleAt(dt).omega; }

std::array<double, 4> MotorBank::rpm() const {
  std::array<double, 4> out{};
  for (size_t i = 0; i < 4; ++i) out[i] = omega_[i] / kRpmToRadPerSec;
  return out;
}

Wrench MotorBank::wrench(const MotorSample& sample) const {
  const double kT = config_.model.thrust_coeff.value;
  const double kQ = config_.model.torque_coeff.value;
  const double J = config_.model.rotor_inertia.value;

  Wrench w;
  for (size_t i = 0; i < 4; ++i) {
    const MotorId id = static_cast<MotorId>(i);
    const double omega = sample.omega[i];
    const double thrust = kT * omega * omega;

    // Thrust acts along body -z (up); the arm from the CG turns it into roll
    // and pitch. docs/coordinate_frames.md §5.
    const Eigen::Vector3d force(0.0, 0.0, -thrust);
    w.force += force;
    w.torque += config_.at(id).position.value.cross(force);

    // Yaw reaction. `s` is +1 for a prop that looks clockwise from above; the
    // airframe is pushed the other way, hence the leading minus on both terms.
    const double s = spinSign(config_.at(id).spin);
    w.torque.z() -= s * kQ * omega * omega;   // aerodynamic drag reaction
    w.torque.z() -= s * J * sample.omega_dot[i];  // rotor spin-up reaction
  }
  return w;
}

double MotorBank::currentDraw(const MotorSample& sample) const {
  if (supply_volts_ <= 0.0) return 0.0;

  const double kQ = config_.model.torque_coeff.value;
  const double R = config_.model.resistance.value;
  const double V = supply_volts_;

  double total = 0.0;
  for (size_t i = 0; i < 4; ++i) {
    const double omega = sample.omega[i];
    const double shaft_power = kQ * omega * omega * omega;  // drag torque * speed
    if (shaft_power <= 0.0) continue;

    // Electrical power in = shaft power out + copper loss:
    //     V*I = kQ*omega^3 + I^2*R
    // Take the low-current root, which is the physical branch.
    if (R <= 0.0) {
      total += shaft_power / V;
      continue;
    }
    const double disc = V * V - 4.0 * R * shaft_power;
    total += (disc > 0.0) ? (V - std::sqrt(disc)) / (2.0 * R)
                          : V / (2.0 * R);  // stalled: the pack cannot supply this much shaft power
  }
  return total;
}

double MotorBank::commandForTotalThrust(double thrust_newtons, double supply_volts) const {
  if (thrust_newtons <= 0.0 || supply_volts <= 0.0) return 0.0;
  const double omega = std::sqrt(thrust_newtons / (4.0 * config_.model.thrust_coeff.value));
  return std::clamp(omega / (kv_rad_ * supply_volts), 0.0, 1.0);
}

}  // namespace fdt
