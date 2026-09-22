// The four motors: command -> first-order lag -> rotor speed -> thrust and
// torque, plus the rotor-inertia reaction that a quad feels in yaw whenever
// the props are changing speed.
//
// Signs and the spin-direction convention: docs/coordinate_frames.md §5.
//
// The lag is solved analytically rather than integrated. Over one step the
// commanded speed and the supply voltage are constant, so
//
//     omega(t)     = w_target + (omega_0 - w_target) * exp(-t / tau)
//     omega_dot(t) = (w_target - omega(t)) / tau
//
// is exact, and can be evaluated at any sub-step time. That is what lets the
// RK4 stages see the true motor speed at t, t+dt/2 and t+dt instead of a value
// frozen at the start of the step.
#pragma once

#include "fdt/quad_config.hpp"
#include "fdt/types.hpp"

#include <array>

namespace fdt {

/// Rotor speeds and their derivatives at one instant.
struct MotorSample {
  std::array<double, 4> omega{};      ///< rad/s, indexed by MotorId
  std::array<double, 4> omega_dot{};  ///< rad/s^2
};

class MotorBank {
 public:
  explicit MotorBank(const Motors& config);

  /// Throttle per motor in [0, 1], indexed by MotorId. Values outside the
  /// range are clamped, not rejected: Betaflight will hand us saturated mixer
  /// output as a matter of course.
  void setCommands(const std::array<double, 4>& commands);

  /// Pack voltage available this step. Scales the steady-state rotor speed,
  /// which is how a sagging battery costs you authority.
  void setSupplyVoltage(double volts);

  /// Exact rotor state `t_offset` seconds into the current step.
  MotorSample sampleAt(double t_offset) const;

  /// Commit the analytic solution: move the stored speeds forward by dt.
  void advance(double dt);

  /// Body-frame force and torque produced by a sample.
  Wrench wrench(const MotorSample& sample) const;

  /// Total pack current for a sample, from the power balance
  /// `V*I = kQ*omega^3 + I^2*R` solved per motor.
  double currentDraw(const MotorSample& sample) const;

  const std::array<double, 4>& speeds() const { return omega_; }
  std::array<double, 4> rpm() const;

  /// Steady-state rotor speed the current command and supply voltage imply.
  double targetSpeed(size_t motor) const;

  /// Throttle at which four motors produce `thrust_newtons` in total, at the
  /// given supply voltage. Used for hover trim in tests and in the runner.
  double commandForTotalThrust(double thrust_newtons, double supply_volts) const;

  void reset();

 private:
  Motors config_;
  std::array<double, 4> commands_{};
  std::array<double, 4> omega_{};
  double supply_volts_ = 0.0;
  double kv_rad_ = 0.0;     ///< kv converted from rpm/V to (rad/s)/V
  double omega_max_ = 0.0;  ///< safety clamp, rad/s
};

}  // namespace fdt
