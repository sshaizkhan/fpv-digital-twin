// The whole quad: rigid body + motors + battery + aerodynamics + ground
// contact + IMU, assembled from a QuadConfig.
//
// Phase 1 drives this directly from test code and from the headless runner.
// Phase 2 will drive the motor commands from Betaflight SITL instead; nothing
// in this class knows or cares where the commands come from.
//
// Determinism: given the same config, seed, initial state and command
// sequence, step() produces bit-identical results. Phase 4's log replay
// depends on that, so there is a test for it.
#pragma once

#include "fdt/aero.hpp"
#include "fdt/battery.hpp"
#include "fdt/ground.hpp"
#include "fdt/imu.hpp"
#include "fdt/motor.hpp"
#include "fdt/rigid_body.hpp"

namespace fdt {

/// Everything the sim knows that is not the rigid-body state itself.
struct Telemetry {
  double time = 0.0;                      ///< s since reset
  ImuSample imu;                          ///< what Betaflight would see
  std::array<double, 4> motor_rpm{};      ///< indexed by MotorId
  double battery_voltage = 0.0;           ///< V, under load
  double battery_current = 0.0;           ///< A
  double battery_soc = 1.0;               ///< fraction
  double total_thrust = 0.0;              ///< N, positive up
  bool in_contact = false;
};

class Multirotor {
 public:
  explicit Multirotor(const QuadConfig& config, uint64_t seed = 0);

  /// Per-motor throttle in [0, 1], indexed by MotorId. Clamped, not rejected.
  void setMotorCommands(const std::array<double, 4>& commands);

  /// Advance by one fixed step.
  void step(double dt);

  const State& state() const { return state_; }
  const Telemetry& telemetry() const { return telemetry_; }
  double time() const { return time_; }
  const InertiaProperties& inertia() const { return inertia_; }
  const QuadConfig& config() const { return config_; }
  const MotorBank& motors() const { return motors_; }
  const BatteryModel& battery() const { return battery_; }

  /// Back to t = 0: given state, full pack, motors stopped, noise stream
  /// rewound to the seed. Telemetry is refreshed too, except for the IMU
  /// sample -- drawing one would consume noise and defeat the rewind, so
  /// `telemetry().imu` stays zero until the first step().
  void reset(const State& initial = State{});

  /// Park level on the ground at the depth where the contact springs carry
  /// exactly the quad's weight, so it starts settled instead of dropping.
  void placeOnGround();

  /// Set all four motors to the throttle that balances weight at the current
  /// pack voltage, AND pre-spin the rotors to that speed, so the sim can start
  /// in a steady hover instead of spooling up from zero.
  void trimHover();

  /// Throttle per motor that balances weight at the current pack voltage.
  double hoverCommand() const;

  /// Body-frame wrench excluding gravity, at a given state and offset into the
  /// current step. This is what the integrator calls; it is public so tests
  /// can assert on forces directly rather than inferring them from motion.
  Wrench wrench(const State& state, double t_offset) const;

 private:
  /// Refresh every telemetry field except the IMU sample, and hand back the
  /// body wrench it computed so step() can reuse it.
  Wrench refreshTelemetry();

  QuadConfig config_;
  InertiaProperties inertia_;
  MotorBank motors_;
  BatteryModel battery_;
  AeroModel aero_;
  GroundModel ground_;
  ImuModel imu_;

  State state_;
  Telemetry telemetry_;
  double time_ = 0.0;
};

/// Mass and inertia from a config, ready for the integrator.
InertiaProperties inertiaFrom(const QuadConfig& config);

}  // namespace fdt
