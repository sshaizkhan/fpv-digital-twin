#include "fdt/multirotor.hpp"

namespace fdt {

InertiaProperties inertiaFrom(const QuadConfig& config) {
  return InertiaProperties::fromMassAndInertia(config.mass.auw.value, config.mass.inertiaMatrix());
}

Multirotor::Multirotor(const QuadConfig& config, uint64_t seed)
    : config_(config),
      inertia_(inertiaFrom(config)),
      motors_(config.motors),
      battery_(config.battery),
      aero_(config.aero),
      ground_(config.ground, config.motors),
      imu_(config.imu, seed) {
  reset();
}

void Multirotor::reset(const State& initial) {
  state_ = initial;
  time_ = 0.0;
  motors_.reset();
  battery_.reset();
  imu_.reset();
  telemetry_ = Telemetry{};
  motors_.setSupplyVoltage(battery_.terminalVoltage());
  refreshTelemetry();
}

void Multirotor::placeOnGround() {
  State s;
  const double weight = inertia_.mass * kGravity;
  // Depth at which the four contact springs carry exactly the weight.
  const double depth = weight / (4.0 * config_.ground.contact_stiffness.value);
  s.position.z() = config_.ground.height.value - config_.ground.stand_height.value + depth;
  reset(s);
}

double Multirotor::hoverCommand() const {
  return motors_.commandForTotalThrust(inertia_.mass * kGravity, battery_.terminalVoltage());
}

void Multirotor::trimHover() {
  motors_.setSupplyVoltage(battery_.terminalVoltage());
  const double c = hoverCommand();
  motors_.setCommands({c, c, c, c});
  // Fifty time constants: the residual is ~1e-22 of the step, i.e. exact.
  motors_.advance(50.0 * config_.motors.model.time_constant.value);
}

void Multirotor::setMotorCommands(const std::array<double, 4>& commands) { motors_.setCommands(commands); }

Wrench Multirotor::wrench(const State& state, double t_offset) const {
  const MotorSample sample = motors_.sampleAt(t_offset);

  Wrench w = motors_.wrench(sample);
  const double total_thrust = -w.force.z();  // thrust is along body -z, so negate for a positive magnitude

  w += aero_.wrench(state.velocityBody(), state.angular_velocity, total_thrust);
  w += ground_.wrench(state);
  return w;
}

void Multirotor::step(double dt) {
  // The pack voltage is held across the step; the motors see whatever the
  // battery could deliver at the end of the previous one.
  motors_.setSupplyVoltage(battery_.terminalVoltage());

  const WrenchFn fn = [this](const State& s, double t_offset) { return wrench(s, t_offset); };
  state_ = integrateRK4(state_, dt, inertia_, fn);

  // Commit the analytic motor solution, then charge the pack for the step.
  motors_.advance(dt);
  const double current = motors_.currentDraw(motors_.sampleAt(0.0));
  battery_.step(dt, current);

  time_ += dt;

  const Wrench w = refreshTelemetry();

  // The accelerometer measures specific force: every force EXCEPT gravity,
  // per unit mass, in the body frame. Our wrench already excludes gravity
  // (the integrator adds it in the world frame), so this is just F/m --
  // identical to specificForceBody(state_, a_world) but without the round
  // trip through the world frame in the hot loop.
  telemetry_.imu = imu_.sample(state_.angular_velocity, w.force / inertia_.mass, dt);
}

Wrench Multirotor::refreshTelemetry() {
  const Wrench w = wrench(state_, 0.0);

  telemetry_.time = time_;
  telemetry_.motor_rpm = motors_.rpm();
  telemetry_.battery_voltage = battery_.terminalVoltage();
  telemetry_.battery_current = battery_.current();
  telemetry_.battery_soc = battery_.stateOfCharge();
  telemetry_.total_thrust = -motors_.wrench(motors_.sampleAt(0.0)).force.z();
  telemetry_.in_contact = ground_.inContact(state_);
  return w;
}

}  // namespace fdt
