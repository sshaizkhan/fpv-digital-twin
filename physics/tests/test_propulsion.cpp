// Motor and battery model tests.

#include "fdt/battery.hpp"
#include "fdt/motor.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

fdt::QuadConfig config() { return fdt::loadQuadConfig(std::string(FDT_REPO_ROOT) + "/config/quad.yaml"); }

size_t idx(fdt::MotorId id) { return static_cast<size_t>(id); }

std::array<double, 4> commands(double fl, double fr, double rl, double rr) {
  std::array<double, 4> c{};
  c[idx(fdt::MotorId::FL)] = fl;
  c[idx(fdt::MotorId::FR)] = fr;
  c[idx(fdt::MotorId::RL)] = rl;
  c[idx(fdt::MotorId::RR)] = rr;
  return c;
}

constexpr double kFullPack4S = 4.0 * 4.20;

}  // namespace

// --- battery --------------------------------------------------------------

TEST(Battery, FullPackReadsOpenCircuitVoltage) {
  fdt::BatteryModel b(config().battery);
  EXPECT_DOUBLE_EQ(b.stateOfCharge(), 1.0);
  EXPECT_DOUBLE_EQ(b.openCircuitVoltage(), kFullPack4S);
  EXPECT_DOUBLE_EQ(b.terminalVoltage(), kFullPack4S) << "no load means no sag";
}

TEST(Battery, SagsUnderLoadByCurrentTimesInternalResistance) {
  const auto cfg = config();
  fdt::BatteryModel b(cfg.battery);
  const double amps = 40.0;
  b.step(1e-4, amps);
  const double expected_sag = amps * cfg.battery.internal_resistance.value;
  EXPECT_NEAR(b.terminalVoltage(), b.openCircuitVoltage() - expected_sag, 1e-9);
  EXPECT_LT(b.terminalVoltage(), kFullPack4S) << "a loaded pack must sag";
}

TEST(Battery, StateOfChargeFallsAtTheRightRate) {
  const auto cfg = config();
  fdt::BatteryModel b(cfg.battery);
  const double amps = 20.0;
  const double dt = 1e-3;
  for (int i = 0; i < 60000; ++i) b.step(dt, amps);  // 60 s at 20 A

  // 20 A for 60 s is 0.3333 Ah out of a 1.3 Ah pack.
  const double expected = 1.0 - (amps * 60.0 / 3600.0) / cfg.battery.capacity.value;
  EXPECT_NEAR(b.stateOfCharge(), expected, 1e-9);
  EXPECT_LT(b.openCircuitVoltage(), kFullPack4S) << "OCV must fall as the pack empties";
}

TEST(Battery, StateOfChargeClampsAtEmpty) {
  fdt::BatteryModel b(config().battery);
  for (int i = 0; i < 1000; ++i) b.step(1.0, 100.0);
  EXPECT_DOUBLE_EQ(b.stateOfCharge(), 0.0);
  EXPECT_GT(b.openCircuitVoltage(), 0.0);
}

TEST(Battery, ResetRestoresTheInitialState) {
  fdt::BatteryModel b(config().battery);
  b.step(10.0, 30.0);
  ASSERT_LT(b.stateOfCharge(), 1.0);
  b.reset();
  EXPECT_DOUBLE_EQ(b.stateOfCharge(), 1.0);
  EXPECT_DOUBLE_EQ(b.current(), 0.0);
  EXPECT_DOUBLE_EQ(b.terminalVoltage(), kFullPack4S);
}

// --- motor lag ------------------------------------------------------------

TEST(Motor, StepResponseReaches63PercentAtOneTimeConstant) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  m.setCommands(commands(1.0, 1.0, 1.0, 1.0));

  const double tau = cfg.motors.model.time_constant.value;
  const double target = m.targetSpeed(0);
  ASSERT_GT(target, 0.0);

  const auto at_tau = m.sampleAt(tau);
  EXPECT_NEAR(at_tau.omega[0] / target, 1.0 - std::exp(-1.0), 1e-12);
  EXPECT_NEAR(at_tau.omega[0] / target, 0.6321205588, 1e-9);
}

TEST(Motor, SettlesOnTheTargetSpeedAndStopsAccelerating) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  m.setCommands(commands(0.5, 0.5, 0.5, 0.5));

  for (int i = 0; i < 2000; ++i) m.advance(1e-3);  // 2 s, ~80 time constants

  const double target = m.targetSpeed(0);
  EXPECT_NEAR(m.speeds()[0], target, 1e-9);
  EXPECT_NEAR(m.sampleAt(0.0).omega_dot[0], 0.0, 1e-6);
}

TEST(Motor, AnalyticSampleMatchesAFineNumericalIntegration) {
  const auto cfg = config();
  fdt::MotorBank analytic(cfg.motors);
  analytic.setSupplyVoltage(kFullPack4S);
  analytic.setCommands(commands(0.8, 0.8, 0.8, 0.8));

  // Forward-Euler the same ODE at a very fine step and compare.
  const double tau = cfg.motors.model.time_constant.value;
  const double target = analytic.targetSpeed(0);
  double omega = 0.0;
  const double h = 1e-9;
  const int steps = 10000;  // 10 us
  for (int i = 0; i < steps; ++i) omega += h * (target - omega) / tau;

  EXPECT_NEAR(analytic.sampleAt(h * steps).omega[0], omega, 1e-6);
}

TEST(Motor, OmegaDotMatchesTheFiniteDifferenceOfOmega) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  m.setCommands(commands(0.7, 0.3, 0.9, 0.1));

  const double t = 0.004;
  const double h = 1e-7;
  for (size_t i = 0; i < 4; ++i) {
    const double fd = (m.sampleAt(t + h).omega[i] - m.sampleAt(t - h).omega[i]) / (2.0 * h);
    EXPECT_NEAR(m.sampleAt(t).omega_dot[i], fd, 1e-3) << "motor " << i;
  }
}

TEST(Motor, AdvanceIsConsistentWithSampleAt) {
  const auto cfg = config();
  fdt::MotorBank a(cfg.motors), b(cfg.motors);
  for (auto* m : {&a, &b}) {
    m->setSupplyVoltage(kFullPack4S);
    m->setCommands(commands(0.6, 0.6, 0.6, 0.6));
  }
  const double dt = 1.0 / 2000.0;
  const double sampled = a.sampleAt(dt).omega[0];
  b.advance(dt);
  EXPECT_NEAR(b.speeds()[0], sampled, 1e-15);
}

TEST(Motor, CommandsAreClampedToTheValidRange) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  m.setCommands(commands(2.0, -1.0, 0.5, 0.5));
  EXPECT_DOUBLE_EQ(m.targetSpeed(idx(fdt::MotorId::FL)), m.targetSpeed(idx(fdt::MotorId::FL)));
  EXPECT_GT(m.targetSpeed(idx(fdt::MotorId::FL)), 0.0);
  EXPECT_DOUBLE_EQ(m.targetSpeed(idx(fdt::MotorId::FR)), 0.0) << "negative command must clamp to zero";
  // Full command at this voltage must not exceed the safety clamp.
  const double max_rad = cfg.motors.model.max_rpm_safety.value * 2.0 * M_PI / 60.0;
  EXPECT_LE(m.targetSpeed(idx(fdt::MotorId::FL)), max_rad + 1e-9);
}

TEST(Motor, LowerSupplyVoltageLowersTopSpeed) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setCommands(commands(1.0, 1.0, 1.0, 1.0));

  m.setSupplyVoltage(kFullPack4S);
  const double fast = m.targetSpeed(0);
  m.setSupplyVoltage(kFullPack4S * 0.75);
  const double slow = m.targetSpeed(0);

  EXPECT_NEAR(slow / fast, 0.75, 1e-12) << "rotor speed is proportional to applied volts";
}

// --- forces and torques ---------------------------------------------------

TEST(Motor, ThrustIsKtOmegaSquaredAndPointsUp) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  m.setCommands(commands(1.0, 1.0, 1.0, 1.0));
  for (int i = 0; i < 2000; ++i) m.advance(1e-3);

  const auto sample = m.sampleAt(0.0);
  const fdt::Wrench w = m.wrench(sample);
  const double expected = 4.0 * cfg.motors.model.thrust_coeff.value * sample.omega[0] * sample.omega[0];

  EXPECT_NEAR(w.force.z(), -expected, 1e-9) << "thrust is along body -z (up)";
  EXPECT_LT(w.force.z(), 0.0);
  EXPECT_NEAR(w.force.x(), 0.0, 1e-15);
  EXPECT_NEAR(w.force.y(), 0.0, 1e-15);

  // Sanity on the placeholder coefficients: a 5in quad should be able to lift
  // several times its own weight.
  const double weight = cfg.mass.auw.value * fdt::kGravity;
  EXPECT_GT(expected / weight, 3.0) << "thrust-to-weight of " << expected / weight << " is implausibly low";
  EXPECT_LT(expected / weight, 15.0) << "thrust-to-weight of " << expected / weight << " is implausibly high";
}

TEST(Motor, SymmetricThrustProducesNoTorque) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  m.setCommands(commands(0.5, 0.5, 0.5, 0.5));
  for (int i = 0; i < 2000; ++i) m.advance(1e-3);  // settled, so no spin-up torque

  const fdt::Wrench w = m.wrench(m.sampleAt(0.0));
  EXPECT_NEAR(w.torque.x(), 0.0, 1e-12) << "four equal motors must not roll";
  EXPECT_NEAR(w.torque.y(), 0.0, 1e-12) << "four equal motors must not pitch";
  EXPECT_NEAR(w.torque.z(), 0.0, 1e-12) << "two CW and two CCW at equal speed must not yaw";
}

TEST(Motor, SingleMotorTorqueSignsFollowTheGeometry) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  m.setCommands(commands(0.0, 0.6, 0.0, 0.0));  // front-right only
  for (int i = 0; i < 2000; ++i) m.advance(1e-3);

  const fdt::Wrench w = m.wrench(m.sampleAt(0.0));
  // Lifting the front-right corner rolls the craft LEFT and pitches the nose UP.
  EXPECT_LT(w.torque.x(), 0.0) << "front-right thrust must roll left (negative p)";
  EXPECT_GT(w.torque.y(), 0.0) << "front-right thrust must pitch nose up (positive q)";
}

TEST(Motor, RollPairProducesPureRollTorque) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  // Spin up the LEFT pair: the craft must roll RIGHT (positive p).
  m.setCommands(commands(0.7, 0.3, 0.7, 0.3));
  for (int i = 0; i < 2000; ++i) m.advance(1e-3);

  const fdt::Wrench w = m.wrench(m.sampleAt(0.0));
  EXPECT_GT(w.torque.x(), 0.0) << "more thrust on the left must roll right";
  EXPECT_NEAR(w.torque.y(), 0.0, 1e-12) << "a left/right split must not pitch";
  EXPECT_NEAR(w.torque.z(), 0.0, 1e-12) << "the split keeps one CW and one CCW on each side";
}

TEST(Motor, PitchPairProducesPurePitchTorque) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  // Spin up the REAR pair. Pushing up at the tail drops the NOSE, so this is
  // negative q -- it is how you pitch forward, not how you flare.
  m.setCommands(commands(0.3, 0.3, 0.7, 0.7));
  for (int i = 0; i < 2000; ++i) m.advance(1e-3);

  const fdt::Wrench rear_up = m.wrench(m.sampleAt(0.0));
  EXPECT_LT(rear_up.torque.y(), 0.0) << "more thrust at the rear must pitch the nose down";
  EXPECT_NEAR(rear_up.torque.x(), 0.0, 1e-12);
  EXPECT_NEAR(rear_up.torque.z(), 0.0, 1e-12);

  // And the front pair must do the opposite, symmetrically.
  fdt::MotorBank f(cfg.motors);
  f.setSupplyVoltage(kFullPack4S);
  f.setCommands(commands(0.7, 0.7, 0.3, 0.3));
  for (int i = 0; i < 2000; ++i) f.advance(1e-3);

  const fdt::Wrench front_up = f.wrench(f.sampleAt(0.0));
  EXPECT_GT(front_up.torque.y(), 0.0) << "more thrust at the front must pitch the nose up";
  EXPECT_NEAR(front_up.torque.y(), -rear_up.torque.y(), 1e-12) << "the X frame is symmetric";
}

TEST(Motor, YawTorqueOpposesTheSpinOfTheFasterDiagonal) {
  const auto cfg = config();
  ASSERT_EQ(cfg.motors.at(fdt::MotorId::FL).spin, fdt::Spin::CW);
  ASSERT_EQ(cfg.motors.at(fdt::MotorId::RR).spin, fdt::Spin::CW);

  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  m.setCommands(commands(0.7, 0.3, 0.3, 0.7));  // the CW diagonal faster
  for (int i = 0; i < 2000; ++i) m.advance(1e-3);

  const fdt::Wrench w = m.wrench(m.sampleAt(0.0));
  // Airframe reaction opposes the props: faster CW props yaw the body LEFT.
  EXPECT_LT(w.torque.z(), 0.0) << "faster CW-from-above props must yaw the airframe left";
  EXPECT_NEAR(w.torque.x(), 0.0, 1e-12) << "this split is balanced left/right";
  EXPECT_NEAR(w.torque.y(), 0.0, 1e-12) << "this split is balanced front/rear";
}

TEST(Motor, RotorInertiaAddsYawTorqueOnlyWhileSpinningUp) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  m.setCommands(commands(1.0, 0.0, 0.0, 1.0));  // slam the CW diagonal from rest

  const fdt::Wrench spinning_up = m.wrench(m.sampleAt(0.0));
  // At t=0 the props are still stopped, so aero drag torque is zero and the
  // only yaw torque is the rotor-inertia reaction.
  EXPECT_LT(spinning_up.torque.z(), 0.0) << "accelerating CW props must kick the airframe left";

  const double J = cfg.motors.model.rotor_inertia.value;
  const auto s0 = m.sampleAt(0.0);
  EXPECT_NEAR(spinning_up.torque.z(), -J * (s0.omega_dot[static_cast<size_t>(fdt::MotorId::FL)] +
                                            s0.omega_dot[static_cast<size_t>(fdt::MotorId::RR)]),
              1e-12);

  for (int i = 0; i < 2000; ++i) m.advance(1e-3);
  const auto settled = m.sampleAt(0.0);
  EXPECT_NEAR(settled.omega_dot[0], 0.0, 1e-6) << "settled props contribute no inertia reaction";
}

// --- current --------------------------------------------------------------

TEST(Motor, CurrentDrawSolvesThePowerBalance) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  m.setCommands(commands(0.8, 0.8, 0.8, 0.8));
  for (int i = 0; i < 2000; ++i) m.advance(1e-3);

  const auto sample = m.sampleAt(0.0);
  const double total = m.currentDraw(sample);
  EXPECT_GT(total, 0.0);

  // Per motor: V*I = kQ*omega^3 + I^2*R
  const double per_motor = total / 4.0;
  const double kQ = cfg.motors.model.torque_coeff.value;
  const double R = cfg.motors.model.resistance.value;
  const double w = sample.omega[0];
  EXPECT_NEAR(kFullPack4S * per_motor, kQ * w * w * w + per_motor * per_motor * R, 1e-6);

  // And the magnitude should be believable for a 5in quad at 80% throttle.
  EXPECT_GT(total, 10.0);
  EXPECT_LT(total, 120.0);
}

TEST(Motor, StoppedMotorsDrawNoCurrent) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);
  m.setCommands(commands(0.0, 0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(m.currentDraw(m.sampleAt(0.0)), 0.0);
}

TEST(Motor, HoverCommandProducesExactlyHoverThrust) {
  const auto cfg = config();
  fdt::MotorBank m(cfg.motors);
  m.setSupplyVoltage(kFullPack4S);

  const double weight = cfg.mass.auw.value * fdt::kGravity;
  const double c = m.commandForTotalThrust(weight, kFullPack4S);
  EXPECT_GT(c, 0.0);
  EXPECT_LT(c, 1.0) << "the quad must be able to hover below full throttle";

  m.setCommands(commands(c, c, c, c));
  for (int i = 0; i < 4000; ++i) m.advance(1e-3);
  const fdt::Wrench w = m.wrench(m.sampleAt(0.0));
  EXPECT_NEAR(-w.force.z(), weight, 1e-6);
}
