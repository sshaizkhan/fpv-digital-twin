// Whole-vehicle tests: the Phase 1 acceptance criteria end to end, the motor
// ordering and sign checks the project's working rules require, and the
// determinism and speed properties Phase 4's replay depends on.

#include "fdt/multirotor.hpp"

#include <gtest/gtest.h>

#include <chrono>
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

fdt::State airborne(double altitude_m) {
  fdt::State s;
  s.position.z() = -altitude_m;  // NED: up is -z
  return s;
}

constexpr double kDt = 1.0 / 2000.0;

/// One IMU sample carries sigma = accel_noise_density / sqrt(dt), which at
/// 2 kHz is ~0.09 m/s^2 -- far too coarse to assert a 1 g reading against.
/// Averaging N samples cuts that by sqrt(N).
Eigen::Vector3d meanAccelOver(fdt::Multirotor& m, int samples) {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (int i = 0; i < samples; ++i) {
    m.step(kDt);
    sum += m.telemetry().imu.accel;
  }
  return sum / static_cast<double>(samples);
}

void run(fdt::Multirotor& m, double seconds) {
  const int steps = static_cast<int>(std::lround(seconds / kDt));
  for (int i = 0; i < steps; ++i) m.step(kDt);
}

}  // namespace

// --- acceptance criterion: free fall = g ----------------------------------

TEST(Multirotor, MotorsOffFreeFallsAtExactlyG) {
  fdt::Multirotor m(config());
  m.reset(airborne(100.0));
  m.setMotorCommands(commands(0, 0, 0, 0));

  // With the motors off and the airspeed zero, the total non-gravitational
  // force is exactly zero, so the acceleration is exactly g. This is the
  // acceptance criterion stated without the integrator in the way.
  const fdt::Wrench w = m.wrench(m.state(), 0.0);
  EXPECT_EQ(w.force.norm(), 0.0);
  EXPECT_EQ(w.torque.norm(), 0.0);

  // Over a real step the RK4 sub-stages already have a little downward speed,
  // so drag shaves a part per ~30000 off. Anything larger is a bug.
  m.step(kDt);
  EXPECT_NEAR(m.state().velocity.z() / (fdt::kGravity * kDt), 1.0, 1e-4);
  EXPECT_LT(m.state().velocity.z(), fdt::kGravity * kDt) << "drag can only ever slow the fall";
  EXPECT_NEAR(m.state().velocity.x(), 0.0, 1e-18);

  run(m, 1.0);
  // Drag now bites, so the speed is a little under free fall but not far.
  const double free_fall = fdt::kGravity * (1.0 + kDt);
  EXPECT_LT(m.state().velocity.z(), free_fall);
  EXPECT_GT(m.state().velocity.z(), 0.90 * free_fall);
  EXPECT_LT(m.state().altitude(), 100.0) << "it must actually have fallen";
}

TEST(Multirotor, FallingAccelerometerReadsNearZeroNotOneG) {
  fdt::Multirotor m(config());
  m.reset(airborne(100.0));
  m.setMotorCommands(commands(0, 0, 0, 0));

  // Averaged over the first 0.1 s of the drop. The reading is not exactly
  // zero because by then the quad is doing ~1 m/s and drag is a real force --
  // but it is a couple of percent of g, not g.
  const Eigen::Vector3d a = meanAccelOver(m, 200);
  EXPECT_LT(a.norm(), 0.15) << "free fall is weightless; got " << a.norm() << " m/s^2";
  EXPECT_LT(a.norm() / fdt::kGravity, 0.02);
  EXPECT_LT(a.z(), 0.0)
      << "the only specific force is drag; it acts upward on a falling body, and up is -z";
}

// --- acceptance criterion: hover thrust balances weight -------------------

TEST(Multirotor, TrimmedHoverThrustExactlyBalancesWeight) {
  fdt::Multirotor m(config());
  m.reset(airborne(10.0));
  m.trimHover();

  const double weight = m.inertia().mass * fdt::kGravity;
  const fdt::Wrench w = m.wrench(m.state(), 0.0);

  EXPECT_NEAR(-w.force.z(), weight, 1e-9) << "hover thrust must equal weight";
  EXPECT_NEAR(w.force.x(), 0.0, 1e-12);
  EXPECT_NEAR(w.force.y(), 0.0, 1e-12);
  EXPECT_NEAR(w.torque.norm(), 0.0, 1e-12) << "a trimmed hover must be torque free";

  EXPECT_GT(m.hoverCommand(), 0.15);
  EXPECT_LT(m.hoverCommand(), 0.60) << "hovering at " << m.hoverCommand() * 100
                                    << "% throttle is not a believable 5in quad";
}

TEST(Multirotor, TrimmedHoverIsAlmostStationaryOverAShortInterval) {
  fdt::Multirotor m(config());
  m.reset(airborne(10.0));
  m.trimHover();

  run(m, 1.0);
  // Open loop, so the slow pack sag makes it sink a little. What matters is
  // that it is centimetres, not metres.
  EXPECT_LT(std::abs(m.state().altitude() - 10.0), 0.05);
  EXPECT_LT(m.state().position.head<2>().norm(), 1e-6) << "no lateral drift in a trimmed hover";
}

TEST(Multirotor, OpenLoopHoverSinksAsThePackSags) {
  // Documents real behaviour rather than hiding it: with a fixed throttle the
  // quad slowly descends as the battery droops. Phase 2 hands this problem to
  // Betaflight, which is the point of the project.
  fdt::Multirotor m(config());
  m.reset(airborne(50.0));
  m.trimHover();
  const double v0 = m.telemetry().battery_voltage;
  ASSERT_NEAR(v0, 4.0 * 4.20, 1e-9) << "telemetry must be valid before the first step";

  run(m, 10.0);

  EXPECT_LT(m.telemetry().battery_voltage, v0) << "the pack must sag under hover load";
  EXPECT_LT(m.state().altitude(), 50.0) << "and the quad must therefore sink";
  EXPECT_GT(m.state().altitude(), 40.0) << "but not fall out of the sky in 10 s";
}

TEST(Multirotor, AProportionalAltitudeHoldKeepsItWithinFiveCentimetres) {
  // Proves the assembled model is not just balanced but controllable: a
  // trivial PD loop on throttle should hold altitude comfortably.
  fdt::Multirotor m(config());
  m.reset(airborne(10.0));
  m.trimHover();

  const double target = 10.0;
  double worst = 0.0;
  for (int i = 0; i < 20000; ++i) {  // 10 s
    const double error = target - m.state().altitude();
    const double rate = -m.state().velocity.z();  // climb rate, positive up
    const double c = std::clamp(m.hoverCommand() + 0.05 * error - 0.03 * rate, 0.0, 1.0);
    m.setMotorCommands(commands(c, c, c, c));
    m.step(kDt);
    worst = std::max(worst, std::abs(m.state().altitude() - target));
  }
  EXPECT_LT(worst, 0.05) << "worst altitude error was " << worst << " m";
}

// --- acceptance criterion: pure roll torque -------------------------------

TEST(Multirotor, RollSplitGivesTheExpectedAngularAcceleration) {
  fdt::Multirotor m(config());
  m.reset(airborne(50.0));
  m.trimHover();

  // Add to the left pair, take the same from the right: pure roll.
  const double c = m.hoverCommand();
  const double d = 0.08;
  m.setMotorCommands(commands(c + d, c - d, c + d, c - d));

  // Let the rotors settle so the measured torque is the steady-state one.
  run(m, 0.30);

  const fdt::Wrench w = m.wrench(m.state(), 0.0);
  const double expected_alpha = w.torque.x() / m.inertia().inertia(0, 0);
  EXPECT_GT(expected_alpha, 0.0) << "extra thrust on the left must roll right";

  // Measure the actual angular acceleration across one step.
  const double before = m.state().angular_velocity.x();
  m.step(kDt);
  const double measured_alpha = (m.state().angular_velocity.x() - before) / kDt;

  EXPECT_NEAR(measured_alpha / expected_alpha, 1.0, 1e-3);

  // A 5in quad should be able to produce hundreds of rad/s^2, not tens.
  EXPECT_GT(std::abs(expected_alpha), 50.0)
      << "roll authority of " << expected_alpha << " rad/s^2 looks far too low";
}

// --- acceptance criterion: energy sane ------------------------------------

TEST(Multirotor, EnergyNeverIncreasesWithTheMotorsOff) {
  fdt::Multirotor m(config());
  fdt::State s = airborne(200.0);
  s.velocity = Eigen::Vector3d(12.0, -5.0, -20.0);
  s.angular_velocity = Eigen::Vector3d(8.0, -3.0, 5.0);
  m.reset(s);
  m.setMotorCommands(commands(0, 0, 0, 0));

  double previous = fdt::mechanicalEnergy(m.state(), m.inertia());
  const double initial = previous;
  for (int i = 0; i < 10000; ++i) {  // 5 s
    m.step(kDt);
    const double e = fdt::mechanicalEnergy(m.state(), m.inertia());
    ASSERT_LE(e, previous + 1e-9) << "drag put energy IN at step " << i;
    previous = e;
  }
  EXPECT_LT(previous, initial) << "drag must actually remove energy";
}

// --- motor ordering and signs through the full stack ----------------------

TEST(Multirotor, EachMotorAloneProducesTheDocumentedRotation) {
  struct Case {
    fdt::MotorId id;
    double expected_roll_sign;   // sign of p
    double expected_pitch_sign;  // sign of q
    const char* description;
  };
  // Thrust at a corner lifts that corner: the craft rolls and pitches AWAY
  // from it. docs/coordinate_frames.md §5.
  const Case cases[] = {
      {fdt::MotorId::FL, +1.0, +1.0, "front-left lifts: rolls right, nose up"},
      {fdt::MotorId::FR, -1.0, +1.0, "front-right lifts: rolls left, nose up"},
      {fdt::MotorId::RL, +1.0, -1.0, "rear-left lifts: rolls right, nose down"},
      {fdt::MotorId::RR, -1.0, -1.0, "rear-right lifts: rolls left, nose down"},
  };

  for (const auto& c : cases) {
    fdt::Multirotor m(config());
    m.reset(airborne(50.0));
    std::array<double, 4> cmd{};
    cmd[idx(c.id)] = 0.6;
    m.setMotorCommands(cmd);
    run(m, 0.10);

    const Eigen::Vector3d omega = m.state().angular_velocity;
    EXPECT_GT(omega.x() * c.expected_roll_sign, 0.0) << c.description << " (roll was " << omega.x() << ")";
    EXPECT_GT(omega.y() * c.expected_pitch_sign, 0.0) << c.description << " (pitch was " << omega.y() << ")";
  }
}

TEST(Multirotor, YawFollowsTheSpinDirectionOfTheFasterDiagonal) {
  const auto cfg = config();
  ASSERT_EQ(cfg.motors.at(fdt::MotorId::FL).spin, fdt::Spin::CW);

  fdt::Multirotor m(cfg);
  m.reset(airborne(50.0));
  m.trimHover();
  const double c = m.hoverCommand();
  const double d = 0.10;
  // Speed up the CW diagonal (FL, RR), slow the CCW one.
  m.setMotorCommands(commands(c + d, c - d, c - d, c + d));
  run(m, 0.30);

  EXPECT_LT(m.state().angular_velocity.z(), 0.0)
      << "faster CW-from-above props must yaw the airframe LEFT";
  EXPECT_NEAR(m.state().angular_velocity.x(), 0.0, 1e-6) << "a yaw command must not roll";
  EXPECT_NEAR(m.state().angular_velocity.y(), 0.0, 1e-6) << "a yaw command must not pitch";
}

// --- ground ---------------------------------------------------------------

TEST(Multirotor, ParkedQuadSitsStillAndLevel) {
  fdt::Multirotor m(config());
  m.placeOnGround();
  m.setMotorCommands(commands(0, 0, 0, 0));
  const double z0 = m.state().position.z();

  run(m, 5.0);

  EXPECT_TRUE(m.telemetry().in_contact);
  EXPECT_NEAR(m.state().position.z(), z0, 1e-3) << "a parked quad must not sink or bounce";
  EXPECT_LT(m.state().position.head<2>().norm(), 1e-6) << "and must not wander off";
  EXPECT_LT(m.state().angular_velocity.norm(), 1e-3) << "and must not spin up";
  const double tilt = 2.0 * std::acos(std::min(1.0, std::abs(m.state().orientation.w())));
  EXPECT_LT(tilt, 1e-3) << "and must stay level";
}

TEST(Multirotor, ParkedQuadAccelerometerReadsOneG) {
  fdt::Multirotor m(config());
  m.placeOnGround();
  m.setMotorCommands(commands(0, 0, 0, 0));
  run(m, 1.0);

  const Eigen::Vector3d a = meanAccelOver(m, 4000);
  EXPECT_NEAR(a.norm(), fdt::kGravity, 0.02);
  EXPECT_NEAR(a.z(), -fdt::kGravity, 0.02) << "the bench pushes up, and up is -z";
  EXPECT_NEAR(a.x(), 0.0, 0.02);
  EXPECT_NEAR(a.y(), 0.0, 0.02);
}

TEST(Multirotor, FullThrottleTakesOff) {
  fdt::Multirotor m(config());
  m.placeOnGround();
  const double z0 = m.state().position.z();
  m.setMotorCommands(commands(1, 1, 1, 1));

  run(m, 1.0);

  EXPECT_FALSE(m.telemetry().in_contact);
  EXPECT_LT(m.state().position.z(), z0 - 1.0) << "full throttle must climb at least a metre in a second";
}

TEST(Multirotor, IdleThrottleDoesNotTakeOff) {
  fdt::Multirotor m(config());
  m.placeOnGround();
  const double z0 = m.state().position.z();
  m.setMotorCommands(commands(0.10, 0.10, 0.10, 0.10));
  run(m, 2.0);
  EXPECT_TRUE(m.telemetry().in_contact);
  EXPECT_NEAR(m.state().position.z(), z0, 0.01);
}

// --- battery --------------------------------------------------------------

TEST(Multirotor, BatteryDrainsAndSagsUnderThrottle) {
  fdt::Multirotor m(config());
  m.reset(airborne(100.0));
  m.setMotorCommands(commands(0.9, 0.9, 0.9, 0.9));

  const double soc0 = m.telemetry().battery_soc;
  run(m, 5.0);

  EXPECT_LT(m.telemetry().battery_soc, soc0);
  EXPECT_GT(m.telemetry().battery_current, 5.0) << "90% throttle must draw real current";
  EXPECT_LT(m.telemetry().battery_voltage, 4.0 * 4.20) << "and must sag the pack";
}

// --- properties Phase 4 depends on ----------------------------------------

TEST(Multirotor, IsBitIdenticalForTheSameSeedAndInputs) {
  const auto cfg = config();
  fdt::Multirotor a(cfg, 42), b(cfg, 42);
  a.reset(airborne(30.0));
  b.reset(airborne(30.0));

  for (int i = 0; i < 10000; ++i) {
    const double t = static_cast<double>(i) * kDt;
    const auto cmd = commands(0.4 + 0.1 * std::sin(t * 7.0), 0.4, 0.5, 0.35 + 0.05 * std::cos(t * 3.0));
    a.setMotorCommands(cmd);
    b.setMotorCommands(cmd);
    a.step(kDt);
    b.step(kDt);
  }

  EXPECT_EQ(a.state().position.x(), b.state().position.x());
  EXPECT_EQ(a.state().position.z(), b.state().position.z());
  EXPECT_EQ(a.state().orientation.w(), b.state().orientation.w());
  EXPECT_EQ(a.state().angular_velocity.y(), b.state().angular_velocity.y());
  EXPECT_EQ(a.telemetry().imu.gyro.x(), b.telemetry().imu.gyro.x())
      << "the noise stream must be reproducible too";
  EXPECT_EQ(a.telemetry().battery_soc, b.telemetry().battery_soc);
}

TEST(Multirotor, ResetRewindsEverything) {
  const auto cfg = config();
  fdt::Multirotor m(cfg, 7);
  m.reset(airborne(30.0));
  m.setMotorCommands(commands(0.8, 0.8, 0.8, 0.8));
  run(m, 2.0);
  ASSERT_LT(m.telemetry().battery_soc, 1.0);

  m.reset(airborne(30.0));
  EXPECT_DOUBLE_EQ(m.time(), 0.0);
  EXPECT_DOUBLE_EQ(m.state().altitude(), 30.0);
  EXPECT_DOUBLE_EQ(m.telemetry().battery_soc, 1.0);
  EXPECT_DOUBLE_EQ(m.motors().speeds()[0], 0.0);
}

TEST(Multirotor, RunsFasterThanRealTime) {
  fdt::Multirotor m(config());
  m.reset(airborne(100.0));
  m.trimHover();

  const double sim_seconds = 60.0;
  const int steps = static_cast<int>(std::lround(sim_seconds / kDt));  // 2 kHz

  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) m.step(kDt);
  const auto t1 = std::chrono::steady_clock::now();

  const double wall = std::chrono::duration<double>(t1 - t0).count();
  const double ratio = sim_seconds / wall;
  std::cout << "[          ] " << steps << " steps of 2 kHz physics in " << wall << " s = " << ratio
            << "x real time (" << (wall / steps) * 1e6 << " us/step)\n";

  EXPECT_GT(ratio, 20.0) << "headless replay needs to be much faster than real time; got " << ratio << "x";
}
