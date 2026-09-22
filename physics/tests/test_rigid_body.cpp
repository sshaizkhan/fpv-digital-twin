// Rigid-body integrator tests.
//
// These cover the Phase 1 acceptance criteria that belong to the integrator
// itself (free fall = g, hover balance, pure roll torque, energy sane) plus
// the properties that catch a wrong sign or a dropped term.

#include "fdt/rigid_body.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using fdt::InertiaProperties;
using fdt::State;
using fdt::Wrench;

// Representative of the quad in config/quad.yaml.
InertiaProperties testInertia() {
  Eigen::Matrix3d I = Eigen::Vector3d(0.00175, 0.00175, 0.00336).asDiagonal();
  return InertiaProperties::fromMassAndInertia(0.720, I);
}

fdt::WrenchFn constantWrench(const Eigen::Vector3d& force_body, const Eigen::Vector3d& torque_body) {
  return [force_body, torque_body](const State&, double) {
    Wrench w;
    w.force = force_body;
    w.torque = torque_body;
    return w;
  };
}

const fdt::WrenchFn kNoWrench = constantWrench(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

State runFor(State s, double duration, double dt, const InertiaProperties& inertia, const fdt::WrenchFn& w) {
  const int steps = static_cast<int>(std::lround(duration / dt));
  for (int i = 0; i < steps; ++i) s = fdt::integrateRK4(s, dt, inertia, w);
  return s;
}

}  // namespace

// --- acceptance criterion: free fall = g ----------------------------------

TEST(RigidBody, FreeFallAcceleratesAtExactlyG) {
  const auto inertia = testInertia();
  const double dt = 1.0 / 1000.0;
  const State end = runFor(State{}, 1.0, dt, inertia, kNoWrench);

  // RK4 is exact for a constant acceleration, so this is an equality test up
  // to floating-point accumulation over 1000 steps, not a physics tolerance.
  EXPECT_NEAR(end.velocity.z(), fdt::kGravity, 1e-12);
  EXPECT_NEAR(end.velocity.x(), 0.0, 1e-15);
  EXPECT_NEAR(end.velocity.y(), 0.0, 1e-15);

  // Down is +z in NED, so a falling body's z increases and its altitude drops.
  EXPECT_NEAR(end.position.z(), 0.5 * fdt::kGravity, 1e-12);
  EXPECT_NEAR(end.altitude(), -0.5 * fdt::kGravity, 1e-12);
  EXPECT_GT(end.position.z(), 0.0) << "NED sign error: a falling body must gain +z";

  // Free fall applies no torque, so the attitude must not drift.
  EXPECT_NEAR(end.angular_velocity.norm(), 0.0, 1e-15);
  EXPECT_NEAR(std::abs(end.orientation.w()), 1.0, 1e-15);
}

// --- acceptance criterion: hover thrust balances weight -------------------

TEST(RigidBody, ThrustBalancingWeightHoldsPositionAndAltitude) {
  const auto inertia = testInertia();
  // Thrust acts along body -z (up). See docs/coordinate_frames.md §5.
  const auto hover = constantWrench(Eigen::Vector3d(0.0, 0.0, -inertia.mass * fdt::kGravity),
                                    Eigen::Vector3d::Zero());
  const State end = runFor(State{}, 10.0, 1.0 / 1000.0, inertia, hover);

  EXPECT_NEAR(end.position.norm(), 0.0, 1e-9) << "hover must not drift";
  EXPECT_NEAR(end.velocity.norm(), 0.0, 1e-9);
}

TEST(RigidBody, ThrustSignIsUpNotDown) {
  // A guard against the single most likely sign error in the whole project.
  const auto inertia = testInertia();
  const auto double_hover = constantWrench(Eigen::Vector3d(0.0, 0.0, -2.0 * inertia.mass * fdt::kGravity),
                                           Eigen::Vector3d::Zero());
  const State end = runFor(State{}, 1.0, 1.0 / 1000.0, inertia, double_hover);
  EXPECT_GT(end.altitude(), 0.0) << "body -z thrust must climb";
  EXPECT_NEAR(end.altitude(), 0.5 * fdt::kGravity, 1e-9);
}

// --- acceptance criterion: pure roll torque -------------------------------

TEST(RigidBody, PureRollTorqueGivesExpectedAngularAcceleration) {
  const auto inertia = testInertia();
  const double tau = 0.05;  // N*m about body +x
  const auto w = constantWrench(Eigen::Vector3d::Zero(), Eigen::Vector3d(tau, 0.0, 0.0));

  const double t = 0.25;
  const State end = runFor(State{}, t, 1.0 / 1000.0, inertia, w);

  // omega is along a principal axis, so the gyroscopic term stays zero and
  // the motion is exactly omega_dot = tau / Ixx.
  const double expected_rate = tau / inertia.inertia(0, 0) * t;
  EXPECT_NEAR(end.angular_velocity.x(), expected_rate, 1e-9);
  EXPECT_NEAR(end.angular_velocity.y(), 0.0, 1e-12);
  EXPECT_NEAR(end.angular_velocity.z(), 0.0, 1e-12);

  // Positive p is roll right: the right wing (body +y) must end up below the
  // horizon, i.e. with a positive world-z (down) component.
  const Eigen::Vector3d right_wing_world = end.orientation * Eigen::Vector3d::UnitY();
  EXPECT_GT(right_wing_world.z(), 0.0) << "positive roll rate must drop the right wing";
}

TEST(RigidBody, PitchAndYawTorquesUseTheDocumentedSigns) {
  const auto inertia = testInertia();
  const double t = 0.2;

  // Positive q (about body +y) is nose UP.
  const State pitched =
      runFor(State{}, t, 1.0 / 1000.0, inertia, constantWrench(Eigen::Vector3d::Zero(), {0.0, 0.02, 0.0}));
  EXPECT_GT(pitched.angular_velocity.y(), 0.0);
  const Eigen::Vector3d nose_world = pitched.orientation * Eigen::Vector3d::UnitX();
  EXPECT_LT(nose_world.z(), 0.0) << "positive pitch rate must raise the nose (-z is up)";

  // Positive r (about body +z) is yaw RIGHT.
  const State yawed =
      runFor(State{}, t, 1.0 / 1000.0, inertia, constantWrench(Eigen::Vector3d::Zero(), {0.0, 0.0, 0.02}));
  EXPECT_GT(yawed.angular_velocity.z(), 0.0);
  const Eigen::Vector3d yawed_nose = yawed.orientation * Eigen::Vector3d::UnitX();
  EXPECT_GT(yawed_nose.y(), 0.0) << "positive yaw rate must swing the nose toward +y (East/right)";
}

// --- acceptance criterion: energy sane ------------------------------------

TEST(RigidBody, BallisticFlightConservesMechanicalEnergy) {
  const auto inertia = testInertia();
  State s;
  s.velocity = Eigen::Vector3d(3.0, -2.0, -8.0);        // thrown up and along
  s.angular_velocity = Eigen::Vector3d(4.0, -7.0, 2.0);  // and tumbling

  const double e0 = fdt::mechanicalEnergy(s, inertia);
  const State end = runFor(s, 10.0, 1.0 / 1000.0, inertia, kNoWrench);
  const double e1 = fdt::mechanicalEnergy(end, inertia);

  EXPECT_GT(std::abs(e0), 1.0) << "test is meaningless if the energy is ~0";
  EXPECT_NEAR(std::abs(e1 - e0) / std::abs(e0), 0.0, 1e-4) << "energy drift must stay under 0.01%";
}

TEST(RigidBody, TumblingBodyConservesAngularMomentumAndRotationalEnergy) {
  // The real test of the omega x (I omega) term: with Ixx == Iyy != Izz and
  // omega off every principal axis, the body rates must keep changing while
  // the world-frame angular momentum stays put.
  const auto inertia = testInertia();
  State s;
  s.angular_velocity = Eigen::Vector3d(12.0, 5.0, 9.0);

  const Eigen::Vector3d L0 = fdt::angularMomentumWorld(s, inertia);
  const double ke0 = 0.5 * s.angular_velocity.dot(inertia.inertia * s.angular_velocity);

  const State end = runFor(s, 5.0, 1.0 / 2000.0, inertia, kNoWrench);

  const Eigen::Vector3d L1 = fdt::angularMomentumWorld(end, inertia);
  const double ke1 = 0.5 * end.angular_velocity.dot(inertia.inertia * end.angular_velocity);

  EXPECT_NEAR((L1 - L0).norm() / L0.norm(), 0.0, 1e-6) << "angular momentum must be conserved";
  EXPECT_NEAR(std::abs(ke1 - ke0) / ke0, 0.0, 1e-6) << "rotational energy must be conserved";

  // And the rates must genuinely have moved, or the test proves nothing.
  EXPECT_GT((end.angular_velocity - s.angular_velocity).norm(), 1.0)
      << "expected nutation; if the rates are frozen the gyroscopic term is missing";
}

// --- integrator properties ------------------------------------------------

TEST(RigidBody, QuaternionStaysNormalised) {
  const auto inertia = testInertia();
  State s;
  s.angular_velocity = Eigen::Vector3d(20.0, -15.0, 30.0);
  const auto w = constantWrench(Eigen::Vector3d(0.3, -0.2, -7.0), Eigen::Vector3d(0.01, 0.02, -0.005));

  for (int i = 0; i < 20000; ++i) {
    s = fdt::integrateRK4(s, 1.0 / 2000.0, inertia, w);
    ASSERT_NEAR(s.orientation.norm(), 1.0, 1e-12) << "drifted at step " << i;
  }
}

TEST(RigidBody, ConstantBodyRateReturnsToIdentityAfterOneRevolution) {
  const auto inertia = testInertia();
  State s;
  s.angular_velocity = Eigen::Vector3d(0.0, 0.0, 2.0 * M_PI);  // one turn per second about +z

  const State end = runFor(s, 1.0, 1.0 / 2000.0, inertia, kNoWrench);

  // q and -q are the same rotation, so compare the rotation angle, not the
  // components.
  const double angle = 2.0 * std::acos(std::min(1.0, std::abs(end.orientation.w())));
  EXPECT_NEAR(angle, 0.0, 1e-9);
  EXPECT_NEAR(end.angular_velocity.z(), 2.0 * M_PI, 1e-12);
}

TEST(RigidBody, IsFourthOrderAccurate) {
  // Halving the step must cut the error by roughly 16x. Catches an integrator
  // that silently degraded to Euler or to RK2.
  const auto inertia = testInertia();
  State s;
  s.angular_velocity = Eigen::Vector3d(9.0, 4.0, 6.0);
  const auto w = constantWrench(Eigen::Vector3d(0.5, 0.4, -6.0), Eigen::Vector3d(0.02, -0.01, 0.004));

  const double T = 1.0;
  const State reference = runFor(s, T, T / 640000.0, inertia, w);

  auto errorAt = [&](double dt) {
    const State end = runFor(s, T, dt, inertia, w);
    return (end.angular_velocity - reference.angular_velocity).norm() +
           (end.position - reference.position).norm();
  };

  const double e_coarse = errorAt(T / 500.0);
  const double e_fine = errorAt(T / 1000.0);
  EXPECT_GT(e_coarse, 0.0);
  EXPECT_GT(e_coarse / e_fine, 8.0) << "expected ~16x; got " << (e_coarse / e_fine)
                                    << ", which is not fourth-order behaviour";
}

TEST(RigidBody, SemiImplicitEulerAlsoFreeFallsAtG) {
  // Lower order, so a loose tolerance; this only pins the frames and signs.
  const auto inertia = testInertia();
  State s;
  const double dt = 1.0 / 2000.0;
  for (int i = 0; i < 2000; ++i) s = fdt::integrateSemiImplicitEuler(s, dt, inertia, kNoWrench);
  EXPECT_NEAR(s.velocity.z(), fdt::kGravity, 1e-9);
  EXPECT_NEAR(s.position.z(), 0.5 * fdt::kGravity, 1e-2);
}

TEST(RigidBody, WrenchCallbackSeesTheSubStepTimeOffsets) {
  // RK4 must sample the wrench at t, t+dt/2, t+dt/2, t+dt. A model whose force
  // varies within the step (the motor lag) depends on this.
  const auto inertia = testInertia();
  std::vector<double> offsets;
  const fdt::WrenchFn probe = [&offsets](const State&, double t_offset) {
    offsets.push_back(t_offset);
    return Wrench{};
  };
  const double dt = 0.001;
  fdt::integrateRK4(State{}, dt, inertia, probe);

  ASSERT_EQ(offsets.size(), 4u);
  EXPECT_DOUBLE_EQ(offsets[0], 0.0);
  EXPECT_DOUBLE_EQ(offsets[1], dt / 2.0);
  EXPECT_DOUBLE_EQ(offsets[2], dt / 2.0);
  EXPECT_DOUBLE_EQ(offsets[3], dt);
}
