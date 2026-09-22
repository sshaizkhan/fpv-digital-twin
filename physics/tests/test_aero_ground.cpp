// Aerodynamics and ground contact tests.

#include "fdt/aero.hpp"
#include "fdt/ground.hpp"

#include <gtest/gtest.h>

namespace {

fdt::QuadConfig config() { return fdt::loadQuadConfig(std::string(FDT_REPO_ROOT) + "/config/quad.yaml"); }

}  // namespace

// --- aero -----------------------------------------------------------------

TEST(Aero, DragOpposesVelocityOnEveryAxis) {
  const auto cfg = config();
  fdt::AeroModel a(cfg.aero);

  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    v[axis] = 12.0;
    const fdt::Wrench w = a.wrench(v, Eigen::Vector3d::Zero(), 0.0);
    EXPECT_LT(w.force[axis], 0.0) << "drag must oppose motion on axis " << axis;
    for (int other = 0; other < 3; ++other) {
      if (other != axis) EXPECT_NEAR(w.force[other], 0.0, 1e-15) << "axes must not cross-couple";
    }

    // And it must be symmetric: reversing the flow reverses the force.
    const fdt::Wrench reversed = a.wrench(-v, Eigen::Vector3d::Zero(), 0.0);
    EXPECT_NEAR(reversed.force[axis], -w.force[axis], 1e-15);
  }
}

TEST(Aero, DragIsLinearPlusQuadraticInSpeed) {
  const auto cfg = config();
  fdt::AeroModel a(cfg.aero);

  const double v = 10.0;
  const fdt::Wrench w = a.wrench(Eigen::Vector3d(v, 0, 0), Eigen::Vector3d::Zero(), 0.0);
  const double expected =
      -(cfg.aero.linear_drag.value.x() * v + cfg.aero.quadratic_drag.value.x() * v * v);
  EXPECT_NEAR(w.force.x(), expected, 1e-12);

  // Doubling the speed must more than double the drag, or the quadratic term
  // is not contributing.
  const fdt::Wrench fast = a.wrench(Eigen::Vector3d(2 * v, 0, 0), Eigen::Vector3d::Zero(), 0.0);
  EXPECT_GT(std::abs(fast.force.x()), 2.0 * std::abs(w.force.x()));
}

TEST(Aero, ZeroVelocityProducesZeroForce) {
  const auto cfg = config();
  fdt::AeroModel a(cfg.aero);
  const fdt::Wrench w = a.wrench(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 20.0);
  EXPECT_NEAR(w.force.norm(), 0.0, 1e-15);
  EXPECT_NEAR(w.torque.norm(), 0.0, 1e-15);
}

TEST(Aero, InducedDragScalesWithThrustAndActsOnlyInPlane) {
  const auto cfg = config();
  fdt::AeroModel a(cfg.aero);
  const Eigen::Vector3d v(15.0, 0.0, 3.0);

  const fdt::Wrench unloaded = a.wrench(v, Eigen::Vector3d::Zero(), 0.0);
  const fdt::Wrench loaded = a.wrench(v, Eigen::Vector3d::Zero(), 20.0);

  // Extra backward force while the rotors are loaded.
  EXPECT_LT(loaded.force.x(), unloaded.force.x());
  const double induced = unloaded.force.x() - loaded.force.x();
  EXPECT_NEAR(induced, cfg.aero.induced_drag_k.value * 20.0 * v.x(), 1e-12);

  // And none of it on the vertical axis: induced drag is an in-plane effect.
  EXPECT_NEAR(loaded.force.z(), unloaded.force.z(), 1e-15);

  // Twice the thrust, twice the induced drag.
  const fdt::Wrench heavier = a.wrench(v, Eigen::Vector3d::Zero(), 40.0);
  EXPECT_NEAR(unloaded.force.x() - heavier.force.x(), 2.0 * induced, 1e-12);
}

TEST(Aero, AngularDampingOpposesRotation) {
  const auto cfg = config();
  fdt::AeroModel a(cfg.aero);
  const Eigen::Vector3d omega(10.0, -20.0, 5.0);
  const fdt::Wrench w = a.wrench(Eigen::Vector3d::Zero(), omega, 0.0);

  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(w.torque[i], -cfg.aero.angular_damping.value[i] * omega[i], 1e-15);
    EXPECT_LT(w.torque[i] * omega[i], 0.0) << "damping torque must oppose the rate on axis " << i;
  }
}

// --- ground ---------------------------------------------------------------

namespace {

/// Level quad whose contact points penetrate the surface by `depth`.
fdt::State restingAt(const fdt::QuadConfig& cfg, double depth) {
  fdt::State s;
  s.position.z() = cfg.ground.height.value - cfg.ground.stand_height.value + depth;
  return s;
}

}  // namespace

TEST(Ground, ContactPointsSitUnderEachMotor) {
  const auto cfg = config();
  fdt::GroundModel g(cfg.ground, cfg.motors);
  const auto& pts = g.contactPoints();

  for (const auto id : fdt::kAllMotorIds) {
    const size_t i = static_cast<size_t>(id);
    EXPECT_NEAR(pts[i].x(), cfg.motors.at(id).position.value.x(), 1e-15);
    EXPECT_NEAR(pts[i].y(), cfg.motors.at(id).position.value.y(), 1e-15);
    EXPECT_NEAR(pts[i].z(), cfg.motors.at(id).position.value.z() + cfg.ground.stand_height.value, 1e-15);
    EXPECT_GT(pts[i].z(), 0.0) << "contact points are BELOW the CG, and +z is down";
  }
}

TEST(Ground, AirborneQuadFeelsNothing) {
  const auto cfg = config();
  fdt::GroundModel g(cfg.ground, cfg.motors);

  fdt::State s;
  s.position.z() = -5.0;  // 5 m up
  EXPECT_FALSE(g.inContact(s));
  const fdt::Wrench w = g.wrench(s);
  EXPECT_NEAR(w.force.norm(), 0.0, 1e-15);
  EXPECT_NEAR(w.torque.norm(), 0.0, 1e-15);
}

TEST(Ground, PenetrationPushesUpProportionallyToDepth) {
  const auto cfg = config();
  fdt::GroundModel g(cfg.ground, cfg.motors);

  const double depth = 0.002;
  const fdt::State s = restingAt(cfg, depth);
  ASSERT_TRUE(g.inContact(s));
  EXPECT_NEAR(g.penetration(s), depth, 1e-12);

  const fdt::Wrench w = g.wrench(s);
  // Four contact points, each a spring of stiffness k.
  EXPECT_NEAR(w.force.z(), -4.0 * cfg.ground.contact_stiffness.value * depth, 1e-9);
  EXPECT_LT(w.force.z(), 0.0) << "the ground pushes UP, which is -z";
  EXPECT_NEAR(w.torque.norm(), 0.0, 1e-9) << "a level quad on level ground gets no torque";
}

TEST(Ground, SupportsExactlyTheQuadsWeightAtItsEquilibriumDepth) {
  const auto cfg = config();
  fdt::GroundModel g(cfg.ground, cfg.motors);

  const double weight = cfg.mass.auw.value * fdt::kGravity;
  const double depth = weight / (4.0 * cfg.ground.contact_stiffness.value);
  const fdt::Wrench w = g.wrench(restingAt(cfg, depth));

  EXPECT_NEAR(-w.force.z(), weight, 1e-9);
  EXPECT_LT(depth, 0.005) << "the quad must not visibly sink into the ground; it sank " << depth * 1000
                          << " mm";
}

TEST(Ground, DampingResistsSinkingButDoesNotStick) {
  const auto cfg = config();
  fdt::GroundModel g(cfg.ground, cfg.motors);
  const double depth = 0.001;

  fdt::State sinking = restingAt(cfg, depth);
  sinking.velocity.z() = 2.0;  // moving down
  fdt::State rising = restingAt(cfg, depth);
  rising.velocity.z() = -2.0;  // moving up

  const double f_sinking = -g.wrench(sinking).force.z();
  const double f_static = -g.wrench(restingAt(cfg, depth)).force.z();
  const double f_rising = -g.wrench(rising).force.z();

  EXPECT_GT(f_sinking, f_static) << "damping must add force against a descent";
  EXPECT_LT(f_rising, f_static) << "and take it away on the rebound";
  EXPECT_GE(f_rising, 0.0) << "the ground must never PULL the quad down";
}

TEST(Ground, NeverPullsDownNoMatterHowFastTheQuadLeaves) {
  const auto cfg = config();
  fdt::GroundModel g(cfg.ground, cfg.motors);
  fdt::State s = restingAt(cfg, 1e-5);
  s.velocity.z() = -50.0;  // leaving very fast
  EXPECT_GE(-g.wrench(s).force.z(), 0.0);
}

TEST(Ground, TiltedQuadGetsARestoringTorque) {
  const auto cfg = config();
  fdt::GroundModel g(cfg.ground, cfg.motors);

  // Roll right by 10 degrees while sitting on the ground: only the right-hand
  // contact points bite, and they must roll it back to the left.
  fdt::State s = restingAt(cfg, 0.004);
  const double roll = 10.0 * M_PI / 180.0;
  s.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

  const fdt::Wrench w = g.wrench(s);
  EXPECT_LT(w.torque.x(), 0.0) << "a right-rolled quad on the ground must be pushed back left";
  EXPECT_NEAR(w.torque.y(), 0.0, 1e-9) << "a pure roll must not produce pitch";
}

TEST(Ground, FrictionOpposesSlidingAndIsBoundedByMuTimesNormal) {
  const auto cfg = config();
  fdt::GroundModel g(cfg.ground, cfg.motors);

  fdt::State s = restingAt(cfg, 0.004);
  s.velocity = Eigen::Vector3d(3.0, 0.0, 0.0);  // sliding north

  const fdt::Wrench w = g.wrench(s);
  EXPECT_LT(w.force.x(), 0.0) << "friction must oppose the slide";

  const double normal = -w.force.z();
  EXPECT_LE(std::abs(w.force.x()), cfg.ground.friction.value * normal + 1e-9)
      << "friction must not exceed the Coulomb limit";
}

TEST(Ground, StationaryQuadHasNoFriction) {
  const auto cfg = config();
  fdt::GroundModel g(cfg.ground, cfg.motors);
  const fdt::Wrench w = g.wrench(restingAt(cfg, 0.004));
  EXPECT_NEAR(w.force.x(), 0.0, 1e-12);
  EXPECT_NEAR(w.force.y(), 0.0, 1e-12);
}
