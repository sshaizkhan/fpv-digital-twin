// Tests for the quad.yaml loader.
//
// Most tests load the *shipped* config/quad.yaml, so this suite doubles as a
// schema check on the real file: if someone edits config/quad.yaml into
// something the physics core cannot use, these fail.

#include "fdt/quad_config.hpp"

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string repoPath(const std::string& rel) { return std::string(FDT_REPO_ROOT) + "/" + rel; }

std::string shippedConfigText() {
  std::ifstream in(repoPath("config/quad.yaml"));
  EXPECT_TRUE(in.good()) << "cannot open shipped config/quad.yaml";
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// Replace `from` with `to`, asserting `from` occurs exactly once. Keeps the
// negative tests honest: if the schema is reworded the test fails loudly
// instead of silently testing nothing.
std::string withSubstitution(const std::string& text, const std::string& from, const std::string& to) {
  const auto first = text.find(from);
  EXPECT_NE(first, std::string::npos) << "substitution target not found: " << from;
  if (first == std::string::npos) return text;
  EXPECT_EQ(text.find(from, first + from.size()), std::string::npos)
      << "substitution target is not unique: " << from;
  std::string out = text;
  out.replace(first, from.size(), to);
  return out;
}

bool contains(const std::vector<std::string>& v, const std::string& s) {
  return std::find(v.begin(), v.end(), s) != v.end();
}

constexpr double kDeg2Rad = M_PI / 180.0;

}  // namespace

// --- happy path -----------------------------------------------------------

TEST(QuadConfig, LoadsShippedConfig) {
  const auto cfg = fdt::loadQuadConfig(repoPath("config/quad.yaml"));
  EXPECT_EQ(cfg.schema_version, 1);
  EXPECT_EQ(cfg.meta.frame_convention, "FRD_NED");
  EXPECT_FALSE(cfg.meta.name.empty());
}

TEST(QuadConfig, ParsesMassProperties) {
  const auto cfg = fdt::loadQuadConfig(repoPath("config/quad.yaml"));
  EXPECT_DOUBLE_EQ(cfg.mass.auw.value, 0.720);
  EXPECT_EQ(cfg.mass.auw.units, "kg");
  EXPECT_FALSE(cfg.mass.auw.measured);
  EXPECT_EQ(cfg.mass.auw.path, "mass.auw");

  const Eigen::Matrix3d I = cfg.mass.inertiaMatrix();
  EXPECT_DOUBLE_EQ(I(0, 0), cfg.mass.inertia_diag.value.x());
  EXPECT_DOUBLE_EQ(I(1, 1), cfg.mass.inertia_diag.value.y());
  EXPECT_DOUBLE_EQ(I(2, 2), cfg.mass.inertia_diag.value.z());
  EXPECT_DOUBLE_EQ(I(0, 1), I(1, 0));  // symmetric
  EXPECT_TRUE(I.isApprox(I.transpose()));
  // A physical inertia tensor must be positive definite.
  EXPECT_GT(I.selfadjointView<Eigen::Upper>().eigenvalues().minCoeff(), 0.0);
}

TEST(QuadConfig, ParsesMotorGeometryInBodyFrame) {
  const auto cfg = fdt::loadQuadConfig(repoPath("config/quad.yaml"));

  // Body FRD: +x forward, +y right. Front-left is +x, -y.
  const auto& fl = cfg.motors.at(fdt::MotorId::FL);
  EXPECT_GT(fl.position.value.x(), 0.0) << "FL must be forward of the CG";
  EXPECT_LT(fl.position.value.y(), 0.0) << "FL must be left of the CG (-y)";

  const auto& rr = cfg.motors.at(fdt::MotorId::RR);
  EXPECT_LT(rr.position.value.x(), 0.0);
  EXPECT_GT(rr.position.value.y(), 0.0);

  // X-frame symmetry: the four arms cancel about the CG.
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto id : fdt::kAllMotorIds) sum += cfg.motors.at(id).position.value;
  EXPECT_NEAR(sum.norm(), 0.0, 1e-9);
}

TEST(QuadConfig, SpinDirectionsAreBalanced) {
  const auto cfg = fdt::loadQuadConfig(repoPath("config/quad.yaml"));
  int cw = 0;
  for (const auto id : fdt::kAllMotorIds) cw += (cfg.motors.at(id).spin == fdt::Spin::CW) ? 1 : 0;
  EXPECT_EQ(cw, 2) << "a quad must have two CW and two CCW props or it cannot hold yaw";

  // Diagonal motors spin the same way.
  EXPECT_EQ(cfg.motors.at(fdt::MotorId::FL).spin, cfg.motors.at(fdt::MotorId::RR).spin);
  EXPECT_EQ(cfg.motors.at(fdt::MotorId::FR).spin, cfg.motors.at(fdt::MotorId::RL).spin);

  // Sign convention, as seen from above (docs/coordinate_frames.md §5).
  EXPECT_DOUBLE_EQ(fdt::spinSign(fdt::Spin::CW), +1.0);
  EXPECT_DOUBLE_EQ(fdt::spinSign(fdt::Spin::CCW), -1.0);
}

TEST(QuadConfig, BetaflightOrderIsAPermutationAndFlaggedUnverified) {
  const auto cfg = fdt::loadQuadConfig(repoPath("config/quad.yaml"));

  // Nothing may rely on this mapping until Phase 2 checks it against the
  // Betaflight source. The flag is what stops that happening by accident.
  EXPECT_FALSE(cfg.motors.order_verified);
  EXPECT_FALSE(cfg.motors.spin_verified);

  std::vector<fdt::MotorId> seen;
  for (int bf = 1; bf <= 4; ++bf) seen.push_back(cfg.motors.motorForBetaflightIndex(bf));
  for (const auto id : fdt::kAllMotorIds) {
    EXPECT_EQ(std::count(seen.begin(), seen.end(), id), 1) << "motor " << fdt::toString(id);
  }
}

TEST(QuadConfig, CameraAnglesAreConvertedToRadians) {
  const auto cfg = fdt::loadQuadConfig(repoPath("config/quad.yaml"));
  EXPECT_NEAR(cfg.camera.tilt.value, 30.0 * kDeg2Rad, 1e-12);
  EXPECT_EQ(cfg.camera.tilt.units, "rad") << "loader must record the converted unit";
  EXPECT_NEAR(cfg.camera.fov_horizontal.value, 120.0 * kDeg2Rad, 1e-12);
}

TEST(QuadConfig, BatteryOcvInterpolatesAndClamps) {
  const auto cfg = fdt::loadQuadConfig(repoPath("config/quad.yaml"));
  EXPECT_EQ(cfg.battery.cells.value, 4);
  EXPECT_DOUBLE_EQ(cfg.battery.cellOcv(1.0), 4.20);
  EXPECT_DOUBLE_EQ(cfg.battery.cellOcv(0.0), 3.30);
  EXPECT_DOUBLE_EQ(cfg.battery.cellOcv(2.0), 4.20) << "must clamp above full";
  EXPECT_DOUBLE_EQ(cfg.battery.cellOcv(-1.0), 3.30) << "must clamp below empty";
  // Halfway between the 0.50 (3.83) and 0.60 (3.87) points.
  EXPECT_NEAR(cfg.battery.cellOcv(0.55), 3.85, 1e-12);
  EXPECT_NEAR(cfg.battery.packOcv(1.0), 4.0 * 4.20, 1e-12);
}

// --- provenance -----------------------------------------------------------

TEST(QuadConfig, UnmeasuredParametersAreListedWithDottedPaths) {
  const auto cfg = fdt::loadQuadConfig(repoPath("config/quad.yaml"));

  EXPECT_FALSE(cfg.unmeasured.empty())
      << "the shipped config is mostly estimates; an empty list means the loader "
         "is not actually tracking provenance";
  EXPECT_TRUE(contains(cfg.unmeasured, "mass.auw"));
  EXPECT_TRUE(contains(cfg.unmeasured, "mass.inertia_diag"));
  EXPECT_TRUE(contains(cfg.unmeasured, "motors.model.thrust_coeff"));
  EXPECT_TRUE(contains(cfg.unmeasured, "motors.geometry.FL"));

  // kv is off the motor spec sheet, so it is measured and must not be listed.
  EXPECT_FALSE(contains(cfg.unmeasured, "motors.model.kv"));
  EXPECT_FALSE(contains(cfg.unmeasured, "battery.cells"));
}

// --- the schema is enforced, not merely documented -------------------------

TEST(QuadConfig, MissingMeasuredFieldIsAnError) {
  const auto text = withSubstitution(shippedConfigText(),
                                     "    value: 0.720\n    units: kg\n    measured: false\n",
                                     "    value: 0.720\n    units: kg\n");
  try {
    fdt::parseQuadConfig(text, "<test>");
    FAIL() << "a parameter block without `measured:` must be rejected";
  } catch (const fdt::ConfigError& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("mass.auw"), std::string::npos) << msg;
    EXPECT_NE(msg.find("measured"), std::string::npos) << msg;
  }
}

TEST(QuadConfig, MissingUnitsFieldIsAnError) {
  const auto text = withSubstitution(shippedConfigText(),
                                     "    value: 0.720\n    units: kg\n",
                                     "    value: 0.720\n");
  EXPECT_THROW(fdt::parseQuadConfig(text, "<test>"), fdt::ConfigError);
}

TEST(QuadConfig, WrongFrameConventionIsAnError) {
  const auto text =
      withSubstitution(shippedConfigText(), "frame_convention: \"FRD_NED\"", "frame_convention: \"ENU_FLU\"");
  EXPECT_THROW(fdt::parseQuadConfig(text, "<test>"), fdt::ConfigError);
}

TEST(QuadConfig, NonPositiveMassIsAnError) {
  const auto text = withSubstitution(shippedConfigText(), "value: 0.720", "value: -0.720");
  EXPECT_THROW(fdt::parseQuadConfig(text, "<test>"), fdt::ConfigError);
}

TEST(QuadConfig, ThreeCwPropsIsAnError) {
  const auto text = withSubstitution(shippedConfigText(), "    FR: CCW", "    FR: CW");
  EXPECT_THROW(fdt::parseQuadConfig(text, "<test>"), fdt::ConfigError);
}

TEST(QuadConfig, NonMonotonicOcvCurveIsAnError) {
  const auto text = withSubstitution(shippedConfigText(), "{soc: 0.90, v: 4.08}", "{soc: 0.90, v: 3.00}");
  EXPECT_THROW(fdt::parseQuadConfig(text, "<test>"), fdt::ConfigError);
}

TEST(QuadConfig, DuplicateBetaflightMotorMappingIsAnError) {
  const auto text = withSubstitution(shippedConfigText(), "    2: FR", "    2: RR");
  EXPECT_THROW(fdt::parseQuadConfig(text, "<test>"), fdt::ConfigError);
}

TEST(QuadConfig, UnknownSchemaVersionIsAnError) {
  const auto text = withSubstitution(shippedConfigText(), "schema_version: 1", "schema_version: 99");
  EXPECT_THROW(fdt::parseQuadConfig(text, "<test>"), fdt::ConfigError);
}

TEST(QuadConfig, MissingFileIsAnError) {
  EXPECT_THROW(fdt::loadQuadConfig(repoPath("config/does_not_exist.yaml")), fdt::ConfigError);
}

TEST(QuadConfig, FirmwareVersionIsUnsetSoPhase2IsBlocked) {
  // Not an error: Phase 0 ships without it. But it must be visibly absent
  // rather than silently defaulted, so Phase 2 has something to check.
  const auto cfg = fdt::loadQuadConfig(repoPath("config/quad.yaml"));
  EXPECT_FALSE(cfg.firmware.betaflight_version.has_value())
      << "if this now has a value, wire up the submodule pin check";
  EXPECT_EQ(cfg.firmware.target, "SPEEDYBEEF405V4");
}
