// Ground contact: a penalty spring-damper at four contact points, one under
// each motor, so that a parked quad sits level and a landing on one arm tips
// the way a real one does.
//
// This is a stability-tuned penalty model, not a realistic impact model. It
// exists so the sim can sit on the ground, take off and land without the
// integrator blowing up; do not read anything into a crash it produces.
#pragma once

#include "fdt/quad_config.hpp"
#include "fdt/types.hpp"

#include <array>

namespace fdt {

class GroundModel {
 public:
  GroundModel(const Ground& config, const Motors& motors);

  /// Body-frame force and torque from all contact points.
  Wrench wrench(const State& state) const;

  /// True if any contact point is at or below the surface.
  bool inContact(const State& state) const;

  /// Deepest penetration of any contact point, metres; negative when clear.
  double penetration(const State& state) const;

  /// Contact points in the body frame: under each motor, `stand_height` below
  /// the CG (body +z is down).
  const std::array<Eigen::Vector3d, 4>& contactPoints() const { return contact_points_; }

 private:
  Ground config_;
  std::array<Eigen::Vector3d, 4> contact_points_{};
};

}  // namespace fdt
