// Airframe aerodynamics: body drag plus the induced drag that makes a real
// quad sink when you push it forward fast, plus rate damping.
//
// Drag is applied per body axis (v_i * |v_i|, not v_i * |v|) so that the three
// axes stay independent and each can be fitted separately against a real log.
#pragma once

#include "fdt/quad_config.hpp"
#include "fdt/types.hpp"

namespace fdt {

class AeroModel {
 public:
  explicit AeroModel(const Aero& config);

  /// Body-frame force and torque.
  ///
  /// `velocity_body`  airspeed of the CG in the body frame, m/s
  /// `angular_velocity`  body rates, rad/s
  /// `total_thrust`  total rotor thrust in newtons, positive; the induced-drag
  ///                 term scales with it because it is the rotor disc, not the
  ///                 frame, that produces it.
  Wrench wrench(const Eigen::Vector3d& velocity_body, const Eigen::Vector3d& angular_velocity,
                double total_thrust) const;

 private:
  Aero config_;
};

}  // namespace fdt
