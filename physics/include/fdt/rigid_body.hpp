// Fixed-step 6DOF rigid-body integrator.
//
// State derivative (docs/coordinate_frames.md for the conventions):
//
//   dp/dt = v                                    (world)
//   dv/dt = R_wb * F_body / m + g_world          (world)
//   dq/dt = 0.5 * q (x) [0, omega]               (body rates, so q on the LEFT)
//   dw/dt = I^-1 * (tau - omega x (I omega))     (body)
//
// The gyroscopic term `omega x (I omega)` is what couples the axes of a body
// whose moments of inertia differ; dropping it is a common and hard-to-spot
// error, so there is a test pinning angular-momentum conservation.
#pragma once

#include "fdt/types.hpp"

#include <functional>

namespace fdt {

/// Body-frame wrench as a function of state and of the time offset from the
/// start of the current step. The offset lets sub-models that evolve
/// independently of the body (the motor lag, which is driven only by the
/// commands) be evaluated exactly at each RK4 stage instead of being frozen
/// across the step.
using WrenchFn = std::function<Wrench(const State& state, double t_offset)>;

/// One classical RK4 step. The returned quaternion is renormalised; the
/// intermediate stages are not, which keeps the stage weights exact.
State integrateRK4(const State& state, double dt, const InertiaProperties& inertia, const WrenchFn& wrench);

/// One semi-implicit (symplectic) Euler step. Cheaper and unconditionally
/// better-behaved for stiff contact than explicit Euler; kept for the ground
/// model if the penalty spring ever needs it.
State integrateSemiImplicitEuler(const State& state, double dt, const InertiaProperties& inertia,
                                 const WrenchFn& wrench);

/// Total mechanical energy: translational + rotational kinetic, plus
/// gravitational potential relative to the world origin plane. Used by the
/// energy-conservation test and by the headless runner's sanity output.
double mechanicalEnergy(const State& state, const InertiaProperties& inertia);

/// Angular momentum about the CG, expressed in the WORLD frame. Conserved
/// under zero external torque regardless of how the body tumbles.
Eigen::Vector3d angularMomentumWorld(const State& state, const InertiaProperties& inertia);

}  // namespace fdt
