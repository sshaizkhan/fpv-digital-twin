// Typed view of config/quad.yaml.
//
// Conventions (body FRD, world NED, SI units): docs/coordinate_frames.md
//
// Every physical parameter carries its provenance. `QuadConfig::unmeasured`
// lists the dotted path of every parameter whose `measured:` flag is false, so
// that a fit or a flight-test comparison can say exactly which numbers are
// still guesses. The loader rejects a parameter block that omits the flag.
#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace fdt {

/// Thrown for anything wrong with the config: unreadable file, bad YAML,
/// missing key, missing provenance, or a value that is not physically usable.
class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// A single parameter plus where it came from.
template <typename T>
struct Param {
  T value{};
  std::string units;    ///< SI unit string as written in the YAML
  bool measured = false;///< false => estimate/placeholder, see `unmeasured`
  std::string source;   ///< free text: how the number was obtained
  std::string path;     ///< dotted path in the YAML, e.g. "mass.auw"
};

using ParamD = Param<double>;
using ParamI = Param<int>;
using ParamV3 = Param<Eigen::Vector3d>;

// --- motors ---------------------------------------------------------------

/// Physical motor positions, named as seen from above with the nose away.
enum class MotorId { FL = 0, FR = 1, RL = 2, RR = 3 };

inline constexpr std::array<MotorId, 4> kAllMotorIds{MotorId::FL, MotorId::FR, MotorId::RL, MotorId::RR};

const char* toString(MotorId id);
/// Throws ConfigError on an unknown name.
MotorId motorIdFromString(const std::string& name);

/// Prop rotation **as seen from above the quad** (looking along body +z).
/// See docs/coordinate_frames.md §5 for why the sign falls out this way.
enum class Spin { CW, CCW };

/// +1 for CW-from-above, -1 for CCW. The airframe yaw reaction torque about
/// body +z is `-spinSign(s) * kQ * omega^2`.
inline constexpr double spinSign(Spin s) { return s == Spin::CW ? 1.0 : -1.0; }

const char* toString(Spin s);

struct MotorGeometry {
  ParamV3 position;  ///< from the CG, body frame, metres
  Spin spin = Spin::CW;
};

struct MotorModel {
  ParamD kv;              ///< rpm/V, unloaded
  ParamI poles;           ///< magnet poles; DSHOT eRPM / (poles/2) = mechanical RPM
  ParamD load_factor;     ///< loaded/unloaded rotor speed at full throttle, (0, 1]
  ParamD thrust_coeff;    ///< kT, N/(rad/s)^2
  ParamD torque_coeff;    ///< kQ, N*m/(rad/s)^2
  ParamD time_constant;   ///< s, first-order command -> rotor speed lag
  ParamD rotor_inertia;   ///< kg*m^2, bell + prop about the spin axis
  ParamD resistance;      ///< ohm
  ParamD max_rpm_safety;  ///< rpm, non-physical clamp
};

struct Motors {
  std::array<MotorGeometry, 4> geometry{};  ///< indexed by MotorId
  MotorModel model;

  bool spin_verified = false;   ///< spin directions checked against the real quad?
  bool order_verified = false;  ///< BF index mapping checked against BF source?

  const MotorGeometry& at(MotorId id) const { return geometry[static_cast<size_t>(id)]; }
  MotorGeometry& at(MotorId id) { return geometry[static_cast<size_t>(id)]; }

  /// Betaflight motor index (1..4) -> physical motor. Throws if out of range.
  /// UNVERIFIED until Phase 2; check `order_verified` before relying on it.
  MotorId motorForBetaflightIndex(int bf_index) const;

 private:
  friend struct QuadConfigParser;
  std::array<MotorId, 4> bf_order_{};  ///< bf_order_[bf_index - 1]
};

// --- the rest of the model ------------------------------------------------

struct MassProperties {
  ParamD auw;               ///< kg, all-up weight
  ParamV3 cg_offset;        ///< m, CG relative to the motor-square centre
  ParamV3 inertia_diag;     ///< kg*m^2, [Ixx, Iyy, Izz] about the CG
  ParamV3 inertia_products; ///< kg*m^2, [Ixy, Ixz, Iyz]

  /// Full symmetric inertia tensor about the CG, body axes.
  /// Note the sign convention: products of inertia enter negated.
  Eigen::Matrix3d inertiaMatrix() const;
};

struct OcvPoint {
  double soc;  ///< state of charge, 0..1
  double v;    ///< open-circuit volts per cell
};

struct Battery {
  ParamI cells;
  ParamD capacity;             ///< A*h
  ParamD internal_resistance;  ///< ohm, whole pack
  ParamD initial_soc;          ///< fraction
  std::vector<OcvPoint> ocv_curve;
  bool ocv_measured = false;
  std::string ocv_source;

  /// Per-cell open-circuit voltage, linearly interpolated, clamped at both ends.
  double cellOcv(double soc) const;
  double packOcv(double soc) const { return cells.value * cellOcv(soc); }
};

struct Aero {
  ParamV3 linear_drag;      ///< N/(m/s), body axes
  ParamV3 quadratic_drag;   ///< N/(m/s)^2, body axes
  ParamV3 angular_damping;  ///< N*m/(rad/s), body axes
  ParamD induced_drag_k;    ///< 1/(m/s)
  ParamD air_density;       ///< kg/m^3
};

struct Imu {
  ParamD sample_rate;           ///< Hz
  ParamD gyro_noise_density;    ///< rad/s/sqrt(Hz)
  ParamV3 gyro_bias;            ///< rad/s
  ParamD gyro_bias_walk;        ///< rad/s/sqrt(s)
  ParamD accel_noise_density;   ///< m/s^2/sqrt(Hz)
  ParamV3 accel_bias;           ///< m/s^2
  ParamD vibration_enabled;     ///< 0/1
};

struct Ground {
  ParamD height;             ///< m, world NED z of the surface
  ParamD contact_stiffness;  ///< N/m
  ParamD contact_damping;    ///< N/(m/s)
  ParamD friction;           ///< dimensionless
  ParamD stand_height;       ///< m, lowest contact point below the CG (body +z)
};

struct Camera {
  ParamD tilt;            ///< RADIANS (YAML gives degrees; converted at load)
  ParamD fov_horizontal;  ///< RADIANS
  ParamV3 position;       ///< m, from the CG, body frame
};

/// UDP/TCP endpoints. The SITL ports are UNVERIFIED until Phase 2 reads them
/// out of the Betaflight source; see docs/sitl_interface.md.
struct Net {
  bool sitl_verified = false;
  std::string sitl_host;
  int sitl_fdm_port = 0;
  int sitl_motor_port = 0;
  int sitl_rc_port = 0;
  int sitl_configurator_tcp = 0;
  std::string viewer_host;
  int viewer_pose_port = 0;
};

struct Sim {
  ParamD physics_rate;  ///< Hz
  ParamD viewer_rate;   ///< Hz
  Net net;
};

/// Betaflight's ARM mode range, mirrored from the `aux` line for box 0 in the
/// real FC's diff. A range is active when start <= us < end (rc_modes.c:87-95).
struct ArmSwitch {
  int aux = 1;                   ///< 1-based: AUX1 is RC channel index 4
  int range_start_us = 0;
  int range_end_us = 0;
  uint16_t armed_us = 0;         ///< a value inside the range
  uint16_t disarmed_us = 0;      ///< a value outside it

  /// Betaflight's test, including its clamp of the channel to [900, 2099].
  bool isArmed(int us) const {
    const int v = std::clamp(us, 900, 2099);
    return v >= range_start_us && v < range_end_us;
  }
};

struct Firmware {
  std::optional<std::string> betaflight_version;
  std::optional<std::string> betaflight_git_tag;
  std::string target;
  std::string diff_all;
  std::string dump_all;
  ArmSwitch arm_switch;
};

struct Meta {
  std::string name;
  std::string description;
  std::string updated;
  std::string frame_convention;  ///< must be "FRD_NED"
};

struct QuadConfig {
  int schema_version = 0;
  Meta meta;
  Firmware firmware;
  MassProperties mass;
  Motors motors;
  Battery battery;
  Aero aero;
  Imu imu;
  Ground ground;
  Camera camera;
  Sim sim;

  /// Dotted paths of every parameter with `measured: false`, in file order.
  std::vector<std::string> unmeasured;
};

/// Load and validate config/quad.yaml. Throws ConfigError.
QuadConfig loadQuadConfig(const std::string& path);

/// Same, from a YAML string. `origin` only appears in error messages.
QuadConfig parseQuadConfig(const std::string& yaml_text, const std::string& origin = "<string>");

/// Human-readable summary, including the unmeasured-parameter list.
std::string summarise(const QuadConfig& cfg);

}  // namespace fdt
