#include "fdt/quad_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

namespace fdt {
namespace {

constexpr int kSupportedSchemaVersion = 1;
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

[[noreturn]] void fail(const std::string& origin, const std::string& path, const std::string& what) {
  throw ConfigError(origin + ": " + path + ": " + what);
}

std::string join(const std::string& base, const std::string& key) {
  return base.empty() ? key : base + "." + key;
}

}  // namespace

const char* toString(MotorId id) {
  switch (id) {
    case MotorId::FL: return "FL";
    case MotorId::FR: return "FR";
    case MotorId::RL: return "RL";
    case MotorId::RR: return "RR";
  }
  return "?";
}

MotorId motorIdFromString(const std::string& name) {
  for (const auto id : kAllMotorIds) {
    if (name == toString(id)) return id;
  }
  throw ConfigError("unknown motor name '" + name + "' (expected FL, FR, RL or RR)");
}

const char* toString(Spin s) { return s == Spin::CW ? "CW" : "CCW"; }

MotorId Motors::motorForBetaflightIndex(int bf_index) const {
  if (bf_index < 1 || bf_index > 4) {
    throw ConfigError("betaflight motor index out of range: " + std::to_string(bf_index));
  }
  return bf_order_[static_cast<size_t>(bf_index - 1)];
}

Eigen::Matrix3d MassProperties::inertiaMatrix() const {
  const Eigen::Vector3d& d = inertia_diag.value;
  const Eigen::Vector3d& p = inertia_products.value;  // [Ixy, Ixz, Iyz]
  Eigen::Matrix3d I;
  I << d.x(), -p.x(), -p.y(),
       -p.x(), d.y(), -p.z(),
       -p.y(), -p.z(), d.z();
  return I;
}

double Battery::cellOcv(double soc) const {
  if (ocv_curve.empty()) return 0.0;
  if (soc <= ocv_curve.front().soc) return ocv_curve.front().v;
  if (soc >= ocv_curve.back().soc) return ocv_curve.back().v;
  for (size_t i = 1; i < ocv_curve.size(); ++i) {
    const auto& a = ocv_curve[i - 1];
    const auto& b = ocv_curve[i];
    if (soc <= b.soc) {
      const double span = b.soc - a.soc;
      const double t = (span > 0.0) ? (soc - a.soc) / span : 0.0;
      return a.v + t * (b.v - a.v);
    }
  }
  return ocv_curve.back().v;
}

// ---------------------------------------------------------------------------

/// Walks the YAML once, building the typed config and collecting provenance.
struct QuadConfigParser {
  YAML::Node root;
  std::string origin;
  QuadConfig cfg;

  [[noreturn]] void bad(const std::string& path, const std::string& what) const { fail(origin, path, what); }

  YAML::Node require(const YAML::Node& parent, const std::string& key, const std::string& parent_path) const {
    if (!parent || !parent.IsMap()) bad(parent_path, "expected a mapping");
    const YAML::Node n = parent[key];
    if (!n) bad(join(parent_path, key), "required key is missing");
    return n;
  }

  template <typename T>
  T scalarAs(const YAML::Node& n, const std::string& path) const {
    try {
      return n.as<T>();
    } catch (const YAML::Exception&) {
      bad(path, "value is not of the expected type");
    }
  }

  /// Reads the units/measured/source trio shared by every parameter block and
  /// records the path if the value is not measured.
  template <typename T>
  void fillMeta(Param<T>& p, const YAML::Node& n, const std::string& path) {
    if (!n.IsMap()) bad(path, "expected a parameter mapping with value/units/measured/source");
    if (!n["units"]) bad(path, "missing `units:` (every parameter must state its units)");
    if (!n["measured"]) {
      bad(path,
          "missing `measured:` (every parameter must declare whether it was measured "
          "or estimated)");
    }
    p.units = scalarAs<std::string>(n["units"], join(path, "units"));
    p.measured = scalarAs<bool>(n["measured"], join(path, "measured"));
    p.source = n["source"] ? scalarAs<std::string>(n["source"], join(path, "source")) : std::string{};
    p.path = path;
    if (!p.measured) cfg.unmeasured.push_back(path);
  }

  ParamD num(const YAML::Node& parent, const std::string& key, const std::string& parent_path) {
    const std::string path = join(parent_path, key);
    const YAML::Node n = require(parent, key, parent_path);
    ParamD p;
    fillMeta(p, n, path);
    p.value = scalarAs<double>(require(n, "value", path), join(path, "value"));
    if (!std::isfinite(p.value)) bad(join(path, "value"), "must be finite");
    return p;
  }

  ParamI integer(const YAML::Node& parent, const std::string& key, const std::string& parent_path) {
    const std::string path = join(parent_path, key);
    const YAML::Node n = require(parent, key, parent_path);
    ParamI p;
    fillMeta(p, n, path);
    p.value = scalarAs<int>(require(n, "value", path), join(path, "value"));
    return p;
  }

  ParamV3 vec3(const YAML::Node& parent, const std::string& key, const std::string& parent_path) {
    const std::string path = join(parent_path, key);
    const YAML::Node n = require(parent, key, parent_path);
    ParamV3 p;
    fillMeta(p, n, path);
    const YAML::Node v = require(n, "value", path);
    if (!v.IsSequence() || v.size() != 3) bad(join(path, "value"), "expected a 3-element [x, y, z] sequence");
    for (int i = 0; i < 3; ++i) {
      p.value[i] = scalarAs<double>(v[static_cast<size_t>(i)], join(path, "value"));
      if (!std::isfinite(p.value[i])) bad(join(path, "value"), "must be finite");
    }
    return p;
  }

  /// Degrees in the file, radians in the struct. Converted here and nowhere else.
  ParamD angleDeg(const YAML::Node& parent, const std::string& key, const std::string& parent_path) {
    ParamD p = num(parent, key, parent_path);
    if (p.units != "deg") bad(p.path, "expected `units: deg` for a human-measured angle, got '" + p.units + "'");
    p.value *= kDeg2Rad;
    p.units = "rad";
    return p;
  }

  void requirePositive(const ParamD& p) const {
    if (!(p.value > 0.0)) bad(p.path, "must be > 0, got " + std::to_string(p.value));
  }
  void requireNonNegative(const ParamD& p) const {
    if (!(p.value >= 0.0)) bad(p.path, "must be >= 0, got " + std::to_string(p.value));
  }
  void requireAllPositive(const ParamV3& p) const {
    if (!(p.value.array() > 0.0).all()) bad(p.path, "all three components must be > 0");
  }

  std::string str(const YAML::Node& parent, const std::string& key, const std::string& parent_path) const {
    return scalarAs<std::string>(require(parent, key, parent_path), join(parent_path, key));
  }

  std::optional<std::string> optStr(const YAML::Node& parent, const std::string& key,
                                    const std::string& parent_path) const {
    const YAML::Node n = parent[key];
    if (!n || n.IsNull()) return std::nullopt;
    const auto s = scalarAs<std::string>(n, join(parent_path, key));
    return s.empty() ? std::nullopt : std::optional<std::string>{s};
  }

  // --- sections ---

  void parseMeta() {
    cfg.schema_version = scalarAs<int>(require(root, "schema_version", ""), "schema_version");
    if (cfg.schema_version != kSupportedSchemaVersion) {
      bad("schema_version", "unsupported schema version " + std::to_string(cfg.schema_version) + ", this build "
          "understands " + std::to_string(kSupportedSchemaVersion));
    }

    const YAML::Node m = require(root, "meta", "");
    cfg.meta.name = str(m, "name", "meta");
    cfg.meta.description = m["description"] ? scalarAs<std::string>(m["description"], "meta.description") : "";
    cfg.meta.updated = m["updated"] ? scalarAs<std::string>(m["updated"], "meta.updated") : "";
    cfg.meta.frame_convention = str(m, "frame_convention", "meta");
    if (cfg.meta.frame_convention != "FRD_NED") {
      bad("meta.frame_convention",
          "this build only implements FRD_NED (body x-fwd/y-right/z-down, world NED); "
          "see docs/coordinate_frames.md. Got '" + cfg.meta.frame_convention + "'");
    }
  }

  void parseFirmware() {
    const YAML::Node f = require(root, "firmware", "");
    cfg.firmware.betaflight_version = optStr(f, "betaflight_version", "firmware");
    cfg.firmware.betaflight_git_tag = optStr(f, "betaflight_git_tag", "firmware");
    cfg.firmware.target = str(f, "target", "firmware");
    cfg.firmware.diff_all = str(f, "diff_all", "firmware");
    cfg.firmware.dump_all = str(f, "dump_all", "firmware");
  }

  void parseMass() {
    const YAML::Node m = require(root, "mass", "");
    cfg.mass.auw = num(m, "auw", "mass");
    requirePositive(cfg.mass.auw);
    cfg.mass.cg_offset = vec3(m, "cg_offset", "mass");
    cfg.mass.inertia_diag = vec3(m, "inertia_diag", "mass");
    requireAllPositive(cfg.mass.inertia_diag);
    cfg.mass.inertia_products = vec3(m, "inertia_products", "mass");

    // A rigid body's principal moments must satisfy the triangle inequalities.
    const Eigen::Vector3d& d = cfg.mass.inertia_diag.value;
    if (d.x() + d.y() < d.z() || d.y() + d.z() < d.x() || d.z() + d.x() < d.y()) {
      bad("mass.inertia_diag", "violates the inertia triangle inequality; no rigid body has these moments");
    }
  }

  void parseMotors() {
    const YAML::Node mo = require(root, "motors", "");

    const YAML::Node geom = require(mo, "geometry", "motors");
    for (const auto id : kAllMotorIds) {
      cfg.motors.at(id).position = vec3(geom, toString(id), "motors.geometry");
    }

    const YAML::Node spin = require(mo, "spin", "motors");
    cfg.motors.spin_verified = spin["verified"] && scalarAs<bool>(spin["verified"], "motors.spin.verified");
    int seen = 0, cw = 0;
    for (const auto& kv : spin) {
      const auto key = kv.first.as<std::string>();
      if (key == "verified" || key == "source") continue;
      const MotorId id = motorIdFromString(key);
      const auto s = kv.second.as<std::string>();
      if (s == "CW") {
        cfg.motors.at(id).spin = Spin::CW;
        ++cw;
      } else if (s == "CCW") {
        cfg.motors.at(id).spin = Spin::CCW;
      } else {
        bad(join("motors.spin", key), "expected CW or CCW (as seen from above), got '" + s + "'");
      }
      ++seen;
    }
    if (seen != 4) bad("motors.spin", "expected a direction for all four motors, got " + std::to_string(seen));
    if (cw != 2) {
      bad("motors.spin", "expected exactly two CW and two CCW props, got " + std::to_string(cw) +
          " CW; a quad cannot hold yaw otherwise");
    }
    if (cfg.motors.at(MotorId::FL).spin != cfg.motors.at(MotorId::RR).spin) {
      bad("motors.spin", "diagonally opposite motors (FL/RR) must spin the same way");
    }

    const YAML::Node order = require(mo, "betaflight_order", "motors");
    cfg.motors.order_verified = order["verified"] && scalarAs<bool>(order["verified"], "motors.betaflight_order.verified");
    std::map<int, MotorId> mapping;
    for (const auto& kv : order) {
      const auto key = kv.first.as<std::string>();
      if (key == "verified" || key == "source") continue;
      int idx = 0;
      try {
        idx = std::stoi(key);
      } catch (const std::exception&) {
        bad(join("motors.betaflight_order", key), "expected a motor index 1..4");
      }
      if (idx < 1 || idx > 4) bad(join("motors.betaflight_order", key), "index out of range 1..4");
      mapping[idx] = motorIdFromString(kv.second.as<std::string>());
    }
    if (mapping.size() != 4) {
      bad("motors.betaflight_order", "expected indices 1..4, got " + std::to_string(mapping.size()));
    }
    std::vector<MotorId> assigned;
    for (const auto& [idx, id] : mapping) {
      cfg.motors.bf_order_[static_cast<size_t>(idx - 1)] = id;
      assigned.push_back(id);
    }
    for (const auto id : kAllMotorIds) {
      if (std::count(assigned.begin(), assigned.end(), id) != 1) {
        bad("motors.betaflight_order",
            std::string("must be a one-to-one mapping; motor ") + toString(id) + " appears " +
                std::to_string(std::count(assigned.begin(), assigned.end(), id)) + " times");
      }
    }

    const YAML::Node mdl = require(mo, "model", "motors");
    auto& M = cfg.motors.model;
    M.kv = num(mdl, "kv", "motors.model");
    M.thrust_coeff = num(mdl, "thrust_coeff", "motors.model");
    M.torque_coeff = num(mdl, "torque_coeff", "motors.model");
    M.time_constant = num(mdl, "time_constant", "motors.model");
    M.rotor_inertia = num(mdl, "rotor_inertia", "motors.model");
    M.resistance = num(mdl, "resistance", "motors.model");
    M.max_rpm_safety = num(mdl, "max_rpm_safety", "motors.model");
    requirePositive(M.kv);
    requirePositive(M.thrust_coeff);
    requirePositive(M.torque_coeff);
    requirePositive(M.time_constant);
    requirePositive(M.rotor_inertia);
    requirePositive(M.resistance);
    requirePositive(M.max_rpm_safety);
  }

  void parseBattery() {
    const YAML::Node b = require(root, "battery", "");
    cfg.battery.cells = integer(b, "cells", "battery");
    if (cfg.battery.cells.value < 1) bad("battery.cells", "must be at least 1");
    cfg.battery.capacity = num(b, "capacity", "battery");
    requirePositive(cfg.battery.capacity);
    cfg.battery.internal_resistance = num(b, "internal_resistance", "battery");
    requirePositive(cfg.battery.internal_resistance);
    cfg.battery.initial_soc = num(b, "initial_soc", "battery");
    if (cfg.battery.initial_soc.value < 0.0 || cfg.battery.initial_soc.value > 1.0) {
      bad("battery.initial_soc", "must be in [0, 1]");
    }

    const YAML::Node c = require(b, "ocv_curve", "battery");
    if (!c["measured"]) bad("battery.ocv_curve", "missing `measured:`");
    cfg.battery.ocv_measured = scalarAs<bool>(c["measured"], "battery.ocv_curve.measured");
    cfg.battery.ocv_source = c["source"] ? c["source"].as<std::string>() : "";
    if (!cfg.battery.ocv_measured) cfg.unmeasured.push_back("battery.ocv_curve");

    const YAML::Node pts = require(c, "points", "battery.ocv_curve");
    if (!pts.IsSequence() || pts.size() < 2) bad("battery.ocv_curve.points", "expected at least two points");
    for (const auto& pn : pts) {
      OcvPoint p{require(pn, "soc", "battery.ocv_curve.points").as<double>(),
                 require(pn, "v", "battery.ocv_curve.points").as<double>()};
      if (!cfg.battery.ocv_curve.empty()) {
        const auto& prev = cfg.battery.ocv_curve.back();
        if (p.soc <= prev.soc) bad("battery.ocv_curve.points", "soc must strictly increase");
        if (p.v < prev.v) bad("battery.ocv_curve.points", "open-circuit voltage must not decrease with soc");
      }
      if (p.v <= 0.0) bad("battery.ocv_curve.points", "cell voltage must be > 0");
      cfg.battery.ocv_curve.push_back(p);
    }
  }

  void parseAero() {
    const YAML::Node a = require(root, "aero", "");
    cfg.aero.linear_drag = vec3(a, "linear_drag", "aero");
    cfg.aero.quadratic_drag = vec3(a, "quadratic_drag", "aero");
    cfg.aero.angular_damping = vec3(a, "angular_damping", "aero");
    cfg.aero.induced_drag_k = num(a, "induced_drag_k", "aero");
    cfg.aero.air_density = num(a, "air_density", "aero");
    requirePositive(cfg.aero.air_density);
    requireNonNegative(cfg.aero.induced_drag_k);
    for (const ParamV3* p : {&cfg.aero.linear_drag, &cfg.aero.quadratic_drag, &cfg.aero.angular_damping}) {
      if ((p->value.array() < 0.0).any()) bad(p->path, "drag coefficients must be >= 0 or the sim will diverge");
    }
  }

  void parseImu() {
    const YAML::Node i = require(root, "imu", "");
    cfg.imu.sample_rate = num(i, "sample_rate", "imu");
    requirePositive(cfg.imu.sample_rate);
    cfg.imu.gyro_noise_density = num(i, "gyro_noise_density", "imu");
    cfg.imu.gyro_bias = vec3(i, "gyro_bias", "imu");
    cfg.imu.gyro_bias_walk = num(i, "gyro_bias_walk", "imu");
    cfg.imu.accel_noise_density = num(i, "accel_noise_density", "imu");
    cfg.imu.accel_bias = vec3(i, "accel_bias", "imu");
    cfg.imu.vibration_enabled = num(i, "vibration_enabled", "imu");
    requireNonNegative(cfg.imu.gyro_noise_density);
    requireNonNegative(cfg.imu.accel_noise_density);
  }

  void parseGround() {
    const YAML::Node g = require(root, "ground", "");
    cfg.ground.height = num(g, "height", "ground");
    cfg.ground.contact_stiffness = num(g, "contact_stiffness", "ground");
    cfg.ground.contact_damping = num(g, "contact_damping", "ground");
    cfg.ground.friction = num(g, "friction", "ground");
    cfg.ground.stand_height = num(g, "stand_height", "ground");
    requirePositive(cfg.ground.contact_stiffness);
    requireNonNegative(cfg.ground.contact_damping);
    requireNonNegative(cfg.ground.friction);
    requireNonNegative(cfg.ground.stand_height);
  }

  void parseCamera() {
    const YAML::Node c = require(root, "camera", "");
    cfg.camera.tilt = angleDeg(c, "tilt", "camera");
    cfg.camera.fov_horizontal = angleDeg(c, "fov_horizontal", "camera");
    if (cfg.camera.fov_horizontal.value <= 0.0 || cfg.camera.fov_horizontal.value >= kDeg2Rad * 180.0) {
      bad("camera.fov_horizontal", "must be in (0, 180) degrees");
    }
    cfg.camera.position = vec3(c, "position", "camera");
  }

  void parseSim() {
    const YAML::Node s = require(root, "sim", "");
    cfg.sim.physics_rate = num(s, "physics_rate", "sim");
    cfg.sim.viewer_rate = num(s, "viewer_rate", "sim");
    requirePositive(cfg.sim.physics_rate);
    requirePositive(cfg.sim.viewer_rate);
    if (cfg.sim.physics_rate.value < 1000.0) {
      bad("sim.physics_rate", "the project requires at least 1 kHz physics");
    }

    const YAML::Node n = require(s, "net", "sim");
    auto& net = cfg.sim.net;
    net.sitl_verified = n["sitl_verified"] && n["sitl_verified"].as<bool>();
    net.sitl_host = str(n, "sitl_host", "sim.net");
    net.sitl_fdm_port = scalarAs<int>(require(n, "sitl_fdm_port", "sim.net"), "sim.net.sitl_fdm_port");
    net.sitl_motor_port = scalarAs<int>(require(n, "sitl_motor_port", "sim.net"), "sim.net.sitl_motor_port");
    net.sitl_rc_port = scalarAs<int>(require(n, "sitl_rc_port", "sim.net"), "sim.net.sitl_rc_port");
    net.sitl_configurator_tcp =
        scalarAs<int>(require(n, "sitl_configurator_tcp", "sim.net"), "sim.net.sitl_configurator_tcp");
    net.viewer_host = str(n, "viewer_host", "sim.net");
    net.viewer_pose_port = scalarAs<int>(require(n, "viewer_pose_port", "sim.net"), "sim.net.viewer_pose_port");
    for (const auto& [name, port] : std::initializer_list<std::pair<const char*, int>>{
             {"sitl_fdm_port", net.sitl_fdm_port},
             {"sitl_motor_port", net.sitl_motor_port},
             {"sitl_rc_port", net.sitl_rc_port},
             {"sitl_configurator_tcp", net.sitl_configurator_tcp},
             {"viewer_pose_port", net.viewer_pose_port}}) {
      if (port < 1 || port > 65535) bad(join("sim.net", name), "not a valid port");
    }
  }

  QuadConfig run() {
    parseMeta();
    parseFirmware();
    parseMass();
    parseMotors();
    parseBattery();
    parseAero();
    parseImu();
    parseGround();
    parseCamera();
    parseSim();
    return std::move(cfg);
  }
};

QuadConfig parseQuadConfig(const std::string& yaml_text, const std::string& origin) {
  QuadConfigParser parser;
  parser.origin = origin;
  try {
    parser.root = YAML::Load(yaml_text);
  } catch (const YAML::Exception& e) {
    throw ConfigError(origin + ": not valid YAML: " + e.what());
  }
  if (!parser.root || !parser.root.IsMap()) throw ConfigError(origin + ": expected a top-level mapping");
  return parser.run();
}

QuadConfig loadQuadConfig(const std::string& path) {
  std::ifstream in(path);
  if (!in.good()) throw ConfigError("cannot open config file: " + path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return parseQuadConfig(ss.str(), path);
}

std::string summarise(const QuadConfig& cfg) {
  std::ostringstream o;
  o << std::fixed;
  o << "quad: " << cfg.meta.name << "  (schema v" << cfg.schema_version << ", frames "
    << cfg.meta.frame_convention << ")\n";
  o << "firmware: betaflight "
    << (cfg.firmware.betaflight_version ? *cfg.firmware.betaflight_version : std::string("UNSET")) << " target "
    << cfg.firmware.target << "\n";
  o << std::setprecision(4);
  o << "mass: " << cfg.mass.auw.value << " kg   inertia diag ["
    << cfg.mass.inertia_diag.value.transpose() << "] kg m^2\n";
  o << "motors:\n";
  for (const auto id : kAllMotorIds) {
    const auto& m = cfg.motors.at(id);
    o << "  " << toString(id) << "  pos [" << m.position.value.transpose() << "] m  spin "
      << toString(m.spin) << " (from above)\n";
  }
  o << "  betaflight index -> motor: ";
  for (int i = 1; i <= 4; ++i) o << i << "=" << toString(cfg.motors.motorForBetaflightIndex(i)) << " ";
  o << (cfg.motors.order_verified ? "[verified]" : "[UNVERIFIED]") << "\n";
  o << std::scientific << std::setprecision(3);
  o << "  kT " << cfg.motors.model.thrust_coeff.value << " N/(rad/s)^2   kQ "
    << cfg.motors.model.torque_coeff.value << " Nm/(rad/s)^2";
  o << std::fixed << std::setprecision(4) << "   tau " << cfg.motors.model.time_constant.value << " s\n";
  o << std::setprecision(2);
  o << "battery: " << cfg.battery.cells.value << "S " << cfg.battery.capacity.value << " Ah, "
    << cfg.battery.packOcv(cfg.battery.initial_soc.value) << " V at soc "
    << cfg.battery.initial_soc.value << "\n";
  o << std::setprecision(1);
  o << "camera: tilt " << cfg.camera.tilt.value / kDeg2Rad << " deg, fov "
    << cfg.camera.fov_horizontal.value / kDeg2Rad << " deg\n";
  o << "rates: physics " << cfg.sim.physics_rate.value << " Hz, viewer " << cfg.sim.viewer_rate.value << " Hz\n";

  o << "\n" << cfg.unmeasured.size() << " parameter(s) are NOT measured:\n";
  for (const auto& p : cfg.unmeasured) o << "  - " << p << "\n";
  if (!cfg.motors.spin_verified) o << "  ! motors.spin is unverified\n";
  if (!cfg.motors.order_verified) o << "  ! motors.betaflight_order is unverified\n";
  if (!cfg.sim.net.sitl_verified) o << "  ! sim.net SITL ports are unverified\n";
  if (!cfg.firmware.betaflight_version) o << "  ! firmware.betaflight_version is unset (blocks Phase 2)\n";
  return o.str();
}

}  // namespace fdt
