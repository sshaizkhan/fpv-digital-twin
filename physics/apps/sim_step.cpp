// fdt_sim - run the physics core headless, with no Betaflight and no viewer.
//
//   fdt_sim [--config PATH] [--profile NAME] [--duration S] [--rate HZ]
//           [--out FILE.csv] [--seed N]
//
// --rate defaults to sim.physics_rate from the config, which is the single
// source of truth for the step size; pass --rate only to override it.
//
// Profiles:
//   hover     trimmed hover, open loop
//   althold   trimmed hover with a proportional altitude hold
//   freefall  motors off from 100 m
//   takeoff   full throttle from the ground
//   rollstep  hover, then a roll step at t = 1 s
//
// Prints a summary and the real-time ratio; with --out, writes a CSV trace.

#include "fdt/multirotor.hpp"
#include "fdt/pose_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <optional>
#include <string>

namespace {

std::array<double, 4> uniform(double c) { return {c, c, c, c}; }

/// Mean motor command, as a stand-in for "throttle" in the viewer HUD.
double throttleOf(const fdt::Multirotor& m) {
  const auto& rpm = m.telemetry().motor_rpm;
  const double max_rpm = m.config().motors.model.max_rpm_safety.value;
  double sum = 0.0;
  for (double r : rpm) sum += r;
  return max_rpm > 0.0 ? std::clamp(sum / (4.0 * max_rpm), 0.0, 1.0) : 0.0;
}

void writeHeader(std::ostream& o) {
  o << "t,x,y,z,alt,vx,vy,vz,qw,qx,qy,qz,p,q,r,"
       "gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,"
       "rpm_fl,rpm_fr,rpm_rl,rpm_rr,thrust,vbat,amps,soc,contact\n";
}

void writeRow(std::ostream& o, const fdt::Multirotor& m) {
  const auto& s = m.state();
  const auto& t = m.telemetry();
  o << t.time << ',' << s.position.x() << ',' << s.position.y() << ',' << s.position.z() << ','
    << s.altitude() << ',' << s.velocity.x() << ',' << s.velocity.y() << ',' << s.velocity.z() << ','
    << s.orientation.w() << ',' << s.orientation.x() << ',' << s.orientation.y() << ','
    << s.orientation.z() << ',' << s.angular_velocity.x() << ',' << s.angular_velocity.y() << ','
    << s.angular_velocity.z() << ',' << t.imu.gyro.x() << ',' << t.imu.gyro.y() << ',' << t.imu.gyro.z()
    << ',' << t.imu.accel.x() << ',' << t.imu.accel.y() << ',' << t.imu.accel.z() << ','
    << t.motor_rpm[0] << ',' << t.motor_rpm[1] << ',' << t.motor_rpm[2] << ',' << t.motor_rpm[3] << ','
    << t.total_thrust << ',' << t.battery_voltage << ',' << t.battery_current << ',' << t.battery_soc
    << ',' << (t.in_contact ? 1 : 0) << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = "config/quad.yaml";
  std::string profile = "hover";
  std::string out_path;
  double duration = 5.0;
  // Unset means "take it from sim.physics_rate", which cannot be read until the
  // config is loaded -- below, after the argument loop.
  std::optional<double> rate_override;
  uint64_t seed = 0;
  bool viewer = false;
  bool realtime = false;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + what);
      return argv[++i];
    };
    try {
      if (a == "--config") config_path = next("--config");
      else if (a == "--profile") profile = next("--profile");
      else if (a == "--out") out_path = next("--out");
      else if (a == "--duration") duration = std::stod(next("--duration"));
      else if (a == "--rate") rate_override = std::stod(next("--rate"));
      else if (a == "--seed") seed = std::stoull(next("--seed"));
      else if (a == "--viewer") { viewer = true; realtime = true; }
      else if (a == "--no-realtime") realtime = false;
      else if (a == "-h" || a == "--help") {
        std::cout << "usage: fdt_sim [--config PATH] [--profile hover|althold|freefall|takeoff|rollstep]"
                     " [--duration S] [--rate HZ] [--out FILE.csv] [--seed N]\n"
                     "  --rate defaults to sim.physics_rate from the config\n";
        return 0;
      } else {
        std::cerr << "unknown argument: " << a << "\n";
        return 2;
      }
    } catch (const std::exception& e) {
      std::cerr << e.what() << "\n";
      return 2;
    }
  }

  try {
    const fdt::QuadConfig cfg = fdt::loadQuadConfig(config_path);

    // The config owns the physics rate; --rate is an override, not the default.
    const double rate = rate_override.value_or(cfg.sim.physics_rate.value);
    if (!std::isfinite(rate) || rate <= 0.0) {
      std::cerr << "--rate must be a positive number of hertz, got " << rate << "\n";
      return 2;
    }
    if (!std::isfinite(duration) || duration <= 0.0) {
      std::cerr << "--duration must be a positive number of seconds, got " << duration << "\n";
      return 2;
    }

    fdt::Multirotor m(cfg, seed);

    // Optional live feed for the browser viewer. With --viewer the loop is
    // also paced to real time, because a run that finishes 1500x faster than
    // real time is over before anything can be watched.
    std::unique_ptr<fdt::PosePublisher> pose;
    if (viewer) {
      pose = std::make_unique<fdt::PosePublisher>(
          cfg.sim.net.viewer_host, static_cast<uint16_t>(cfg.sim.net.viewer_pose_port));
      std::cout << "viewer: pose -> " << cfg.sim.net.viewer_host << ":"
                << cfg.sim.net.viewer_pose_port << " at " << cfg.sim.viewer_rate.value
                << " Hz (real time pacing)\n";
    }

    if (profile == "takeoff") {
      m.placeOnGround();
    } else if (profile == "freefall") {
      fdt::State s;
      s.position.z() = -100.0;
      m.reset(s);
    } else {
      fdt::State s;
      s.position.z() = -10.0;
      m.reset(s);
      m.trimHover();
    }

    std::ofstream out;
    if (!out_path.empty()) {
      out.open(out_path);
      if (!out.good()) {
        std::cerr << "cannot write " << out_path << "\n";
        return 2;
      }
      out << std::setprecision(9);
      writeHeader(out);
    }

    const double dt = 1.0 / rate;
    const long steps = std::lround(duration / dt);
    if (steps < 1) {
      std::cerr << "--duration " << duration << " s at --rate " << rate << " Hz is less than one step\n";
      return 2;
    }
    const double hover = m.hoverCommand();

    const auto t0 = std::chrono::steady_clock::now();
    for (long i = 0; i < steps; ++i) {
      const double t = static_cast<double>(i) * dt;

      if (profile == "freefall") {
        m.setMotorCommands(uniform(0.0));
      } else if (profile == "takeoff") {
        m.setMotorCommands(uniform(t < 0.5 ? 0.0 : 1.0));
      } else if (profile == "althold") {
        const double error = 10.0 - m.state().altitude();
        const double climb = -m.state().velocity.z();
        m.setMotorCommands(uniform(std::clamp(hover + 0.05 * error - 0.03 * climb, 0.0, 1.0)));
      } else if (profile == "rollstep") {
        const double d = (t >= 1.0 && t < 1.2) ? 0.08 : 0.0;
        m.setMotorCommands({hover + d, hover - d, hover + d, hover - d});
      } else {
        m.setMotorCommands(uniform(hover));
      }

      m.step(dt);

      if (pose) {
        pose->publishThrottled(m, m.telemetry().total_thrust > 0.0, throttleOf(m), cfg.sim.viewer_rate.value);
        if (realtime) {
          const auto target = t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                       std::chrono::duration<double>(m.time()));
          std::this_thread::sleep_until(target);
        }
      }
      if (out.is_open()) writeRow(out, m);
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double wall = std::chrono::duration<double>(t1 - t0).count();

    const auto& s = m.state();
    const auto& tel = m.telemetry();
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "profile      : " << profile << "\n"
              << "steps        : " << steps << " at " << rate << " Hz (" << duration << " s)\n"
              << "wall clock   : " << wall << " s  ->  " << std::setprecision(1) << (duration / wall)
              << "x real time, " << std::setprecision(2) << (wall / static_cast<double>(steps)) * 1e6
              << " us/step\n"
              << std::setprecision(3)
              << "hover cmd    : " << hover * 100.0 << " %\n"
              << "final alt    : " << s.altitude() << " m\n"
              << "final vel    : [" << s.velocity.transpose() << "] m/s\n"
              << "final rates  : [" << s.angular_velocity.transpose() << "] rad/s\n"
              << "battery      : " << tel.battery_voltage << " V, " << tel.battery_current << " A, soc "
              << tel.battery_soc * 100.0 << " %\n"
              << "energy       : " << fdt::mechanicalEnergy(s, m.inertia()) << " J\n"
              << "on ground    : " << (tel.in_contact ? "yes" : "no") << "\n";
    if (out.is_open()) std::cout << "trace        : " << out_path << "\n";
    return 0;
  } catch (const fdt::ConfigError& e) {
    std::cerr << "config error: " << e.what() << "\n";
    return 2;
  }
}
