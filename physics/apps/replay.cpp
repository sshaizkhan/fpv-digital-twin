// fdt_replay - drive the physics model from a recorded motor trace and write
// out the gyro it produces, for comparison against the real thing.
//
//   fdt_replay --config config/quad.yaml --in motors.csv --out sim.csv
//
// Input CSV:  t,m0,m1,m2,m3   (seconds, and four throttles in [0,1] in OUR
//                              physical motor order: FL, FR, RL, RR)
// Output CSV: t,gyro_x,gyro_y,gyro_z,... in SI units, body FRD.
//
// WHY MOTOR COMMANDS RATHER THAN STICKS. Replaying sticks needs Betaflight in
// the loop, and SITL is pinned to wall-clock time, so a 60 s log costs 60 s and
// is not reproducible. Feeding the motor outputs the FC actually produced
// removes the controller from the comparison entirely: what is left is our
// plant model against the real quad's response to the same inputs. If the gyro
// traces disagree, the physics is wrong -- there is nowhere else for the error
// to hide. That is the comparison the Phase 4 fitter needs.
//
// The closed-loop stick replay, with Betaflight deciding the motor outputs, is
// a separate question and needs SITL (see fdt_sitl_hover).

#include "fdt/multirotor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Sample {
  double t = 0.0;
  std::array<double, 4> motor{};
};

std::vector<Sample> readMotorCsv(const std::string& path, std::string& error) {
  std::vector<Sample> out;
  std::ifstream in(path);
  if (!in.good()) {
    error = "cannot open " + path;
    return out;
  }

  std::string line;
  size_t line_no = 0;
  while (std::getline(in, line)) {
    ++line_no;
    if (line.empty()) continue;

    std::istringstream ss(line);
    std::string field;
    std::vector<double> values;
    bool parse_failed = false;
    while (std::getline(ss, field, ',')) {
      try {
        values.push_back(std::stod(field));
      } catch (const std::exception&) {
        parse_failed = true;
        break;
      }
    }
    if (parse_failed) {
      // A header is simply a first line that is not numbers. Detecting it by
      // pattern ("no digits") failed on the obvious header `t,m0,m1,m2,m3`,
      // which does contain digits.
      if (line_no == 1) continue;
      error = path + ":" + std::to_string(line_no) + ": '" + field + "' is not a number";
      return {};
    }
    if (values.size() < 5) {
      error = path + ":" + std::to_string(line_no) + ": expected t,m0,m1,m2,m3";
      return {};
    }

    Sample s;
    s.t = values[0];
    for (size_t i = 0; i < 4; ++i) s.motor[i] = std::clamp(values[i + 1], 0.0, 1.0);
    out.push_back(s);
  }
  if (out.size() < 2) error = path + ": need at least two samples";
  return out;
}

/// Zero-order hold: the motor output the FC produced holds until it produces
/// the next one, which is what actually happened on the real quad.
size_t advanceIndex(const std::vector<Sample>& samples, size_t index, double t) {
  while (index + 1 < samples.size() && samples[index + 1].t <= t) ++index;
  return index;
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = "config/quad.yaml";
  std::string in_path;
  std::string out_path;
  double rate_override = 0.0;
  bool start_airborne = true;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + what);
      return argv[++i];
    };
    try {
      if (a == "--config") config_path = next("--config");
      else if (a == "--in") in_path = next("--in");
      else if (a == "--out") out_path = next("--out");
      else if (a == "--rate") rate_override = std::stod(next("--rate"));
      else if (a == "--from-ground") start_airborne = false;
      else if (a == "-h" || a == "--help") {
        std::cout << "usage: fdt_replay --in motors.csv --out sim.csv [--config PATH]\n"
                     "                  [--rate HZ] [--from-ground]\n";
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
  if (in_path.empty() || out_path.empty()) {
    std::cerr << "--in and --out are both required\n";
    return 2;
  }

  try {
    const fdt::QuadConfig config = fdt::loadQuadConfig(config_path);

    std::string error;
    const std::vector<Sample> samples = readMotorCsv(in_path, error);
    if (!error.empty()) {
      std::cerr << error << "\n";
      return 2;
    }

    const double rate = rate_override > 0.0 ? rate_override : config.sim.physics_rate.value;
    if (!(rate > 0.0)) {
      std::cerr << "physics rate must be positive\n";
      return 2;
    }
    const double dt = 1.0 / rate;

    fdt::Multirotor quad(config);
    if (start_airborne) {
      // Replaying a mid-flight segment: start clear of the ground so contact
      // forces do not contaminate the comparison.
      fdt::State s;
      s.position.z() = -100.0;
      quad.reset(s);
    } else {
      quad.placeOnGround();
    }

    std::ofstream out(out_path);
    if (!out.good()) {
      std::cerr << "cannot write " << out_path << "\n";
      return 2;
    }
    out << std::setprecision(9);
    out << "t,gyro_x,gyro_y,gyro_z,alt,vx,vy,vz,m0,m1,m2,m3\n";

    const double t_start = samples.front().t;
    const double t_end = samples.back().t;
    const long steps = std::lround((t_end - t_start) / dt);

    const auto wall_start = std::chrono::steady_clock::now();
    size_t index = 0;
    for (long step = 0; step <= steps; ++step) {
      const double t = t_start + static_cast<double>(step) * dt;
      index = advanceIndex(samples, index, t);
      quad.setMotorCommands(samples[index].motor);
      quad.step(dt);

      const auto& s = quad.state();
      out << t << ',' << s.angular_velocity.x() << ',' << s.angular_velocity.y() << ','
          << s.angular_velocity.z() << ',' << s.altitude() << ',' << s.velocity.x() << ','
          << s.velocity.y() << ',' << s.velocity.z();
      for (size_t m = 0; m < 4; ++m) out << ',' << samples[index].motor[m];
      out << '\n';
    }
    const double wall =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();

    const double sim_seconds = t_end - t_start;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "replayed " << sim_seconds << " s from " << samples.size() << " motor samples at "
              << std::setprecision(0) << rate << " Hz\n";
    std::cout << std::setprecision(3) << "wall clock " << wall << " s -> " << std::setprecision(0)
              << (wall > 0 ? sim_seconds / wall : 0.0) << "x real time\n";
    std::cout << "wrote " << out_path << "\n";
    return 0;
  } catch (const fdt::ConfigError& e) {
    std::cerr << "config error: " << e.what() << "\n";
    return 2;
  }
}
