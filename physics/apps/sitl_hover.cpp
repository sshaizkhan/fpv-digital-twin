// fdt_sitl_hover - Phase 2's final acceptance test: arm the real Betaflight
// firmware and hold a hover with scripted RC.
//
//   ./tools/run_sitl.sh &
//   ./tools/load_config_sitl.py --restart-container fdt-sitl
//   ./build/make/physics/fdt_sitl_hover
//
// The loop is paced to REAL TIME on purpose. Betaflight free-runs its
// scheduler on the wall clock under SITL (SIMULATOR_GYROPID_SYNC is off,
// target.h:50-53), so the physics cannot outrun it: state packets arriving
// faster than real time are simply overwritten before the gyro task reads
// them. Deterministic faster-than-real-time replay stays a headless-physics
// feature (Phase 4); it is not available with SITL in the loop.
//
// Modes come from the real FC's own aux config:
//   AUX1 = ARM    (arms LOW on this quad -- see firmware.arm_switch)
//   AUX2 = ANGLE  (BOXANGLE = 1, rc_modes.h:31-33), so Betaflight self-levels.
//   --acro leaves ANGLE off if you want to see the bare rate loop.

#include "fdt/msp_client.hpp"
#include "fdt/multirotor.hpp"
#include "fdt/pose_publisher.hpp"
#include "fdt/sitl_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

struct Schedule {
  double settle_s = 3.0;   ///< disarmed, switch held OFF: Betaflight refuses to
                           ///< arm until it has seen the switch off once, and
                           ///< the baro needs this long to finish calibrating
  double arm_s = 3.0;      ///< ARM switch on
  double ramp_end_s = 5.0; ///< throttle ramped in by here, then hold altitude
};

/// Tilt from level, degrees.
double tiltDeg(const fdt::State& state) {
  const Eigen::Vector3d up_body = state.orientation * Eigen::Vector3d(0, 0, -1);
  return std::acos(std::clamp(-up_body.z(), -1.0, 1.0)) * kRadToDeg;
}

const char* armingFlagName(int bit) {
  static const char* names[] = {
      "NO_GYRO", "FAILSAFE", "RX_FAILSAFE", "NOT_DISARMED", "BOXFAILSAFE", "RUNAWAY_TAKEOFF",
      "CRASH_DETECTED", "THROTTLE", "ANGLE", "BOOT_GRACE_TIME", "NOPREARM", "LOAD",
      "CALIBRATING", "CLI", "CMS_MENU", "BST", "MSP", "PARALYZE", "GPS", "RESC",
      "DSHOT_TELEM", "REBOOT_REQUIRED", "DSHOT_BITBANG", "ACC_CALIBRATION",
      "MOTOR_PROTOCOL", "ARM_SWITCH"};
  return (bit >= 0 && bit < 26) ? names[bit] : "?";
}

std::string describeArmingFlags(uint32_t flags) {
  if (flags == 0) return "none";
  std::string out;
  for (int bit = 0; bit < 26; ++bit) {
    if (flags & (1u << bit)) {
      if (!out.empty()) out += " ";
      out += armingFlagName(bit);
    }
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = "config/quad.yaml";
  std::string host = "127.0.0.1";
  std::string out_path;
  double duration = 15.0;
  double target_alt = 2.0;
  double rate = 1000.0;
  bool acro = false;
  bool publish_pose = true;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + what);
      return argv[++i];
    };
    try {
      if (a == "--config") config_path = next("--config");
      else if (a == "--host") host = next("--host");
      else if (a == "--out") out_path = next("--out");
      else if (a == "--duration") duration = std::stod(next("--duration"));
      else if (a == "--target-alt") target_alt = std::stod(next("--target-alt"));
      else if (a == "--rate") rate = std::stod(next("--rate"));
      else if (a == "--acro") acro = true;
      else if (a == "--no-viewer") publish_pose = false;
      else if (a == "-h" || a == "--help") {
        std::cout << "usage: fdt_sitl_hover [--config PATH] [--host IP] [--duration S]\n"
                     "                      [--target-alt M] [--rate HZ] [--out CSV] [--acro]\n"
                     "                      [--no-viewer]\n";
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
  if (!(duration > 0.0) || !(rate > 0.0) || !(target_alt > 0.0)) {
    std::cerr << "--duration, --rate and --target-alt must all be positive\n";
    return 2;
  }

  try {
    const fdt::QuadConfig config = fdt::loadQuadConfig(config_path);
    const fdt::ArmSwitch& arm = config.firmware.arm_switch;

    // Each tick must advance the physics by exactly 1/rate: Betaflight runs on
    // the wall clock, so any mismatch silently speeds up or slows down the
    // model relative to the controller. That needs a whole number of substeps.
    const double physics_rate = config.sim.physics_rate.value;
    const double ratio = physics_rate / rate;
    const int substeps = static_cast<int>(std::lround(ratio));
    if (substeps < 1 || std::abs(ratio - substeps) > 1e-9 * ratio) {
      std::cerr << "--rate " << rate << " must divide sim.physics_rate (" << physics_rate
                << " Hz) into a whole number of physics steps\n";
      return 2;
    }
    const double dt_phys = 1.0 / physics_rate;
    const auto tick = std::chrono::duration<double>(1.0 / rate);

    fdt::msp::MspClient msp(host, fdt::sitl::kPortConfiguratorTcp);
    std::cout << "SITL     : " << msp.fcVariant() << " " << msp.fcVersion() << "\n";
    std::cout << "arming   : blocked by [" << describeArmingFlags(msp.armingDisableFlags()) << "]\n";
    std::cout << "arm sw   : AUX" << arm.aux << "  armed " << arm.armed_us << "us  disarmed "
              << arm.disarmed_us << "us  (range " << arm.range_start_us << "-" << arm.range_end_us
              << ")\n";
    std::cout << "mode     : " << (acro ? "ACRO (rate only)" : "ANGLE (self-levelling)") << "\n";

    fdt::Multirotor quad(config);
    fdt::sitl::SitlLink::Endpoints endpoints;
    endpoints.sitl_host = host;
    fdt::sitl::SitlBridge bridge(config, endpoints);
    if (!bridge.motorOrderVerified()) {
      std::cout << "WARNING  : motors.betaflight_order is UNVERIFIED. A stable hover here does\n"
                   "           NOT confirm the mapping matches your quad -- a symmetric hover\n"
                   "           looks identical under several wrong orderings.\n";
    }
    std::cout << "\n";

    // Pose feed for the viewer. Fire and forget on UDP: if nothing is
    // listening the sends simply go nowhere, which is the normal case.
    std::unique_ptr<fdt::PosePublisher> pose;
    if (publish_pose) {
      pose = std::make_unique<fdt::PosePublisher>(config.sim.net.viewer_host,
                                                  static_cast<uint16_t>(config.sim.net.viewer_pose_port));
      std::cout << "viewer   : pose -> " << config.sim.net.viewer_host << ":"
                << config.sim.net.viewer_pose_port << " at "
                << config.sim.viewer_rate.value << " Hz\n";
    }

    quad.placeOnGround();
    const double ground_z = quad.state().position.z();

    // Ramp to the throttle the PHYSICS MODEL says hovers, rather than to a
    // guessed constant. Betaflight maps the stick roughly linearly onto motor
    // output, so hover_cmd of the 1000-2000 range is a good first estimate and
    // the controller trims the rest. Ramping past it is what produced a 1.7 m/s
    // climb and a 1.75 m overshoot on the first run.
    const double hover_cmd = quad.hoverCommand();
    const double hover_us = std::clamp(1000.0 + hover_cmd * 1000.0, 1050.0, 1800.0);
    std::cout << "hover est: motor " << std::fixed << std::setprecision(3) << hover_cmd
              << " -> throttle " << std::setprecision(0) << hover_us << "us\n\n";

    const Schedule schedule;

    std::ofstream csv;
    if (!out_path.empty()) {
      csv.open(out_path);
      csv << std::setprecision(7)
          << "t,alt,vz,tilt_deg,throttle_us,arm_switch,fc_armed,m_fl,m_fr,m_rl,m_rr,motor_packets\n";
    }

    std::array<double, 4> commands{};
    // Betaflight must CONFIRM it is armed before the throttle moves. Ramping on
    // a timer instead raced the arm: the throttle passed min_check (1050) while
    // arming was still blocked, which set ARMING_DISABLED_THROTTLE, and because
    // the switch was already on ARM_SWITCH then latched -- it will not arm
    // again until the switch is cycled. Exactly how a real quad behaves.
    bool armed_confirmed = false;  ///< Betaflight has reported ARMED at least once
    double arm_confirmed_at = -1.0;
    // Keep asking after the arm: a mid-flight disarm (runaway takeoff, crash
    // detection, failsafe) must fail as a disarm, not show up later as a
    // mysterious altitude-hold error.
    bool fc_armed = false;         ///< Betaflight's latest answer
    double disarmed_at = -1.0;
    uint32_t disarm_arming_flags = 0;
    std::array<uint16_t, 6> disarm_rc{};
    constexpr double ramp_seconds = 2.0;
    constexpr double kSettleAfterRampS = 5.0;  ///< climb time excluded from the score
    double last_arm_poll = -1.0;
    size_t motor_packets = 0;
    double integral = 0.0;
    double max_tilt = 0.0;
    double peak_alt = 0.0;
    std::vector<double> hold_errors;
    bool ever_armed = false;

    // Failsafe fires after failsafe_delay (1.5 s on this quad) of RX failure,
    // so any stall in this loop is a candidate cause for a mid-flight disarm.
    double worst_gap_s = 0.0;
    double worst_gap_at = 0.0;
    auto last_iteration = Clock::now();

    const auto start = Clock::now();
    auto next_tick = start;

    while (true) {
      const auto now = Clock::now();
      const double t = std::chrono::duration<double>(now - start).count();
      if (t >= duration) break;
      const double gap = std::chrono::duration<double>(now - last_iteration).count();
      if (gap > worst_gap_s) {
        worst_gap_s = gap;
        worst_gap_at = t;
      }
      last_iteration = now;

      // --- scripted RC ---
      fdt::sitl::RcChannels rc = fdt::sitl::RcChannels::neutral(arm);
      if (!acro) rc.setAux(1, 1500);  // AUX2 mid -> ANGLE
      const bool want_armed = t >= schedule.arm_s;
      if (want_armed) rc.setArmed(arm, true);

      double throttle_us = 1000.0;
      if (armed_confirmed && arm_confirmed_at < 0.0) arm_confirmed_at = t;
      if (armed_confirmed && disarmed_at < 0.0) {
        const double alt = -(quad.state().position.z() - ground_z);
        const double climb = -quad.state().velocity.z();

        const double since_arm = t - arm_confirmed_at;
        if (since_arm < ramp_seconds) {
          // Ease the throttle in rather than stepping it: a step would let the
          // I-term wind up against a quad still sitting on its feet.
          const double f = since_arm / ramp_seconds;
          throttle_us = 1000.0 + f * (hover_us - 1000.0);
          integral = 0.0;
        } else {
          const double error = target_alt - alt;
          integral = std::clamp(integral + error * (1.0 / rate), -6.0, 6.0);
          // Heavily damped on climb rate: overshoot costs far more than a slow
          // approach, because the quad has to come back down through it.
          throttle_us = hover_us + 50.0 * error + 26.0 * integral - 165.0 * climb;

          // Only SCORE the hold once it has had time to get there. Counting the
          // climb itself made "worst error" simply the target altitude, which
          // says nothing about how well it holds.
          if (since_arm > ramp_seconds + kSettleAfterRampS) hold_errors.push_back(std::abs(error));
        }
        throttle_us = std::clamp(throttle_us, 1000.0, 1900.0);
      }
      rc.setAetr(1500, 1500, static_cast<uint16_t>(std::lround(throttle_us)), 1500);
      bridge.sendRc(rc, t);

      // --- state out ---
      const fdt::Wrench w = quad.wrench(quad.state(), 0.0);
      const Eigen::Vector3d specific_force = w.force / quad.inertia().mass;
      bridge.sendState(quad.state(), specific_force, t);

      // --- motors in: take the freshest packet, do not queue up latency ---
      std::array<double, 4> latest{};
      bool got = false;
      while (bridge.receiveMotors(latest, 0us)) {
        got = true;
        ++motor_packets;
      }
      if (got) {
        commands = latest;
        for (double c : commands) {
          if (c > 0.0) ever_armed = true;
        }
      }

      // Ask Betaflight directly, at a low rate so the real-time loop is not
      // disturbed by a blocking MSP round trip.
      if (want_armed && disarmed_at < 0.0 && (t - last_arm_poll) > 0.25) {
        last_arm_poll = t;
        fc_armed = msp.isArmed();
        if (fc_armed && !armed_confirmed) {
          armed_confirmed = true;
          std::cout << "armed at t=" << std::fixed << std::setprecision(2) << t << "s\n";
        } else if (!fc_armed && armed_confirmed) {
          disarmed_at = t;
          // Capture WHY, at the moment it happened. Read after the fact these
          // flags only describe what blocks re-arming, not the cause.
          disarm_arming_flags = msp.armingDisableFlags();
          // What does Betaflight think the sticks are at this instant? If the
          // RC has gone stale or out of the rx_min/max_usec window, failsafe
          // is explained; if it is exactly what we are sending, it is not.
          try {
            const auto rc_payload = msp.request(fdt::msp::Command::Rc);
            for (size_t ch = 0; ch < 6 && (ch * 2 + 1) < rc_payload.size(); ++ch) {
              disarm_rc[ch] = static_cast<uint16_t>(rc_payload[ch * 2] |
                                                    (rc_payload[ch * 2 + 1] << 8));
            }
          } catch (const fdt::msp::MspError&) {
          }
          std::cout << "DISARMED by Betaflight at t=" << std::fixed << std::setprecision(2) << t
                    << "s  blocked by [" << describeArmingFlags(msp.armingDisableFlags()) << "]\n";
        }
      }

      // --- physics ---
      quad.setMotorCommands(commands);
      for (int i = 0; i < substeps; ++i) quad.step(dt_phys);

      if (pose) {
        pose->publishThrottled(quad, fc_armed, throttle_us > 1000.0 ? (throttle_us - 1000.0) / 1000.0 : 0.0,
                               config.sim.viewer_rate.value);
      }

      const double alt = -(quad.state().position.z() - ground_z);
      const double tilt = tiltDeg(quad.state());
      max_tilt = std::max(max_tilt, tilt);
      peak_alt = std::max(peak_alt, alt);

      if (csv.is_open()) {
        csv << t << ',' << alt << ',' << -quad.state().velocity.z() << ',' << tilt << ','
            << throttle_us << ',' << (want_armed ? 1 : 0) << ',' << (fc_armed ? 1 : 0) << ',' << commands[0] << ',' << commands[1]
            << ',' << commands[2] << ',' << commands[3] << ',' << motor_packets << '\n';
      }

      next_tick += std::chrono::duration_cast<Clock::duration>(tick);
      std::this_thread::sleep_until(next_tick);
    }

    // --- report ----------------------------------------------------------
    const auto& s = quad.state();
    const double final_alt = -(s.position.z() - ground_z);
    const uint32_t flags = msp.armingDisableFlags();

    double mean_err = 0.0, worst_err = 0.0;
    if (!hold_errors.empty()) {
      for (double e : hold_errors) {
        mean_err += e;
        worst_err = std::max(worst_err, e);
      }
      mean_err /= static_cast<double>(hold_errors.size());
    }

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "worst loop gap : " << std::setprecision(3) << worst_gap_s << " s at t="
              << worst_gap_at << " s" << std::setprecision(2)
              << (worst_gap_s > 1.5 ? "   <- exceeds failsafe_delay!" : "") << "\n";
    if (pose) std::cout << "pose packets   : " << pose->sent() << "\n";
    std::cout << "motor packets  : " << motor_packets << "\n";
    std::cout << "armed (MSP)    : " << (armed_confirmed ? "yes" : "NO")
              << (armed_confirmed ? "" : "  <- Betaflight never reported ARMED") << "\n";
    if (disarmed_at >= 0.0) {
      std::cout << "disarmed (MSP) : at t=" << disarmed_at << " s\n";
      std::cout << "  arming flags AT the disarm : [" << describeArmingFlags(disarm_arming_flags) << "]\n";
      std::cout << "  betaflight rcData AT disarm: [";
      for (size_t ch = 0; ch < 6; ++ch) std::cout << (ch ? ", " : "") << disarm_rc[ch];
      std::cout << "]  (roll,pitch,yaw,throttle,aux1,aux2)\n";
    }
    std::cout << "motors ran     : " << (ever_armed ? "yes" : "NO") << "\n";
    std::cout << "peak altitude  : " << peak_alt << " m\n";
    std::cout << "final altitude : " << final_alt << " m  (target " << target_alt << ")\n";
    std::cout << "hold error     : mean " << mean_err << " m, worst " << worst_err << " m\n";
    std::cout << "max tilt       : " << max_tilt << " deg\n";
    std::cout << "lateral drift  : " << s.position.head<2>().norm() << " m\n";
    std::cout << "arming flags   : [" << describeArmingFlags(flags) << "]\n";
    if (csv.is_open()) std::cout << "trace          : " << out_path << "\n";

    // --- did it pass? -----------------------------------------------------
    int failures = 0;
    auto require = [&](bool ok, const std::string& what) {
      std::cout << (ok ? "  PASS  " : "  FAIL  ") << what << "\n";
      if (!ok) ++failures;
    };
    std::cout << "\n";
    require(motor_packets > 100, "SITL sent motor packets (the loop is closed)");
    require(armed_confirmed, "Betaflight reported ARMED (MSP flightModeFlags bit 0)");
    require(armed_confirmed && disarmed_at < 0.0, "stayed ARMED to the end (polled over MSP)");
    require(ever_armed, "the motors actually ran");
    require(peak_alt > 0.5, "it left the ground");
    require(!hold_errors.empty(), "the hold window was actually reached");
    require(!hold_errors.empty() && mean_err < 0.35,
            "held the target altitude once settled (mean error < 0.35 m)");
    require(max_tilt < 25.0, "stayed upright (max tilt < 25 deg)");
    require(std::isfinite(final_alt), "the state stayed finite");

    if (failures == 0) {
      std::cout << "\nScripted hover OK: the real Betaflight firmware is flying our model.\n";
      return 0;
    }
    std::cout << "\n" << failures << " check(s) failed.\n";
    return 1;
  } catch (const fdt::msp::MspError& e) {
    std::cerr << "MSP error: " << e.what() << "\n  is ./tools/run_sitl.sh running?\n";
    return 2;
  } catch (const fdt::sitl::SitlError& e) {
    std::cerr << "SITL link error: " << e.what() << "\n";
    return 2;
  } catch (const fdt::ConfigError& e) {
    std::cerr << "config error: " << e.what() << "\n";
    return 2;
  }
}
