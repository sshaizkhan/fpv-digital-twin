// fdt_sitl_probe - close the physics<->SITL loop and MEASURE the conventions
// that could not be read out of the source.
//
// Requires a running SITL:  ./tools/run_sitl.sh
//
//   fdt_sitl_probe [--host IP] [--seconds N] [--quiet]
//
// What it does, in order:
//   1. MSP handshake, so we know which firmware is answering.
//   2. Streams state + RC and checks motor packets come back -- the loop.
//   3. Sends a known gyro rate on ONE fdm axis at a time and reads back
//      MSP_RAW_IMU to see which Betaflight axis moved, and with which sign.
//   4. Same for the accelerometer.
//   5. Sends a known attitude quaternion and reads back MSP_ATTITUDE.
//
// It CONCLUDES NOTHING on its own: it prints a table of what it measured, to
// be turned into a conversion plus a test. See docs/sitl_interface.md
// section 5.

#include "fdt/axis_hit.hpp"
#include "fdt/msp_client.hpp"
#include "fdt/sitl_bridge.hpp"
#include "fdt/sitl_link.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;
using fdt::probe::interpret;
using fdt::sitl::FdmPacket;
using fdt::sitl::RcPacket;
using fdt::sitl::ServoPacket;

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kG = 9.80665;

FdmPacket baseState(double t) {
  FdmPacket p{};
  p.timestamp = t;
  p.imu_orientation_quat[0] = 1.0;  // w, identity
  p.pressure = 101325.0;
  return p;
}

/// A level, stationary quad sitting at the origin.
///
/// An accelerometer at rest reads 1 g, so this sends it. The sign is not an
/// open question: SITL negates all three accel axes (sitl.c:136-138, VERIFIED
/// in docs/sitl_interface.md 4.4), so -kG on fdm Z arrives as about +256 counts
/// on Betaflight Z. Sending a physically impossible 0 g instead used to make
/// the "acc at rest" readback indistinguishable from no accelerometer data
/// arriving at all -- which destroyed the one cheap liveness signal available
/// at exactly the point where the gyro is suspected of being dead.
FdmPacket levelState(double t) {
  FdmPacket p = baseState(t);
  p.imu_linear_acceleration_xyz[2] = -kG;
  return p;
}

/// Level, but in free fall: a zero accel baseline.
///
/// Only the accelerometer sweep uses this, and only because it needs exactly
/// one accel axis excited per step. Against `levelState` the resting 1 g on Z
/// would light up a second axis on every step and trip the multi-axis guard.
FdmPacket zeroAccelState(double t) { return baseState(t); }

/// Mid-sticks, throttle low, AUX all low.
RcPacket neutralRc(double t) {
  RcPacket p{};
  p.timestamp = t;
  for (size_t i = 0; i < fdt::sitl::kMaxRcChannels; ++i) p.channels[i] = 1500;
  p.channels[2] = 1000;  // throttle, AETR order per sitl.c:249
  for (size_t i = 4; i < fdt::sitl::kMaxRcChannels; ++i) p.channels[i] = 1000;
  return p;
}

/// Betaflight's tasks run on REAL wall-clock time here -- SIMULATOR_GYROPID_SYNC
/// is commented out (target.h:50-53), so SITL free-runs its scheduler. Flooding
/// packets faster than real time therefore does NOT deliver more sensor samples;
/// the later packets simply overwrite the buffer before the gyro task reads it
/// (virtualGyroRead clears dataReady on each successful read,
/// accgyro_virtual.c:67-82). Every stream below is paced in real time for that
/// reason.
void stream(fdt::sitl::SitlLink& link, const FdmPacket& base, double& clock,
            std::chrono::milliseconds duration) {
  const auto until = std::chrono::steady_clock::now() + duration;
  int since_drain = 0;
  while (std::chrono::steady_clock::now() < until) {
    FdmPacket p = base;
    clock += 0.001;
    p.timestamp = clock;
    link.sendState(p);
    link.sendRc(neutralRc(clock));
    std::this_thread::sleep_for(1ms);
    if (++since_drain >= 64) {
      link.drainMotors();
      since_drain = 0;
    }
  }
  link.drainMotors();
}

struct SweepResult {
  std::array<int16_t, 3> gyro{};
  std::array<int16_t, 3> acc{};
};

SweepResult holdAndRead(fdt::sitl::SitlLink& link, fdt::msp::MspClient& msp, const FdmPacket& base,
                        double& clock) {
  // Long enough for Betaflight's own filters to settle on the new value.
  stream(link, base, clock, 400ms);
  const fdt::msp::RawImu imu = msp.rawImu();
  return {imu.gyro, imu.acc};
}

/// Stream a level, stationary state until ARMING_DISABLED_CALIBRATING clears.
///
/// Note what this does and does NOT wait for. `isCalibrating` (fc/core.c:183-195)
/// ORs the gyro, ACC, BARO and MAG states and SITL compiles all four in
/// (target.h:72-82), but on a fresh boot only the BARO is ever calibrating:
///
///   - GYRO: `gyroSetCalibrationCycles` forces `cyclesRemaining = 0` for
///     GYRO_VIRTUAL (gyro.c:174-182) and `isGyroSensorCalibrationComplete` is
///     just `cyclesRemaining == 0`, so the virtual gyro is complete from boot
///     and `performGyroCalibration` is never reached at all.
///   - ACC: `init.c:821-824` calls `accStartCalibration` at boot only when the
///     mixer is MIXER_GIMBAL. `calibratingA` therefore stays 0 on a quad.
///   - MAG: only started by a stick command or MSP_MAG_CALIBRATION.
///   - BARO: `baroStartCalibration` runs unconditionally (`init.c:827-829`).
///
/// So this is a baro wait on our `pressure` field, and a failure here says
/// nothing whatsoever about the gyro.
bool waitForSensorCalibration(fdt::sitl::SitlLink& link, fdt::msp::MspClient& msp, double& clock,
                              std::chrono::seconds limit) {
  const auto deadline = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < deadline) {
    stream(link, levelState(clock), clock, 250ms);
    if (!msp.isCalibrating()) return true;
  }
  return false;
}

/// Parse a positive number of seconds, or report why not.
bool parseSeconds(const std::string& text, double& out, std::string& error) {
  try {
    size_t end = 0;
    const double value = std::stod(text, &end);
    if (end != text.size()) {
      error = "not a number: " + text;
      return false;
    }
    if (!std::isfinite(value) || value <= 0.0) {
      error = "--seconds must be a positive, finite number of seconds, got: " + text;
      return false;
    }
    out = value;
    return true;
  } catch (const std::exception&) {
    error = "not a number: " + text;
    return false;
  }
}

std::string motorOrderLine() {
  // Print this from the mapping the code actually uses, both sides 0-based, so
  // the line cannot drift from `betaflightMotorForPacketSlot`. It previously
  // read "0->2, 1->3, 2->4, 3->1", mixing 0-based slots with 1-based motor
  // numbers, which reads as a flat contradiction of the table against the one
  // convention most likely to be silently wrong.
  std::string line = "(packet slot -> BF motor index, both 0-based: ";
  for (size_t slot = 0; slot < 4; ++slot) {
    if (slot > 0) line += ", ";
    line += std::to_string(slot) + "->" + std::to_string(fdt::sitl::betaflightMotorForPacketSlot(slot));
  }
  return line + ", per sitl.c:592-595)";
}

}  // namespace

int main(int argc, char** argv) {
  std::string host = "127.0.0.1";
  double seconds = 2.0;
  bool quiet = false;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--host" && i + 1 < argc) {
      host = argv[++i];
    } else if (a == "--seconds" && i + 1 < argc) {
      // Parsed here, with validation, rather than letting std::stod throw from
      // outside the try below -- that escaped main() and aborted via
      // std::terminate. A non-positive value was worse than a crash: the loop
      // below never ran, motor_packets stayed 0, and the probe blamed the
      // container's networking for a bad command line.
      std::string error;
      if (!parseSeconds(argv[++i], seconds, error)) {
        std::cerr << error << "\n";
        return 2;
      }
    } else if (a == "--quiet") {
      quiet = true;
    } else if (a == "-h" || a == "--help") {
      std::cout << "usage: fdt_sitl_probe [--host IP] [--seconds N] [--quiet]\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << a << "\n";
      return 2;
    }
  }

  try {
    // --- 1. who is answering ---------------------------------------------
    fdt::msp::MspClient msp(host, fdt::sitl::kPortConfiguratorTcp);
    std::cout << "MSP   : " << msp.fcVariant() << " " << msp.fcVersion() << " on " << host << ":"
              << fdt::sitl::kPortConfiguratorTcp << "\n";

    fdt::sitl::SitlLink::Endpoints endpoints;
    endpoints.sitl_host = host;
    fdt::sitl::SitlLink link(endpoints);
    std::cout << "UDP   : state->" << endpoints.state_port << " rc->" << endpoints.rc_port
              << " motors<-" << link.boundMotorPort() << "\n\n";

    // --- 2. close the loop -------------------------------------------------
    std::cout << "--- closing the loop ---\n";
    double clock = 0.0;
    size_t motor_packets = 0;
    ServoPacket motors{};
    const auto loop_until = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(static_cast<long>(seconds * 1000));
    while (std::chrono::steady_clock::now() < loop_until) {
      clock += 0.001;
      link.sendState(levelState(clock));
      link.sendRc(neutralRc(clock));
      if (link.receiveMotors(motors, 2000us)) ++motor_packets;
      std::this_thread::sleep_for(1ms);
    }

    std::cout << "motor packets received : " << motor_packets << "\n";
    std::cout << "malformed              : " << link.malformedMotorPackets() << "\n";
    if (motor_packets == 0) {
      std::cerr << "\nFAIL: SITL sent no motor packets.\n"
                   "  SITL sends to the address given as argv[1], which inside Docker must be\n"
                   "  the host, not 127.0.0.1. Check the container's startup line.\n";
      return 1;
    }
    std::cout << "last motor_speed       : [" << motors.motor_speed[0] << ", " << motors.motor_speed[1]
              << ", " << motors.motor_speed[2] << ", " << motors.motor_speed[3] << "]\n";
    std::cout << motorOrderLine() << "\n";
    if (motor_packets < 10) {
      std::cout << "NOTE: barely any motor packets. Expected while motor_pwm_protocol is\n"
                   "      unset -- SITL defaults it to DISABLED because USE_DSHOT is not\n"
                   "      defined for SITL (common_pre.h:52-54), so the motor device never\n"
                   "      completes an update. Set it to PWM to close the loop properly.\n";
    }
    std::cout << "\n";

    if (quiet) return 0;

    // --- sensor calibration warm-up ---------------------------------------
    std::cout << "--- waiting for baro calibration (streaming a level, still state) ---\n";
    const auto cal_start = std::chrono::steady_clock::now();
    if (!waitForSensorCalibration(link, msp, clock, 60s)) {
      std::cerr << "FAIL: Betaflight is still reporting ARMING_DISABLED_CALIBRATING.\n"
                   "  On a fresh boot that flag is the BARO -- NOT the gyro, which is\n"
                   "  calibration-complete from boot under GYRO_VIRTUAL (gyro.c:174-182).\n"
                   "  So this is a baro problem: check the `pressure` field in the state\n"
                   "  packet. It says nothing about the gyro.\n";
      return 1;
    }
    const auto cal_secs =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - cal_start).count();
    std::cout << "calibrated after " << std::fixed << std::setprecision(1) << cal_secs << " s\n";
    {
      const fdt::msp::RawImu imu = msp.rawImu();
      std::cout << "gyro at rest: [" << imu.gyro[0] << ", " << imu.gyro[1] << ", " << imu.gyro[2]
                << "]   acc at rest: [" << imu.acc[0] << ", " << imu.acc[1] << ", " << imu.acc[2]
                << "]\n";
      // Sending -1 g on fdm Z, negated by SITL, should land as about +256 on
      // Betaflight Z. If it does, IMU packets are reaching the sensors and a
      // flat gyro is specific to the gyro path rather than to the transport.
      if (std::abs(static_cast<int>(imu.acc[2])) < 100) {
        std::cout << "WARNING: acc Z is near zero although the state packet carries -1 g.\n"
                     "         IMU packets may not be reaching Betaflight at all -- fix that\n"
                     "         before reading anything into the sweeps below.\n";
      }
      std::cout << "\n";
    }

    // --- 3. gyro axis sweep ------------------------------------------------
    // Send 1 rad/s on one fdm axis at a time. GYRO_SCALE * RAD2DEG = 16.4 *
    // 57.3 = 939.7 counts per rad/s, so a clean hit reads about +/-940.
    std::cout << "--- gyro: 1.0 rad/s on one fdm axis at a time ---\n";
    std::cout << "expect |940| counts (16.4 LSB/deg/s * 57.3 deg/rad)\n";
    std::cout << std::setw(12) << "fdm axis" << std::setw(26) << "betaflight gyro[X,Y,Z]"
              << "   interpretation\n";

    for (int axis = 0; axis < 3; ++axis) {
      // Settle at rest first so a residual from the previous step cannot be
      // read as a second responding axis.
      holdAndRead(link, msp, levelState(clock), clock);

      FdmPacket p = levelState(clock);
      p.imu_angular_velocity_rpy[axis] = 1.0;
      const SweepResult r = holdAndRead(link, msp, p, clock);

      std::cout << std::setw(12) << (std::string("rpy[") + std::to_string(axis) + "]")
                << std::setw(8) << r.gyro[0] << std::setw(8) << r.gyro[1] << std::setw(8) << r.gyro[2]
                << "   " << interpret(axis, r.gyro, 300) << "\n";
    }

    // --- 4. accelerometer sweep -------------------------------------------
    // ACC_SCALE = 256/9.80665, so 9.80665 m/s^2 should read about 256 counts.
    // Baselined on free fall, not on levelState, so exactly one axis moves.
    std::cout << "\n--- accel: 9.80665 m/s^2 on one fdm axis at a time (free-fall baseline) ---\n";
    std::cout << "expect |256| counts (1 g = 256)\n";
    std::cout << std::setw(12) << "fdm axis" << std::setw(26) << "betaflight acc[X,Y,Z]"
              << "   interpretation\n";

    for (int axis = 0; axis < 3; ++axis) {
      holdAndRead(link, msp, zeroAccelState(clock), clock);

      FdmPacket p = zeroAccelState(clock);
      p.imu_linear_acceleration_xyz[axis] = kG;
      const SweepResult r = holdAndRead(link, msp, p, clock);

      std::cout << std::setw(12) << (std::string("xyz[") + std::to_string(axis) + "]")
                << std::setw(8) << r.acc[0] << std::setw(8) << r.acc[1] << std::setw(8) << r.acc[2]
                << "   " << interpret(axis, r.acc, 100) << "\n";
    }

    // --- 5. attitude quaternion -------------------------------------------
    std::cout << "\n--- attitude: known quaternions vs MSP_ATTITUDE ---\n";
    std::cout << std::setw(34) << "sent" << std::setw(12) << "roll" << std::setw(9) << "pitch"
              << std::setw(9) << "yaw\n";

    struct AttCase {
      const char* label;
      double angle_rad;
      int axis;  // 0 = x, 1 = y, 2 = z
    };
    const AttCase cases[] = {
        {"identity (level)", 0.0, 0},
        {"+30 deg about fdm x", 30.0 / kRadToDeg, 0},
        {"+30 deg about fdm y", 30.0 / kRadToDeg, 1},
        {"+30 deg about fdm z", 30.0 / kRadToDeg, 2},
    };

    // USE_IMU_CALC is undefined for SITL (target.h:48), so the attitude comes
    // straight from imuSetAttitudeQuat and the accel field does not feed into
    // it. levelState's resting 1 g is therefore harmless here even though a
    // genuinely tilted body would carry it on different axes.
    for (const auto& c : cases) {
      FdmPacket p = levelState(clock);
      const double half = c.angle_rad / 2.0;
      p.imu_orientation_quat[0] = std::cos(half);
      p.imu_orientation_quat[1] = 0.0;
      p.imu_orientation_quat[2] = 0.0;
      p.imu_orientation_quat[3] = 0.0;
      p.imu_orientation_quat[static_cast<size_t>(c.axis) + 1] = std::sin(half);

      holdAndRead(link, msp, p, clock);
      const fdt::msp::Attitude a = msp.attitude();
      std::cout << std::setw(34) << c.label << std::fixed << std::setprecision(1) << std::setw(12)
                << a.roll_deg << std::setw(9) << a.pitch_deg << std::setw(9) << a.yaw_deg << "\n";
    }

    // --- 6. the bridge, end to end ----------------------------------------
    // Everything above measures raw packet behaviour. This checks that
    // fdt::sitl's conversions actually deliver a correct world to Betaflight.
    std::cout << "\n--- bridge round-trip: does Betaflight see what we mean? ---\n";

    int failures = 0;
    auto check = [&](const char* what, double got, double want, double tol) {
      const bool ok = std::abs(got - want) <= tol;
      std::cout << (ok ? "  OK   " : "  FAIL ") << std::left << std::setw(42) << what << std::right
                << std::fixed << std::setprecision(1) << std::setw(9) << got << "  want " << want << "\n";
      if (!ok) ++failures;
    };

    // Hold a physical state through the real conversions long enough for
    // Betaflight's filters to settle.
    auto hold = [&](const fdt::State& s) {
      const Eigen::Vector3d specific_force = fdt::specificForceBody(s, Eigen::Vector3d::Zero());
      const auto until = std::chrono::steady_clock::now() + 600ms;
      while (std::chrono::steady_clock::now() < until) {
        clock += 0.001;
        link.sendState(fdt::sitl::toFdmPacket(s, specific_force, clock));
        link.sendRc(fdt::sitl::toRcPacket(fdt::sitl::RcChannels::neutral(), clock));
        std::this_thread::sleep_for(1ms);
        link.drainMotors();
      }
    };

    // Our NED/FRD attitude from ZYX Euler angles in degrees.
    auto euler = [](double roll, double pitch, double yaw) {
      return Eigen::Quaterniond(Eigen::AngleAxisd(yaw / kRadToDeg, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(pitch / kRadToDeg, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(roll / kRadToDeg, Eigen::Vector3d::UnitX()));
    };

    // What "correct" means is Betaflight's own convention, read from the
    // sign of the setpoint each stick produces (rc.c:691-709, pid.c:387-395):
    // FLU body, +gyro Y is nose DOWN, +gyro Z is yaw LEFT, attitude pitch is
    // nose-DOWN positive, yaw is a compass heading.
    const double counts_per_rad = 16.4 * kRadToDeg;
    {
      // Nose up 30 degrees, pitching further up at 3 rad/s, right roll 2 rad/s,
      // yawing right at 1 rad/s.
      fdt::State s;
      s.orientation = euler(0.0, 30.0, 0.0);
      s.angular_velocity = Eigen::Vector3d(2.0, 3.0, 1.0);
      hold(s);

      const fdt::msp::Attitude att = msp.attitude();
      const fdt::msp::RawImu imu = msp.rawImu();
      check("attitude pitch, nose up 30 deg", att.pitch_deg, -30.0, 2.0);
      check("attitude roll, level", att.roll_deg, 0.0, 2.0);
      check("gyro X, right roll 2 rad/s", imu.gyro[0], 2.0 * counts_per_rad, 60.0);
      check("gyro Y, nose-up pitch 3 rad/s", imu.gyro[1], -3.0 * counts_per_rad, 60.0);
      check("gyro Z, yaw right 1 rad/s", imu.gyro[2], -1.0 * counts_per_rad, 60.0);
    }
    {
      // Combined attitude: a single-axis case cannot tell a frame change from
      // a per-component sign hack.
      fdt::State s;
      s.orientation = euler(25.0, 15.0, 250.0);
      hold(s);
      const fdt::msp::Attitude att = msp.attitude();
      check("combined: roll right 25", att.roll_deg, 25.0, 2.0);
      check("combined: nose up 15", att.pitch_deg, -15.0, 2.0);
      check("combined: heading 250", att.yaw_deg, 250.0, 2.0);
    }
    {
      // Level and at rest: a real FC reads +1 g on Z and nothing else.
      fdt::State s;
      hold(s);
      const fdt::msp::RawImu imu = msp.rawImu();
      check("acc Z at rest (1 g = 256)", imu.acc[2], 256.0, 12.0);
      check("acc X at rest", imu.acc[0], 0.0, 12.0);
      check("gyro at rest, X", imu.gyro[0], 0.0, 30.0);
    }
    {
      // At rest, nose down 20 and right wing down 20: gravity's reaction leans
      // toward the tail (-X) and the raised left wing (+Y) in Betaflight's FLU.
      fdt::State s;
      s.orientation = euler(20.0, -20.0, 0.0);
      hold(s);
      const Eigen::Vector3d f = fdt::specificForceBody(s, Eigen::Vector3d::Zero());
      const fdt::msp::RawImu imu = msp.rawImu();
      const double counts_per_ms2 = 256.0 / kG;
      check("acc X, nose down 20 at rest", imu.acc[0], f.x() * counts_per_ms2, 12.0);
      check("acc Y, right wing down 20 at rest", imu.acc[1], -f.y() * counts_per_ms2, 12.0);
      check("acc X is negative nose down", imu.acc[0] < 0.0 ? 1.0 : 0.0, 1.0, 0.0);
      check("acc Y is positive right wing down", imu.acc[1] > 0.0 ? 1.0 : 0.0, 1.0, 0.0);
    }

    if (failures == 0) {
      std::cout << "\nBridge verified: Betaflight sees the world we intend.\n";
      return 0;
    }
    std::cout << "\n" << failures << " bridge check(s) FAILED -- a sign or axis is wrong.\n";
    return 1;
  } catch (const fdt::msp::MspError& e) {
    std::cerr << "MSP error: " << e.what() << "\n";
    return 1;
  } catch (const fdt::sitl::SitlError& e) {
    std::cerr << "SITL link error: " << e.what() << "\n";
    return 1;
  }
}
