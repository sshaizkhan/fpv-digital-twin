// Minimal MSP v1 client over TCP -- the same link the Configurator uses
// (SITL exposes UART1 on TCP 5761; serial_tcp.c:41,113).
//
// Its immediate job is calibration: MSP_RAW_IMU reports the gyro and
// accelerometer values Betaflight ACTUALLY RECEIVED, which is the most direct
// way to settle the axis and sign questions in docs/sitl_interface.md section
// 5 -- send a known value on one axis, see where it lands.
//
// It is also the foundation for the MSP/autonomy work the project wants later.
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace fdt::msp {

class MspError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

enum class Command : uint8_t {
  ApiVersion = 1,
  FcVariant = 2,
  FcVersion = 3,
  BoardInfo = 4,
  Status = 101,
  RawImu = 102,
  Motor = 104,
  Rc = 105,
  Attitude = 108,
};

// --- framing, exposed as free functions so it is testable without a socket --

/// MSP v1 request: '$','M','<', length, command, payload, XOR checksum.
std::vector<uint8_t> encodeRequest(Command command, const std::vector<uint8_t>& payload = {});

/// XOR of length, command and payload -- the MSP v1 checksum.
uint8_t checksum(uint8_t length, uint8_t command, const uint8_t* payload, size_t size);

struct Reply {
  Command command{};
  std::vector<uint8_t> payload;
};

/// Parse one reply from `bytes`. Returns false if the buffer does not yet hold
/// a complete frame; throws MspError if it holds a malformed or error one
/// ('$','M','!' is Betaflight rejecting the command).
///
/// `consumed` is always set to the number of leading bytes the caller should
/// discard -- on success the frame just parsed, and on a throw the bytes being
/// rejected (a whole bad frame, or everything up to the next '$' when the
/// header itself is garbage). It is only left at zero when the function
/// returns false, i.e. when more data is needed and nothing may be dropped.
/// A caller that ignores it on the throwing paths will re-parse the same bad
/// bytes on every subsequent call and never recover.
bool decodeReply(const std::vector<uint8_t>& bytes, Reply& out, size_t& consumed);

/// ARMING_DISABLED_CALIBRATING (runtime_config.h:55).
constexpr uint32_t kArmingDisabledCalibrating = 1u << 12;

/// Pull the arming-disable bitfield out of an MSP_STATUS payload. Split out
/// from MspClient so the variable-length offset walk can be tested against
/// byte-exact fixtures without a socket -- it is keyed to the payload layout
/// of one firmware version and will break silently when the submodule pin
/// moves.
uint32_t armingDisableFlagsFromStatus(const std::vector<uint8_t>& payload);

// --- typed views -----------------------------------------------------------

/// MSP_RAW_IMU: the raw sensor values as Betaflight sees them, post-alignment.
struct RawImu {
  std::array<int16_t, 3> acc{};   ///< 1 g = 256 counts under SITL (ACC_SCALE)
  std::array<int16_t, 3> gyro{};  ///< 16.4 counts per deg/s (GYRO_SCALE)
  std::array<int16_t, 3> mag{};
};

/// MSP_ATTITUDE, converted to degrees.
struct Attitude {
  double roll_deg = 0.0;   ///< wire is decidegrees
  double pitch_deg = 0.0;  ///< wire is decidegrees
  double yaw_deg = 0.0;    ///< wire is whole degrees
};

class MspClient {
 public:
  MspClient(const std::string& host, uint16_t port,
            std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));
  ~MspClient();

  MspClient(const MspClient&) = delete;
  MspClient& operator=(const MspClient&) = delete;

  std::vector<uint8_t> request(Command command, const std::vector<uint8_t>& payload = {});

  std::string fcVariant();
  std::string fcVersion();
  RawImu rawImu();
  Attitude attitude();
  std::array<uint16_t, 8> motors();

  /// MSP_STATUS arming-disable bitfield (runtime_config.h).
  uint32_t armingDisableFlags();

  /// ARMING_DISABLED_CALIBRATING. `isCalibrating` (fc/core.c:183-195) ORs the
  /// gyro, ACC, BARO and MAG states, but on a freshly booted SITL only the
  /// BARO is ever actually calibrating:
  ///   - gyro: `gyroSetCalibrationCycles` forces `cyclesRemaining = 0` for
  ///     GYRO_VIRTUAL (gyro.c:174-182), so it is complete from boot;
  ///   - acc: `accStartCalibration` runs at boot only for MIXER_GIMBAL
  ///     (init.c:821-824), which a quad is not;
  ///   - mag: only on a stick command or MSP_MAG_CALIBRATION.
  /// So this waits on the baro, and it tells you nothing about the gyro.
  bool isCalibrating();

 private:
  /// Drop `consumed` leading bytes, clamped to the buffer size.
  void dropFromBuffer(size_t consumed);

  int fd_ = -1;
  std::chrono::milliseconds timeout_;
  std::vector<uint8_t> buffer_;
};

}  // namespace fdt::msp
