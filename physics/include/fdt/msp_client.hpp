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
bool decodeReply(const std::vector<uint8_t>& bytes, Reply& out, size_t& consumed);

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

  /// ARMING_DISABLED_CALIBRATING. Betaflight freezes gyro.gyroADC at zero
  /// until calibration finishes (gyro.c:417), so a gyro readback taken before
  /// this clears is meaningless.
  bool isCalibrating();

 private:
  int fd_ = -1;
  std::chrono::milliseconds timeout_;
  std::vector<uint8_t> buffer_;
};

}  // namespace fdt::msp
