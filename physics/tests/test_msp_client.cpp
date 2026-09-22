// MSP v1 framing and payload-decode tests.
//
// The framing is deliberately exposed as free functions so it can be tested
// without a socket (msp_client.hpp). Two things here are worth more than the
// usual round-trip coverage:
//
//   1. Every failing decode must tell the caller how many bytes to throw away.
//      When it did not, one bad byte stayed at the head of the client's buffer
//      and every later request re-parsed it and rethrew the same error against
//      the wrong command.
//   2. armingDisableFlags walks a VARIABLE-LENGTH MSP_STATUS payload. The
//      offsets are correct for the pinned firmware and will break silently
//      when the submodule pin moves, so they are pinned to byte-exact
//      fixtures here and the relevant source lines are asserted to still exist.

#include "fdt/msp_client.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <fstream>
#include <initializer_list>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using fdt::msp::Command;
using fdt::msp::MspError;
using fdt::msp::Reply;

std::string readFile(const std::string& path) {
  std::ifstream in(path);
  if (!in.good()) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string betaflightSource(const std::string& rel) {
  return readFile(std::string(FDT_REPO_ROOT) + "/third_party/betaflight/" + rel);
}

std::string squeeze(const std::string& s) {
  std::string out = std::regex_replace(s, std::regex(R"(/\*[\s\S]*?\*/)"), " ");
  out = std::regex_replace(out, std::regex(R"(//[^\n]*)"), " ");
  return std::regex_replace(out, std::regex(R"(\s+)"), " ");
}

/// A well-formed reply frame: '$','M','>', length, command, payload, checksum.
std::vector<uint8_t> replyFrame(uint8_t command, const std::vector<uint8_t>& payload) {
  const uint8_t length = static_cast<uint8_t>(payload.size());
  std::vector<uint8_t> f{'$', 'M', '>', length, command};
  f.insert(f.end(), payload.begin(), payload.end());
  f.push_back(fdt::msp::checksum(length, command, payload.data(), payload.size()));
  return f;
}

void appendU16(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>(x >> 8));
}

void appendU32(std::vector<uint8_t>& v, uint32_t x) {
  for (int i = 0; i < 4; ++i) v.push_back(static_cast<uint8_t>((x >> (8 * i)) & 0xFF));
}

/// An MSP_STATUS payload built to the msp.c:1086-1115 layout:
///   u16 taskDelta | u16 i2cErrors | u16 sensors | u32 flightModeFlags
///   u8 pidProfile | u16 systemLoad | u16 gyroCycleTime | u8 byteCount
///   byteCount bytes of extra mode flags | u8 flagCount | u32 armingDisableFlags
///   u8 configStateFlags
std::vector<uint8_t> statusPayload(uint8_t byte_count, uint32_t arming_flags) {
  std::vector<uint8_t> p;
  appendU16(p, 125);         // taskDelta
  appendU16(p, 0);           // i2cErrors
  appendU16(p, 0b100001);    // sensors: ACC | GYRO
  appendU32(p, 0);           // flightModeFlags, low 32 bits
  p.push_back(0);            // pidProfile
  appendU16(p, 7);           // systemLoad
  appendU16(p, 0);           // gyroCycleTime
  p.push_back(byte_count);   // byteCount
  for (uint8_t i = 0; i < (byte_count & 0x0Fu); ++i) p.push_back(static_cast<uint8_t>(0xA0 + i));
  p.push_back(27);           // flagCount (ARMING_DISABLE_FLAGS_COUNT)
  appendU32(p, arming_flags);
  p.push_back(0);            // configStateFlags
  return p;
}

}  // namespace

// --- encodeRequest ---------------------------------------------------------

TEST(MspFraming, EncodesAnEmptyRequestWithTheXorChecksum) {
  const auto f = fdt::msp::encodeRequest(Command::FcVersion);
  ASSERT_EQ(f.size(), 6u);
  EXPECT_EQ(f[0], '$');
  EXPECT_EQ(f[1], 'M');
  EXPECT_EQ(f[2], '<');
  EXPECT_EQ(f[3], 0);  // length
  EXPECT_EQ(f[4], 3);  // MSP_FC_VERSION
  EXPECT_EQ(f[5], 0 ^ 3);
}

TEST(MspFraming, EncodesAPayloadAndXorsItIntoTheChecksum) {
  const std::vector<uint8_t> payload{0x01, 0x02, 0x7F};
  const auto f = fdt::msp::encodeRequest(Command::Motor, payload);
  ASSERT_EQ(f.size(), payload.size() + 6);
  EXPECT_EQ(f[3], 3);
  EXPECT_EQ(f[4], 104);
  EXPECT_EQ(f.back(), static_cast<uint8_t>(3 ^ 104 ^ 0x01 ^ 0x02 ^ 0x7F));
}

TEST(MspFraming, RejectsAPayloadTooLongForMspV1) {
  EXPECT_THROW(fdt::msp::encodeRequest(Command::Motor, std::vector<uint8_t>(256, 0)), MspError);
  EXPECT_NO_THROW(fdt::msp::encodeRequest(Command::Motor, std::vector<uint8_t>(255, 0)));
}

// --- decodeReply, happy path ----------------------------------------------

TEST(MspDecode, NeedsMoreDataForAShortBufferAndConsumesNothing) {
  Reply out;
  size_t consumed = 123;
  const std::vector<uint8_t> partial{'$', 'M', '>', 2, 102};
  EXPECT_FALSE(fdt::msp::decodeReply(partial, out, consumed));
  EXPECT_EQ(consumed, 0u) << "nothing may be dropped while we are still waiting for the frame";
}

TEST(MspDecode, NeedsMoreDataWhenThePayloadIsIncompleteAndConsumesNothing) {
  Reply out;
  size_t consumed = 123;
  std::vector<uint8_t> f = replyFrame(102, {1, 2, 3, 4});
  f.pop_back();  // drop the checksum
  EXPECT_FALSE(fdt::msp::decodeReply(f, out, consumed));
  EXPECT_EQ(consumed, 0u);
}

TEST(MspDecode, ParsesAFrameAndConsumesExactlyIt) {
  const std::vector<uint8_t> payload{0xDE, 0xAD, 0xBE, 0xEF};
  std::vector<uint8_t> bytes = replyFrame(108, payload);
  const size_t frame_size = bytes.size();
  bytes.push_back(0x99);  // a trailing byte from the next frame

  Reply out;
  size_t consumed = 0;
  ASSERT_TRUE(fdt::msp::decodeReply(bytes, out, consumed));
  EXPECT_EQ(out.command, Command::Attitude);
  EXPECT_EQ(out.payload, payload);
  EXPECT_EQ(consumed, frame_size) << "must not swallow the start of the following frame";
}

TEST(MspDecode, ParsesAZeroLengthFrame) {
  Reply out;
  size_t consumed = 0;
  ASSERT_TRUE(fdt::msp::decodeReply(replyFrame(2, {}), out, consumed));
  EXPECT_EQ(out.command, Command::FcVariant);
  EXPECT_TRUE(out.payload.empty());
  EXPECT_EQ(consumed, 6u);
}

// --- decodeReply, failure paths must allow recovery -----------------------

TEST(MspDecode, ABadPreambleResyncsToTheNextDollar) {
  std::vector<uint8_t> bytes{0x00, 0xFF, 0x12};
  const std::vector<uint8_t> good = replyFrame(3, {4, 5, 6});
  bytes.insert(bytes.end(), good.begin(), good.end());

  Reply out;
  size_t consumed = 0;
  EXPECT_THROW(fdt::msp::decodeReply(bytes, out, consumed), MspError);
  EXPECT_EQ(consumed, 3u) << "drop the garbage and land on the '$' that starts the real frame";

  // Dropping `consumed` must leave a buffer that parses cleanly.
  const std::vector<uint8_t> rest(bytes.begin() + static_cast<long>(consumed), bytes.end());
  size_t consumed2 = 0;
  ASSERT_TRUE(fdt::msp::decodeReply(rest, out, consumed2));
  EXPECT_EQ(out.command, Command::FcVersion);
}

TEST(MspDecode, ABadPreambleWithNoLaterDollarConsumesTheWholeBuffer) {
  const std::vector<uint8_t> bytes{0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
  Reply out;
  size_t consumed = 0;
  EXPECT_THROW(fdt::msp::decodeReply(bytes, out, consumed), MspError);
  EXPECT_EQ(consumed, bytes.size());
}

TEST(MspDecode, AnErrorReplyThrowsAndConsumesTheWholeErrorFrame) {
  // '!' is Betaflight rejecting the command. It carries a real length byte, so
  // the whole frame has to go -- leaving the tail behind would desync the next
  // parse, and leaving all of it behind poisoned the client permanently.
  std::vector<uint8_t> bytes{'$', 'M', '!', 0, 102, 102};
  const std::vector<uint8_t> good = replyFrame(2, {'B', 'T', 'F', 'L'});
  bytes.insert(bytes.end(), good.begin(), good.end());

  Reply out;
  size_t consumed = 0;
  EXPECT_THROW(fdt::msp::decodeReply(bytes, out, consumed), MspError);
  EXPECT_EQ(consumed, 6u);

  const std::vector<uint8_t> rest(bytes.begin() + static_cast<long>(consumed), bytes.end());
  size_t consumed2 = 0;
  ASSERT_TRUE(fdt::msp::decodeReply(rest, out, consumed2));
  EXPECT_EQ(out.command, Command::FcVariant);
}

TEST(MspDecode, AnIncompleteErrorReplyWaitsForTheRestOfTheFrame) {
  const std::vector<uint8_t> bytes{'$', 'M', '!', 4, 102, 0x11};
  Reply out;
  size_t consumed = 0;
  EXPECT_FALSE(fdt::msp::decodeReply(bytes, out, consumed));
  EXPECT_EQ(consumed, 0u);
}

TEST(MspDecode, AnUnknownDirectionByteResyncsRatherThanTrustingTheLength) {
  std::vector<uint8_t> bytes{'$', 'M', '?', 200, 102, 0x00};
  const std::vector<uint8_t> good = replyFrame(3, {1, 2, 3});
  bytes.insert(bytes.end(), good.begin(), good.end());

  Reply out;
  size_t consumed = 0;
  EXPECT_THROW(fdt::msp::decodeReply(bytes, out, consumed), MspError);
  EXPECT_EQ(consumed, 6u) << "the length byte is untrustworthy, so scan to the next '$'";

  const std::vector<uint8_t> rest(bytes.begin() + static_cast<long>(consumed), bytes.end());
  size_t consumed2 = 0;
  ASSERT_TRUE(fdt::msp::decodeReply(rest, out, consumed2));
  EXPECT_EQ(out.command, Command::FcVersion);
}

TEST(MspDecode, AChecksumMismatchThrowsAndDropsThatFrameOnly) {
  std::vector<uint8_t> bytes = replyFrame(102, {1, 2, 3, 4});
  const size_t frame_size = bytes.size();
  bytes.back() ^= 0xFF;  // corrupt the checksum
  const std::vector<uint8_t> good = replyFrame(3, {7, 8, 9});
  bytes.insert(bytes.end(), good.begin(), good.end());

  Reply out;
  size_t consumed = 0;
  EXPECT_THROW(fdt::msp::decodeReply(bytes, out, consumed), MspError);
  EXPECT_EQ(consumed, frame_size);

  const std::vector<uint8_t> rest(bytes.begin() + static_cast<long>(consumed), bytes.end());
  size_t consumed2 = 0;
  ASSERT_TRUE(fdt::msp::decodeReply(rest, out, consumed2));
  EXPECT_EQ(out.command, Command::FcVersion);
}

TEST(MspDecode, EveryThrowingPathMakesProgress) {
  // The contract the client depends on: a decode that throws always asks for
  // at least one byte to be dropped. If any path returned 0 here the client
  // would spin on the same bytes forever.
  const std::vector<std::vector<uint8_t>> bad{
      {0x00, 0x01, 0x02, 0x03, 0x04, 0x05},        // bad preamble
      {'$', 'X', '>', 0, 102, 102},                // bad second byte
      {'$', 'M', '?', 0, 102, 102},                // bad direction byte
      {'$', 'M', '!', 0, 102, 102},                // error reply
      {'$', 'M', '>', 0, 102, 0x00},               // checksum mismatch
  };
  for (const auto& bytes : bad) {
    Reply out;
    size_t consumed = 0;
    EXPECT_THROW(fdt::msp::decodeReply(bytes, out, consumed), MspError);
    EXPECT_GE(consumed, 1u) << "a throwing decode must always make progress";
    EXPECT_LE(consumed, bytes.size());
  }
}

// --- MSP_STATUS arming flags ----------------------------------------------

TEST(MspStatus, ReadsTheArmingFlagsWithNoExtraModeBytes) {
  const uint32_t flags = fdt::msp::kArmingDisabledCalibrating | (1u << 3);
  EXPECT_EQ(fdt::msp::armingDisableFlagsFromStatus(statusPayload(0, flags)), flags);
}

TEST(MspStatus, SkipsTheExtraModeFlagBytesBeforeReadingTheArmingFlags) {
  const uint32_t flags = 0xDEADBEEFu;
  for (uint8_t extra = 0; extra <= 15; ++extra) {
    EXPECT_EQ(fdt::msp::armingDisableFlagsFromStatus(statusPayload(extra, flags)), flags)
        << "byteCount = " << static_cast<int>(extra);
  }
}

TEST(MspStatus, MasksTheReservedUpperNibbleOfTheByteCountHeader) {
  // msp.c:1104-1105: only the lowest 4 bits are the byte count; the rest of
  // the header is reserved for future extension. Reading the whole byte would
  // walk the offset off the end of the payload if it is ever used.
  const uint32_t flags = 0x0000CAFEu;
  std::vector<uint8_t> p = statusPayload(2, flags);
  p[15] = static_cast<uint8_t>(0xB0 | 2);  // reserved bits set, count still 2
  EXPECT_EQ(fdt::msp::armingDisableFlagsFromStatus(p), flags);
}

TEST(MspStatus, IsCalibratingBitIsBitTwelve) {
  EXPECT_EQ(fdt::msp::kArmingDisabledCalibrating, 1u << 12);
  EXPECT_NE(fdt::msp::armingDisableFlagsFromStatus(
                statusPayload(0, fdt::msp::kArmingDisabledCalibrating)) &
                fdt::msp::kArmingDisabledCalibrating,
            0u);
  EXPECT_EQ(fdt::msp::armingDisableFlagsFromStatus(statusPayload(0, ~(1u << 12))) &
                fdt::msp::kArmingDisabledCalibrating,
            0u);
}

TEST(MspStatus, RejectsAPayloadTooShortToHoldTheByteCount) {
  for (size_t size = 0; size <= 16; ++size) {
    EXPECT_THROW(fdt::msp::armingDisableFlagsFromStatus(std::vector<uint8_t>(size, 0)), MspError)
        << "size = " << size;
  }
}

TEST(MspStatus, RejectsAPayloadTruncatedPartWayThroughTheWalk) {
  const std::vector<uint8_t> full = statusPayload(4, 0x12345678u);
  // Every truncation between the byteCount byte and the last arming-flag byte
  // must be rejected rather than reading past the end.
  for (size_t size = 17; size < full.size() - 1; ++size) {
    const std::vector<uint8_t> truncated(full.begin(), full.begin() + static_cast<long>(size));
    EXPECT_THROW(fdt::msp::armingDisableFlagsFromStatus(truncated), MspError) << "size = " << size;
  }
}

// --- the claim the probe's calibration wait rests on ----------------------

TEST(BetaflightSource, TheVirtualGyroIsCalibrationCompleteFromBoot) {
  // The probe used to wait on ARMING_DISABLED_CALIBRATING believing it gated
  // the gyro, and reported a frozen-zero gyro when it did not clear. It does
  // not gate the gyro at all: calibration cycles are forced to zero for
  // GYRO_VIRTUAL, so performGyroCalibration is never reached. If this
  // short-circuit disappears upstream, the reasoning in
  // docs/sitl_interface.md 5a and in sitl_probe.cpp has to be revisited.
  const std::string gyro = squeeze(betaflightSource("src/main/sensors/gyro.c"));
  ASSERT_FALSE(gyro.empty()) << "submodule not checked out?";
  EXPECT_NE(gyro.find("gyroSensor->gyroDev.gyroHardware == GYRO_VIRTUAL"), std::string::npos);
  EXPECT_NE(gyro.find("gyroSensor->calibration.cyclesRemaining = 0;"), std::string::npos);

  const std::string target = squeeze(betaflightSource("src/main/target/SITL/target.h"));
  ASSERT_FALSE(target.empty());
  EXPECT_NE(target.find("#define USE_VIRTUAL_GYRO"), std::string::npos);
}

TEST(BetaflightSource, ArmingDisabledCalibratingIsTheBaroOnAFreshBoot) {
  // Why the probe's wait is documented as a baro wait. ARMING_DISABLED_CALIBRATING
  // ORs four sensors, all four are compiled in for SITL, and yet only the baro
  // starts calibrating at boot -- so that flag is the baro and nothing else.
  const std::string core = squeeze(betaflightSource("src/main/fc/core.c"));
  ASSERT_FALSE(core.empty());
  EXPECT_NE(core.find("sensors(SENSOR_ACC) && !accIsCalibrationComplete()"), std::string::npos);
  EXPECT_NE(core.find("sensors(SENSOR_BARO) && !baroIsCalibrated()"), std::string::npos);

  const std::string target = squeeze(betaflightSource("src/main/target/SITL/target.h"));
  EXPECT_NE(target.find("#define USE_BARO"), std::string::npos);
  EXPECT_NE(target.find("#define USE_ACC"), std::string::npos);

  // The boot-time calls: baro unconditional, acc only for MIXER_GIMBAL.
  const std::string init = squeeze(betaflightSource("src/main/fc/init.c"));
  ASSERT_FALSE(init.empty());
  EXPECT_NE(init.find("baroStartCalibration();"), std::string::npos);
  EXPECT_NE(init.find("mixerConfig()->mixerMode == MIXER_GIMBAL) { accStartCalibration();"),
            std::string::npos)
      << "if acc calibration ever starts unconditionally at boot, the probe's wait and "
         "docs/sitl_interface.md 5a both need revisiting -- and a 0 g state during "
         "calibration would bias the Z accel trim by a full 1 g";
}

// --- client-level recovery against a stub flight controller ---------------
//
// The framing tests above pin decodeReply's contract. These pin that MspClient
// actually honours it: a rejected frame must be dropped from the client's own
// buffer, or the next request re-parses it and rethrows. That was the bug --
// one '!' reply or one desynced byte and every later request failed with a
// stale error attributed to the wrong command.

namespace {

/// A TCP listener on an ephemeral loopback port that dumps a scripted byte
/// stream at whoever connects, then holds the connection open.
class FakeFlightController {
 public:
  explicit FakeFlightController(std::vector<uint8_t> script) : script_(std::move(script)) {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    EXPECT_GE(listen_fd_, 0);
    int one = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    EXPECT_EQ(::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
    EXPECT_EQ(::listen(listen_fd_, 1), 0);

    socklen_t len = sizeof(addr);
    EXPECT_EQ(::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len), 0);
    port_ = ntohs(addr.sin_port);

    worker_ = std::thread([this] { run(); });
  }

  ~FakeFlightController() {
    stop_ = true;
    if (worker_.joinable()) worker_.join();
    if (listen_fd_ >= 0) ::close(listen_fd_);
  }

  FakeFlightController(const FakeFlightController&) = delete;
  FakeFlightController& operator=(const FakeFlightController&) = delete;

  uint16_t port() const { return port_; }

 private:
  static bool readable(int fd, int ms) {
    timeval tv{};
    tv.tv_usec = ms * 1000;
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    return ::select(fd + 1, &fds, nullptr, nullptr, &tv) > 0;
  }

  void run() {
    int conn = -1;
    while (!stop_ && conn < 0) {
      if (readable(listen_fd_, 25)) conn = ::accept(listen_fd_, nullptr, nullptr);
    }
    if (conn < 0) return;

    size_t sent = 0;
    while (sent < script_.size()) {
      const ssize_t n = ::send(conn, script_.data() + sent, script_.size() - sent, 0);
      if (n <= 0) break;
      sent += static_cast<size_t>(n);
    }

    // Stay connected and swallow requests, so the client sees a silent FC
    // rather than a closed socket.
    while (!stop_) {
      if (!readable(conn, 25)) continue;
      uint8_t sink[256];
      if (::recv(conn, sink, sizeof(sink), 0) <= 0) break;
    }
    ::close(conn);
  }

  std::vector<uint8_t> script_;
  int listen_fd_ = -1;
  uint16_t port_ = 0;
  std::atomic<bool> stop_{false};
  std::thread worker_;
};

std::vector<uint8_t> concat(std::initializer_list<std::vector<uint8_t>> parts) {
  std::vector<uint8_t> out;
  for (const auto& part : parts) out.insert(out.end(), part.begin(), part.end());
  return out;
}

const std::vector<uint8_t> kBtflVariant = replyFrame(2, {'B', 'T', 'F', 'L'});

}  // namespace

TEST(MspClientOverTcp, ReadsAReplyFromTheStub) {
  FakeFlightController fc(kBtflVariant);
  fdt::msp::MspClient client("127.0.0.1", fc.port(), std::chrono::milliseconds(1000));
  EXPECT_EQ(client.fcVariant(), "BTFL");
}

TEST(MspClientOverTcp, SkipsAStaleReplyToAnEarlierRequest) {
  FakeFlightController fc(concat({replyFrame(3, {4, 5, 6}), kBtflVariant}));
  fdt::msp::MspClient client("127.0.0.1", fc.port(), std::chrono::milliseconds(1000));
  EXPECT_EQ(client.fcVariant(), "BTFL") << "an out-of-order frame must be discarded, not returned";
}

TEST(MspClientOverTcp, RecoversAfterGarbageBytes) {
  FakeFlightController fc(concat({{0x00, 0xFF, 0x7E}, kBtflVariant}));
  fdt::msp::MspClient client("127.0.0.1", fc.port(), std::chrono::milliseconds(1000));

  EXPECT_THROW(client.fcVariant(), MspError);
  // The whole point: the client must have dropped the garbage, so the frame
  // behind it is now readable. This used to throw "bad preamble" forever.
  EXPECT_EQ(client.fcVariant(), "BTFL");
}

TEST(MspClientOverTcp, RecoversAfterAnErrorReply) {
  const std::vector<uint8_t> rejected{'$', 'M', '!', 0, 102, 102};
  FakeFlightController fc(concat({rejected, kBtflVariant}));
  fdt::msp::MspClient client("127.0.0.1", fc.port(), std::chrono::milliseconds(1000));

  EXPECT_THROW(client.fcVariant(), MspError);
  EXPECT_EQ(client.fcVariant(), "BTFL")
      << "a rejected command must not make every later command fail too";
}

TEST(MspClientOverTcp, RecoversAfterACorruptChecksum) {
  std::vector<uint8_t> corrupt = replyFrame(2, {'x', 'y'});
  corrupt.back() ^= 0xFF;
  FakeFlightController fc(concat({corrupt, kBtflVariant}));
  fdt::msp::MspClient client("127.0.0.1", fc.port(), std::chrono::milliseconds(1000));

  EXPECT_THROW(client.fcVariant(), MspError);
  EXPECT_EQ(client.fcVariant(), "BTFL");
}

TEST(MspClientOverTcp, ASilentFlightControllerTimesOutNearTheConfiguredTimeout) {
  // Bounded, not exact: recv is now driven off the remaining deadline instead
  // of its own SO_RCVTIMEO, so a request cannot run to a multiple of the
  // timeout. A generous ceiling keeps this from flaking on a loaded machine
  // while still catching a return to per-recv timeouts.
  constexpr auto kTimeout = std::chrono::milliseconds(300);
  FakeFlightController fc({});
  fdt::msp::MspClient client("127.0.0.1", fc.port(), kTimeout);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(client.fcVariant(), MspError);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_GE(elapsed, kTimeout * 9 / 10);
  EXPECT_LT(elapsed, kTimeout * 3);
}
