// Wire-format tests for the Betaflight SITL packets.
//
// The struct layouts are already pinned by static_asserts in the header, so
// these cover what those cannot: that our transcription still matches the
// Betaflight source in the submodule, and that the motor permutation is the
// one SITL actually applies.

#include "fdt/sitl_packets.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace {

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

/// Strip C/C++ comments, then collapse runs of whitespace, so that a reflow or
/// a reworded comment upstream does not fail the match while a changed token
/// still does. Comments must go first, or a `//` would swallow the rest of the
/// file once newlines are squeezed away.
std::string squeeze(const std::string& s) {
  std::string out = std::regex_replace(s, std::regex(R"(/\*[\s\S]*?\*/)"), " ");
  out = std::regex_replace(out, std::regex(R"(//[^\n]*)"), " ");
  return std::regex_replace(out, std::regex(R"(\s+)"), " ");
}

}  // namespace

TEST(SitlPackets, PortsMatchTheBetaflightSource) {
  const std::string src = squeeze(betaflightSource("src/main/target/SITL/sitl.c"));
  ASSERT_FALSE(src.empty()) << "third_party/betaflight submodule is not checked out";

  EXPECT_NE(src.find("#define PORT_PWM_RAW 9001"), std::string::npos);
  EXPECT_NE(src.find("#define PORT_PWM 9002"), std::string::npos);
  EXPECT_NE(src.find("#define PORT_STATE 9003"), std::string::npos);
  EXPECT_NE(src.find("#define PORT_RC 9004"), std::string::npos);

  EXPECT_EQ(fdt::sitl::kPortPwmRaw, 9001);
  EXPECT_EQ(fdt::sitl::kPortPwm, 9002);
  EXPECT_EQ(fdt::sitl::kPortState, 9003);
  EXPECT_EQ(fdt::sitl::kPortRc, 9004);
}

TEST(SitlPackets, ConfiguratorTcpPortMatchesTheSerialDriver) {
  const std::string src = squeeze(betaflightSource("src/main/drivers/serial_tcp.c"));
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("#define BASE_PORT 5760"), std::string::npos);
  // UART1 is id 0, and the listen port is BASE_PORT + id + 1.
  EXPECT_NE(src.find("BASE_PORT + id + 1"), std::string::npos);
  EXPECT_EQ(fdt::sitl::kPortConfiguratorTcp, 5760 + 0 + 1);
}

TEST(SitlPackets, ChannelAndPwmLimitsMatchTheTargetHeader) {
  const std::string src = squeeze(betaflightSource("src/main/target/SITL/target.h"));
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("#define SIMULATOR_MAX_RC_CHANNELS 16"), std::string::npos);
  EXPECT_NE(src.find("#define SIMULATOR_MAX_PWM_CHANNELS 16"), std::string::npos);
  EXPECT_EQ(fdt::sitl::kMaxRcChannels, 16u);
  EXPECT_EQ(fdt::sitl::kMaxPwmChannels, 16u);
}

TEST(SitlPackets, FieldOrderMatchesTheBetaflightStructDefinition) {
  // If upstream reorders a field, every offset static_assert in our header
  // could still pass while the meaning of the bytes changed. So check the
  // declaration order in the source itself.
  const std::string src = squeeze(betaflightSource("src/main/target/SITL/target.h"));
  ASSERT_FALSE(src.empty());

  const auto fdm = src.find("double timestamp; double imu_angular_velocity_rpy[3];");
  EXPECT_NE(fdm, std::string::npos) << "fdm_packet field order changed upstream";

  const std::string expected_fdm =
      "double timestamp; double imu_angular_velocity_rpy[3]; "
      "double imu_linear_acceleration_xyz[3]; double imu_orientation_quat[4]; "
      "double velocity_xyz[3]; double position_xyz[3]; double pressure; } fdm_packet;";
  EXPECT_NE(src.find(expected_fdm), std::string::npos) << "fdm_packet layout changed upstream";

  EXPECT_NE(src.find("double timestamp; uint16_t channels[SIMULATOR_MAX_RC_CHANNELS]; } rc_packet;"),
            std::string::npos);
  EXPECT_NE(src.find("float motor_speed[4];"), std::string::npos);
}

TEST(SitlPackets, MotorPermutationMatchesWhatSitlActuallyDoes) {
  // sitl.c:592-595, the remap that makes the packet NOT Betaflight motor order.
  const std::string src = squeeze(betaflightSource("src/main/target/SITL/sitl.c"));
  ASSERT_FALSE(src.empty());

  const std::string remap =
      "pwmPkt.motor_speed[3] = motorsPwm[0] / outScale; "
      "pwmPkt.motor_speed[0] = motorsPwm[1] / outScale; "
      "pwmPkt.motor_speed[1] = motorsPwm[2] / outScale; "
      "pwmPkt.motor_speed[2] = motorsPwm[3] / outScale;";
  ASSERT_NE(src.find(remap), std::string::npos)
      << "the SITL motor remap changed upstream; re-derive kPacketSlotToBetaflightMotor";

  // Encoded from exactly those four lines.
  EXPECT_EQ(fdt::sitl::betaflightMotorForPacketSlot(3), 0u);
  EXPECT_EQ(fdt::sitl::betaflightMotorForPacketSlot(0), 1u);
  EXPECT_EQ(fdt::sitl::betaflightMotorForPacketSlot(1), 2u);
  EXPECT_EQ(fdt::sitl::betaflightMotorForPacketSlot(2), 3u);

  EXPECT_EQ(fdt::sitl::packetSlotForBetaflightMotor(0), 3u);
  EXPECT_EQ(fdt::sitl::packetSlotForBetaflightMotor(1), 0u);
  EXPECT_EQ(fdt::sitl::packetSlotForBetaflightMotor(2), 1u);
  EXPECT_EQ(fdt::sitl::packetSlotForBetaflightMotor(3), 2u);

  // It is a genuine permutation, not the identity.
  bool any_moved = false;
  for (size_t slot = 0; slot < 4; ++slot) {
    if (fdt::sitl::betaflightMotorForPacketSlot(slot) != slot) any_moved = true;
  }
  EXPECT_TRUE(any_moved) << "if this is ever the identity, the remap was removed upstream";
}

TEST(SitlPackets, MotorScalingConstantsMatchTheSource) {
  const std::string src = squeeze(betaflightSource("src/main/target/SITL/sitl.c"));
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("double outScale = 1000.0;"), std::string::npos);
  EXPECT_NE(src.find("motorsPwm[index] = value - idlePulse;"), std::string::npos)
      << "the idle offset is subtracted by SITL; we must not re-apply it";
  EXPECT_DOUBLE_EQ(fdt::sitl::kMotorOutScale, 1000.0);

  EXPECT_NE(src.find("#define ACC_SCALE (256 / 9.80665)"), std::string::npos);
  EXPECT_NE(src.find("#define GYRO_SCALE (16.4)"), std::string::npos);
  EXPECT_DOUBLE_EQ(fdt::sitl::kSitlGyroScale, 16.4);
  EXPECT_DOUBLE_EQ(fdt::sitl::kSitlAccScale, 256.0 / 9.80665);
}

TEST(SitlPackets, AttitudeIsTakenFromOurQuaternionNotEstimated) {
  // target.h undefines USE_IMU_CALC, so sitl.c calls imuSetAttitudeQuat with
  // our quaternion instead of running Betaflight's attitude estimator. If that
  // ever changes, the accelerometer suddenly matters a great deal more and the
  // bridge needs revisiting.
  const std::string target = squeeze(betaflightSource("src/main/target/SITL/target.h"));
  const std::string src = squeeze(betaflightSource("src/main/target/SITL/sitl.c"));
  ASSERT_FALSE(target.empty());
  ASSERT_FALSE(src.empty());

  EXPECT_NE(target.find("#undef USE_IMU_CALC"), std::string::npos);
  EXPECT_NE(src.find("imuSetAttitudeQuat(pkt->imu_orientation_quat[0]"), std::string::npos);
}

TEST(SitlPackets, SitlResolvesTheTargetAddressWithInetAddrOnly) {
  // udplink.c uses inet_addr(), so the Docker entrypoint must hand SITL a
  // dotted IPv4 address -- "host.docker.internal" would silently fail.
  const std::string src = squeeze(betaflightSource("src/main/target/SITL/udplink.c"));
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("inet_addr(addr)"), std::string::npos);
  EXPECT_EQ(src.find("getaddrinfo"), std::string::npos) << "if this gains name resolution, simplify the entrypoint";
}
