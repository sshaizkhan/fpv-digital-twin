// Transport tests. These need no SITL: a plain UDP socket stands in for it,
// which is enough to prove the bytes go to the right place unchanged.

#include "fdt/sitl_link.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <thread>

namespace {

using namespace std::chrono_literals;

/// A UDP socket on an ephemeral port, standing in for SITL.
class FakeSitl {
 public:
  FakeSitl() {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    EXPECT_GE(fd_, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    EXPECT_EQ(::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
    socklen_t len = sizeof(addr);
    EXPECT_EQ(::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len), 0);
    port_ = ntohs(addr.sin_port);
  }
  ~FakeSitl() { if (fd_ >= 0) ::close(fd_); }

  uint16_t port() const { return port_; }

  /// Receive into a buffer, returning the byte count (-1 on timeout).
  ssize_t receive(void* buf, size_t size, std::chrono::milliseconds timeout = 500ms) {
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout.count() / 1000);
    tv.tv_usec = static_cast<suseconds_t>((timeout.count() % 1000) * 1000);
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    if (::select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) return -1;
    return ::recvfrom(fd_, buf, size, 0, nullptr, nullptr);
  }

  /// Send arbitrary bytes to a port on loopback, as SITL would.
  void sendTo(uint16_t port, const void* data, size_t size) {
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(port);
    dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(::sendto(fd_, data, size, 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst)),
              static_cast<ssize_t>(size));
  }

 private:
  int fd_ = -1;
  uint16_t port_ = 0;
};

fdt::sitl::SitlLink::Endpoints loopbackTo(uint16_t state, uint16_t rc) {
  fdt::sitl::SitlLink::Endpoints e;
  e.sitl_host = "127.0.0.1";
  e.state_port = state;
  e.rc_port = rc;
  e.motor_port = 0;  // let the OS pick, so tests never collide with a real SITL
  return e;
}

}  // namespace

TEST(SitlLink, BindsAnEphemeralMotorPortWhenAskedForZero) {
  FakeSitl sitl;
  fdt::sitl::SitlLink link(loopbackTo(sitl.port(), sitl.port()));
  EXPECT_NE(link.boundMotorPort(), 0) << "the OS must have assigned a real port";
  EXPECT_EQ(link.malformedMotorPackets(), 0u);
}

TEST(SitlLink, RejectsAHostnameBecauseSitlCannotResolveOne) {
  auto endpoints = loopbackTo(9003, 9004);
  endpoints.sitl_host = "host.docker.internal";
  // SITL calls inet_addr (udplink.c:36), so a name would silently fail there.
  // Fail loudly here instead.
  EXPECT_THROW(fdt::sitl::SitlLink{endpoints}, fdt::sitl::SitlError);
}

TEST(SitlLink, TwoLinksCannotBindTheSameMotorPort) {
  FakeSitl sitl;
  auto first = loopbackTo(sitl.port(), sitl.port());
  fdt::sitl::SitlLink link(first);

  auto second = loopbackTo(sitl.port(), sitl.port());
  second.motor_port = link.boundMotorPort();
  // Without this the second simulator silently steals half the motor packets.
  EXPECT_THROW(fdt::sitl::SitlLink{second}, fdt::sitl::SitlError);
}

TEST(SitlLink, StatePacketArrivesByteForByte) {
  FakeSitl sitl;
  fdt::sitl::SitlLink link(loopbackTo(sitl.port(), sitl.port()));

  fdt::sitl::FdmPacket sent{};
  sent.timestamp = 12.5;
  sent.imu_angular_velocity_rpy[0] = 1.0;
  sent.imu_angular_velocity_rpy[1] = -2.0;
  sent.imu_angular_velocity_rpy[2] = 3.0;
  sent.imu_linear_acceleration_xyz[2] = -9.80665;
  sent.imu_orientation_quat[0] = 1.0;
  sent.position_xyz[2] = -10.0;
  sent.pressure = 101325.0;

  link.sendState(sent);

  fdt::sitl::FdmPacket got{};
  ASSERT_EQ(sitl.receive(&got, sizeof(got)), static_cast<ssize_t>(sizeof(got)))
      << "SITL must see exactly 144 bytes";
  EXPECT_EQ(std::memcmp(&sent, &got, sizeof(got)), 0) << "the packet must arrive unmodified";
  EXPECT_DOUBLE_EQ(got.timestamp, 12.5);
  EXPECT_DOUBLE_EQ(got.imu_angular_velocity_rpy[1], -2.0);
  EXPECT_DOUBLE_EQ(got.position_xyz[2], -10.0);
}

TEST(SitlLink, RcPacketArrivesByteForByteOnItsOwnPort) {
  FakeSitl state_sink, rc_sink;
  fdt::sitl::SitlLink link(loopbackTo(state_sink.port(), rc_sink.port()));

  fdt::sitl::RcPacket sent{};
  sent.timestamp = 3.25;
  for (size_t i = 0; i < fdt::sitl::kMaxRcChannels; ++i) {
    sent.channels[i] = static_cast<uint16_t>(1000 + i);
  }

  link.sendRc(sent);

  fdt::sitl::RcPacket got{};
  ASSERT_EQ(rc_sink.receive(&got, sizeof(got)), static_cast<ssize_t>(sizeof(got)))
      << "RC must go to the RC port, not the state port";
  EXPECT_EQ(std::memcmp(&sent, &got, sizeof(got)), 0);
  EXPECT_EQ(got.channels[0], 1000);
  EXPECT_EQ(got.channels[15], 1015);

  // And nothing should have gone to the state port.
  fdt::sitl::RcPacket stray{};
  EXPECT_EQ(state_sink.receive(&stray, sizeof(stray), 50ms), -1) << "RC leaked onto the state port";
}

TEST(SitlLink, ReceivesAMotorPacket) {
  FakeSitl sitl;
  fdt::sitl::SitlLink link(loopbackTo(sitl.port(), sitl.port()));

  fdt::sitl::ServoPacket sent{};
  sent.motor_speed[0] = 0.10f;
  sent.motor_speed[1] = 0.20f;
  sent.motor_speed[2] = 0.30f;
  sent.motor_speed[3] = 0.40f;
  sitl.sendTo(link.boundMotorPort(), &sent, sizeof(sent));

  fdt::sitl::ServoPacket got{};
  ASSERT_TRUE(link.receiveMotors(got, 500ms));
  EXPECT_FLOAT_EQ(got.motor_speed[0], 0.10f);
  EXPECT_FLOAT_EQ(got.motor_speed[3], 0.40f);
  EXPECT_EQ(link.malformedMotorPackets(), 0u);
}

TEST(SitlLink, ReturnsFalseOnTimeoutRatherThanBlockingForever) {
  FakeSitl sitl;
  fdt::sitl::SitlLink link(loopbackTo(sitl.port(), sitl.port()));

  const auto start = std::chrono::steady_clock::now();
  fdt::sitl::ServoPacket got{};
  EXPECT_FALSE(link.receiveMotors(got, 50ms));
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_GE(elapsed, 40ms) << "returned far too early to have waited";
  EXPECT_LT(elapsed, 2s) << "timeout was not honoured";
}

TEST(SitlLink, WrongSizedDatagramsAreCountedNotReturned) {
  FakeSitl sitl;
  fdt::sitl::SitlLink link(loopbackTo(sitl.port(), sitl.port()));

  // SITL's 68-byte servo_packet_raw, as if 9001 and 9002 were confused.
  fdt::sitl::ServoPacketRaw raw{};
  raw.motorCount = 4;
  sitl.sendTo(link.boundMotorPort(), &raw, sizeof(raw));

  fdt::sitl::ServoPacket got{};
  EXPECT_FALSE(link.receiveMotors(got, 100ms)) << "a 68-byte packet is not a servo_packet";
  EXPECT_EQ(link.malformedMotorPackets(), 1u) << "and it must be visible, not silently dropped";
}

TEST(SitlLink, AGoodPacketAfterABadOneIsStillDelivered) {
  FakeSitl sitl;
  fdt::sitl::SitlLink link(loopbackTo(sitl.port(), sitl.port()));

  fdt::sitl::ServoPacketRaw raw{};
  sitl.sendTo(link.boundMotorPort(), &raw, sizeof(raw));

  fdt::sitl::ServoPacket good{};
  good.motor_speed[2] = 0.75f;
  sitl.sendTo(link.boundMotorPort(), &good, sizeof(good));

  fdt::sitl::ServoPacket got{};
  ASSERT_TRUE(link.receiveMotors(got, 500ms));
  EXPECT_FLOAT_EQ(got.motor_speed[2], 0.75f);
  EXPECT_EQ(link.malformedMotorPackets(), 1u);
}

TEST(SitlLink, DrainDiscardsTheBacklogAndReportsHowMuch) {
  FakeSitl sitl;
  fdt::sitl::SitlLink link(loopbackTo(sitl.port(), sitl.port()));

  fdt::sitl::ServoPacket stale{};
  for (int i = 0; i < 5; ++i) {
    stale.motor_speed[0] = static_cast<float>(i);
    sitl.sendTo(link.boundMotorPort(), &stale, sizeof(stale));
  }
  // Give the loopback a moment to queue them all.
  std::this_thread::sleep_for(50ms);

  EXPECT_EQ(link.drainMotors(), 5u);
  fdt::sitl::ServoPacket got{};
  EXPECT_FALSE(link.receiveMotors(got, 50ms)) << "the queue should be empty after a drain";
}

TEST(SitlLink, IsMovableSoItCanLiveInAContainer) {
  FakeSitl sitl;
  fdt::sitl::SitlLink first(loopbackTo(sitl.port(), sitl.port()));
  const uint16_t port = first.boundMotorPort();

  fdt::sitl::SitlLink second(std::move(first));
  EXPECT_EQ(second.boundMotorPort(), port);

  // The moved-to link must still work.
  fdt::sitl::ServoPacket sent{};
  sent.motor_speed[1] = 0.5f;
  sitl.sendTo(port, &sent, sizeof(sent));
  fdt::sitl::ServoPacket got{};
  ASSERT_TRUE(second.receiveMotors(got, 500ms));
  EXPECT_FLOAT_EQ(got.motor_speed[1], 0.5f);
}
