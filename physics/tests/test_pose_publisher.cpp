// Pose packet and publisher tests. The wire format is ours, so these pin the
// conventions the viewer will decode against.

#include "fdt/pose_publisher.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace {

fdt::QuadConfig config() { return fdt::loadQuadConfig(std::string(FDT_REPO_ROOT) + "/config/quad.yaml"); }

/// A UDP socket standing in for the viewer.
class FakeViewer {
 public:
  FakeViewer() {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    ::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    socklen_t len = sizeof(addr);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len);
    port_ = ntohs(addr.sin_port);
  }
  ~FakeViewer() { if (fd_ >= 0) ::close(fd_); }

  uint16_t port() const { return port_; }

  bool receive(fdt::PosePacket& out, int timeout_ms = 500) {
    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    if (::select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) return false;
    return ::recvfrom(fd_, &out, sizeof(out), 0, nullptr, nullptr) ==
           static_cast<ssize_t>(sizeof(out));
  }

 private:
  int fd_ = -1;
  uint16_t port_ = 0;
};

}  // namespace

TEST(PosePacket, HasTheDocumentedWireLayout) {
  EXPECT_EQ(sizeof(fdt::PosePacket), 120u);
  fdt::PosePacket p;
  EXPECT_EQ(p.magic, fdt::kPoseMagic);
  EXPECT_EQ(p.version, 1);
  // 'FDTP' little-endian, so the first byte on the wire is 'F'.
  unsigned char bytes[4];
  std::memcpy(bytes, &p.magic, 4);
  EXPECT_EQ(bytes[0], 'F');
  EXPECT_EQ(bytes[1], 'D');
  EXPECT_EQ(bytes[2], 'T');
  EXPECT_EQ(bytes[3], 'P');
}

TEST(PosePacket, QuaternionIsScalarFirst) {
  fdt::Multirotor quad(config());
  fdt::State s;
  // A rotation whose four components are all distinct, so an ordering error
  // cannot hide behind a symmetry.
  s.orientation = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);  // Eigen ctor is (w,x,y,z)
  quad.reset(s);

  const fdt::PosePacket p = fdt::makePosePacket(quad, false, 0.0);
  EXPECT_DOUBLE_EQ(p.orientation[0], quad.state().orientation.w()) << "index 0 must be w";
  EXPECT_DOUBLE_EQ(p.orientation[1], quad.state().orientation.x());
  EXPECT_DOUBLE_EQ(p.orientation[2], quad.state().orientation.y());
  EXPECT_DOUBLE_EQ(p.orientation[3], quad.state().orientation.z());
}

TEST(PosePacket, PositionIsNedSoAltitudeIsNegativeZ) {
  fdt::Multirotor quad(config());
  fdt::State s;
  s.position = Eigen::Vector3d(1.0, 2.0, -30.0);  // 30 m UP
  quad.reset(s);

  const fdt::PosePacket p = fdt::makePosePacket(quad, false, 0.0);
  EXPECT_DOUBLE_EQ(p.position_ned[2], -30.0);
  EXPECT_DOUBLE_EQ(quad.state().altitude(), 30.0) << "the viewer must flip this itself";
}

TEST(PosePacket, FlagsCarryArmedAndContact) {
  fdt::Multirotor quad(config());
  quad.placeOnGround();
  quad.step(1.0 / 8000.0);

  const fdt::PosePacket parked = fdt::makePosePacket(quad, false, 0.0);
  EXPECT_FALSE(parked.flags & fdt::kPoseArmed);
  EXPECT_TRUE(parked.flags & fdt::kPoseInContact) << "a parked quad is touching the ground";

  const fdt::PosePacket armed = fdt::makePosePacket(quad, true, 0.5);
  EXPECT_TRUE(armed.flags & fdt::kPoseArmed);
  EXPECT_FLOAT_EQ(armed.throttle, 0.5f);
}

TEST(PosePublisher, PacketArrivesByteForByte) {
  FakeViewer viewer;
  fdt::PosePublisher publisher("127.0.0.1", viewer.port());

  fdt::Multirotor quad(config());
  fdt::State s;
  s.position = Eigen::Vector3d(3.0, -4.0, -12.0);
  s.velocity = Eigen::Vector3d(1.0, 0.0, -2.0);
  quad.reset(s);

  const fdt::PosePacket sent = fdt::makePosePacket(quad, true, 0.42);
  publisher.send(sent);

  fdt::PosePacket got{};
  ASSERT_TRUE(viewer.receive(got));
  EXPECT_EQ(std::memcmp(&sent, &got, sizeof(got)), 0);
  EXPECT_DOUBLE_EQ(got.position_ned[2], -12.0);
  EXPECT_FLOAT_EQ(got.throttle, 0.42f);
  EXPECT_EQ(publisher.sent(), 1u);
}

TEST(PosePublisher, NoViewerListeningIsNotAnError) {
  // The viewer is a spectator: it must never be able to stall or fail the sim.
  fdt::PosePublisher publisher("127.0.0.1", 9);  // discard port, nothing bound
  fdt::Multirotor quad(config());
  for (int i = 0; i < 10; ++i) {
    EXPECT_NO_THROW(publisher.publish(quad, false, 0.0));
  }
}

TEST(PosePublisher, ThrottlesToTheRequestedRate) {
  FakeViewer viewer;
  fdt::PosePublisher publisher("127.0.0.1", viewer.port());
  fdt::Multirotor quad(config());
  quad.reset();

  // One simulated second at 8 kHz, published at 120 Hz.
  const double dt = 1.0 / 8000.0;
  int published = 0;
  for (int i = 0; i < 8000; ++i) {
    quad.step(dt);
    if (publisher.publishThrottled(quad, false, 0.0, 120.0)) ++published;
  }
  EXPECT_GE(published, 110);
  EXPECT_LE(published, 130) << "publishing every physics step would flood the viewer";
}

TEST(PosePublisher, RejectsAHostname) {
  EXPECT_THROW(fdt::PosePublisher("localhost", 9100), fdt::ConfigError);
}
