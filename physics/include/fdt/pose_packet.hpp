// Pose packet: physics -> viewer, over UDP.
//
// Unlike the SITL packets, this format is OURS, so it is defined rather than
// reverse-engineered. Conventions match docs/coordinate_frames.md exactly and
// the viewer converts at its own boundary:
//
//   position, velocity   world NED, metres (so altitude is -z)
//   orientation          q_wb, Hamilton, SCALAR FIRST [w, x, y, z]
//
// The magic number and version exist because this is a UDP port on a machine
// that also runs SITL on four nearby ports; a stray packet should be rejected,
// not rendered.
#pragma once

#include <cstddef>
#include <cstdint>

namespace fdt {

/// 'FDTP' little-endian. Any packet without this is not ours.
inline constexpr uint32_t kPoseMagic = 0x50544446u;

/// Bump whenever a field moves. The viewer refuses a version it does not know
/// rather than silently misreading a shifted field.
inline constexpr uint16_t kPoseVersion = 1;

enum PoseFlags : uint16_t {
  kPoseArmed = 1u << 0,
  kPoseInContact = 1u << 1,
};

struct PosePacket {
  uint32_t magic = kPoseMagic;
  uint16_t version = kPoseVersion;
  uint16_t flags = 0;

  double time_s = 0.0;
  double position_ned[3]{};   ///< metres, world NED
  double orientation[4]{};    ///< w, x, y, z -- scalar first
  double velocity_ned[3]{};   ///< m/s, world NED

  float motor_rpm[4]{};
  float battery_v = 0.0f;
  float throttle = 0.0f;      ///< 0..1, the mean motor command
};

// Every field is naturally aligned, so the layout is stable across the
// compilers we care about and needs no packing attribute.
static_assert(sizeof(PosePacket) == 120, "pose packet must be 120 bytes");
static_assert(offsetof(PosePacket, magic) == 0);
static_assert(offsetof(PosePacket, version) == 4);
static_assert(offsetof(PosePacket, flags) == 6);
static_assert(offsetof(PosePacket, time_s) == 8);
static_assert(offsetof(PosePacket, position_ned) == 16);
static_assert(offsetof(PosePacket, orientation) == 40);
static_assert(offsetof(PosePacket, velocity_ned) == 72);
static_assert(offsetof(PosePacket, motor_rpm) == 96);
static_assert(offsetof(PosePacket, battery_v) == 112);
static_assert(offsetof(PosePacket, throttle) == 116);

}  // namespace fdt
