// The real transmitter, read over SDL2 as a USB HID joystick.
//
// The T-Pro V2 in EdgeTX's "USB Joystick" mode enumerates as
// "Jumper TPro V2 Joystick" (VID 0x1209, PID 0x4f54). EdgeTX decides which
// internal channel lands on which HID axis, so NOTHING about the axis order is
// assumed here: the mapping lives in config/radio.yaml, produced by
// fdt_radio_probe watching you move the sticks.
//
// Raw SDL axes are int16 (-32768..32767). RC channels are microseconds
// (1000..2000). The conversion is per-channel, with its own range and
// direction, because a transmitter's idea of "up" is not something to guess.
#pragma once

#include "fdt/sitl_bridge.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace fdt {

class RadioError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// One transmitter control mapped onto one RC channel.
struct ChannelMap {
  int source_index = -1;        ///< SDL axis index, or button/hat index
  enum class Source { Axis, Button, Hat } source = Source::Axis;
  bool inverted = false;
  int16_t raw_min = -32768;     ///< raw value that should read 1000 us
  int16_t raw_max = 32767;      ///< raw value that should read 2000 us
  int16_t deadband = 0;         ///< around centre, raw units

  /// Raw SDL value -> microseconds, clamped to [1000, 2000].
  uint16_t toMicroseconds(int raw) const;
};

struct RadioDevice {
  int index = -1;
  std::string name;
  int axes = 0;
  int buttons = 0;
  int hats = 0;
  std::string guid;
};

/// Live view of every control, before mapping. What fdt_radio_probe shows.
struct RadioSnapshot {
  std::vector<int16_t> axes;
  std::vector<bool> buttons;
  std::vector<int> hats;
};

/// A full channel map, loaded from config/radio.yaml.
struct RadioConfig {
  std::string device_name;
  std::string guid;
  int expected_axes = 0;
  std::array<ChannelMap, sitl::kMaxRcChannels> channels{};

  /// Channels that were actually assigned, for reporting.
  std::vector<std::pair<std::string, size_t>> named;
};

/// Load config/radio.yaml. Throws RadioError on anything malformed -- a radio
/// map that is silently half-loaded would fly the quad sideways.
RadioConfig loadRadioConfig(const std::string& path);

class Radio {
 public:
  /// Opens joystick `index`. Throws RadioError if SDL or the device fails.
  explicit Radio(int index = 0);
  ~Radio();

  Radio(const Radio&) = delete;
  Radio& operator=(const Radio&) = delete;

  /// Every joystick SDL can see. Static so it works before opening one.
  static std::vector<RadioDevice> enumerate();

  const RadioDevice& device() const { return device_; }

  /// Pump SDL's event queue and refresh the cached control values. Must be
  /// called regularly or the values go stale.
  void poll();

  RadioSnapshot snapshot() const;

  int16_t axis(int index) const;
  bool button(int index) const;

  /// Apply a channel map to produce RC channels for the SITL bridge.
  sitl::RcChannels toRcChannels(const std::array<ChannelMap, sitl::kMaxRcChannels>& map,
                                const ArmSwitch& arm) const;

 private:
  void* joystick_ = nullptr;  ///< SDL_Joystick*, kept opaque to avoid the header
  RadioDevice device_;
  static bool sdl_initialised_;
};

}  // namespace fdt
