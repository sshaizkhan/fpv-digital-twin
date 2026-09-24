#include "fdt/radio.hpp"

#include <SDL.h>

#include <algorithm>
#include <fstream>
#include <cmath>

namespace fdt {

bool Radio::sdl_initialised_ = false;

namespace {

void ensureSdl() {
  if (SDL_WasInit(SDL_INIT_JOYSTICK)) return;
  // No video: this is a headless process and initialising video on macOS wants
  // a window server connection we neither have nor need.
  SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
  if (SDL_Init(SDL_INIT_JOYSTICK) != 0) {
    throw RadioError(std::string("SDL_Init(JOYSTICK) failed: ") + SDL_GetError());
  }
}

}  // namespace

uint16_t ChannelMap::toMicroseconds(int raw) const {
  if (source == Source::Button) {
    const uint16_t on = inverted ? 1000 : 2000;
    const uint16_t off = inverted ? 2000 : 1000;
    return raw ? on : off;
  }

  int value = raw;
  if (deadband > 0 && std::abs(value) < deadband) value = 0;

  const double lo = static_cast<double>(raw_min);
  const double hi = static_cast<double>(raw_max);
  if (hi <= lo) return 1500;

  double t = (static_cast<double>(value) - lo) / (hi - lo);
  if (inverted) t = 1.0 - t;
  const double us = 1000.0 + t * 1000.0;
  return static_cast<uint16_t>(std::lround(std::clamp(us, 1000.0, 2000.0)));
}

std::vector<RadioDevice> Radio::enumerate() {
  ensureSdl();
  SDL_JoystickUpdate();

  std::vector<RadioDevice> out;
  const int count = SDL_NumJoysticks();
  for (int i = 0; i < count; ++i) {
    RadioDevice d;
    d.index = i;
    const char* name = SDL_JoystickNameForIndex(i);
    d.name = name ? name : "(unnamed)";

    char guid[64] = {0};
    SDL_JoystickGetGUIDString(SDL_JoystickGetDeviceGUID(i), guid, sizeof(guid));
    d.guid = guid;

    // Opening is the only way to learn the control counts.
    if (SDL_Joystick* js = SDL_JoystickOpen(i)) {
      d.axes = SDL_JoystickNumAxes(js);
      d.buttons = SDL_JoystickNumButtons(js);
      d.hats = SDL_JoystickNumHats(js);
      SDL_JoystickClose(js);
    }
    out.push_back(d);
  }
  return out;
}

Radio::Radio(int index) {
  ensureSdl();
  SDL_JoystickUpdate();

  if (index < 0 || index >= SDL_NumJoysticks()) {
    throw RadioError("no joystick at index " + std::to_string(index) + " (" +
                     std::to_string(SDL_NumJoysticks()) +
                     " connected). Is the radio in USB Joystick mode?");
  }

  SDL_Joystick* js = SDL_JoystickOpen(index);
  if (!js) throw RadioError(std::string("cannot open joystick: ") + SDL_GetError());
  joystick_ = js;

  device_.index = index;
  const char* name = SDL_JoystickName(js);
  device_.name = name ? name : "(unnamed)";
  device_.axes = SDL_JoystickNumAxes(js);
  device_.buttons = SDL_JoystickNumButtons(js);
  device_.hats = SDL_JoystickNumHats(js);

  char guid[64] = {0};
  SDL_JoystickGetGUIDString(SDL_JoystickGetGUID(js), guid, sizeof(guid));
  device_.guid = guid;
}

Radio::~Radio() {
  if (joystick_) SDL_JoystickClose(static_cast<SDL_Joystick*>(joystick_));
}

void Radio::poll() {
  // Drain the event queue, otherwise SDL stops delivering device state.
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
  }
  SDL_JoystickUpdate();
}

int16_t Radio::axis(int index) const {
  if (!joystick_ || index < 0 || index >= device_.axes) return 0;
  return SDL_JoystickGetAxis(static_cast<SDL_Joystick*>(joystick_), index);
}

bool Radio::button(int index) const {
  if (!joystick_ || index < 0 || index >= device_.buttons) return false;
  return SDL_JoystickGetButton(static_cast<SDL_Joystick*>(joystick_), index) != 0;
}

RadioSnapshot Radio::snapshot() const {
  RadioSnapshot s;
  s.axes.reserve(static_cast<size_t>(device_.axes));
  for (int i = 0; i < device_.axes; ++i) s.axes.push_back(axis(i));
  for (int i = 0; i < device_.buttons; ++i) s.buttons.push_back(button(i));
  for (int i = 0; i < device_.hats; ++i) {
    s.hats.push_back(SDL_JoystickGetHat(static_cast<SDL_Joystick*>(joystick_), i));
  }
  return s;
}

namespace {

/// Minimal reader for the flat structure of radio.yaml. Deliberately not a
/// general YAML parser: it must fail loudly on anything it does not recognise
/// rather than quietly skipping a channel.
std::string trimmed(const std::string& s) {
  const size_t b = s.find_first_not_of(" \t");
  if (b == std::string::npos) return "";
  const size_t e = s.find_last_not_of(" \t\r");
  return s.substr(b, e - b + 1);
}

}  // namespace

RadioConfig loadRadioConfig(const std::string& path) {
  std::ifstream in(path);
  if (!in.good()) throw RadioError("cannot open radio config: " + path);

  RadioConfig config;
  for (auto& c : config.channels) c.source_index = -1;

  std::string line;
  std::string current_name;
  int current_channel = -1;
  ChannelMap current{};
  bool in_channel = false;

  auto commit = [&]() {
    if (!in_channel) return;
    if (current_channel < 0 || current_channel >= static_cast<int>(sitl::kMaxRcChannels)) {
      throw RadioError(path + ": '" + current_name + "' has no valid channel index");
    }
    if (current.source_index < 0) {
      throw RadioError(path + ": '" + current_name + "' has no axis");
    }
    config.channels[static_cast<size_t>(current_channel)] = current;
    config.named.emplace_back(current_name, static_cast<size_t>(current_channel));
    in_channel = false;
  };

  while (std::getline(in, line)) {
    const size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    const std::string t = trimmed(line);
    if (t.empty()) continue;

    const size_t colon = t.find(':');
    if (colon == std::string::npos) continue;
    const std::string key = trimmed(t.substr(0, colon));
    const std::string value = trimmed(t.substr(colon + 1));
    const size_t indent = line.find_first_not_of(" \t");

    if (indent == 2 && value.empty()) {  // a channel block under `channels:`
      commit();
      current_name = key;
      current = ChannelMap{};
      current.source_index = -1;
      current_channel = -1;
      in_channel = true;
      continue;
    }
    if (value.empty()) continue;

    auto asInt = [&](const std::string& v) {
      try {
        return std::stoi(v);
      } catch (const std::exception&) {
        throw RadioError(path + ": '" + key + "' is not a number: " + v);
      }
    };

    if (in_channel) {
      if (key == "channel") current_channel = asInt(value);
      else if (key == "axis") current.source_index = asInt(value);
      else if (key == "raw_min") current.raw_min = static_cast<int16_t>(asInt(value));
      else if (key == "raw_max") current.raw_max = static_cast<int16_t>(asInt(value));
      else if (key == "deadband") current.deadband = static_cast<int16_t>(asInt(value));
      else if (key == "inverted") current.inverted = (value == "true");
    } else {
      if (key == "name") config.device_name = value.substr(0, value.find_last_not_of("\"") + 1)
                                                   .substr(value.find_first_not_of("\""));
      else if (key == "guid") config.guid = value.substr(0, value.find_last_not_of("\"") + 1)
                                                 .substr(value.find_first_not_of("\""));
      else if (key == "axes") config.expected_axes = asInt(value);
    }
  }
  commit();

  if (config.named.empty()) throw RadioError(path + ": no channels defined");
  return config;
}

sitl::RcChannels Radio::toRcChannels(
    const std::array<ChannelMap, sitl::kMaxRcChannels>& map, const ArmSwitch& arm) const {
  sitl::RcChannels rc = sitl::RcChannels::neutral(arm);
  for (size_t channel = 0; channel < sitl::kMaxRcChannels; ++channel) {
    const ChannelMap& m = map[channel];
    if (m.source_index < 0) continue;  // unmapped: keep the neutral value
    const int raw = (m.source == ChannelMap::Source::Button) ? (button(m.source_index) ? 1 : 0)
                                                             : axis(m.source_index);
    rc.us[channel] = m.toMicroseconds(raw);
  }
  return rc;
}

}  // namespace fdt
