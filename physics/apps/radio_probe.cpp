// fdt_radio_probe - see what the transmitter actually sends.
//
//   fdt_radio_probe                 list joysticks
//   fdt_radio_probe --watch         live dump of every axis and button
//   fdt_radio_probe --calibrate     record each control's travel
//
// EdgeTX decides which of its internal channels lands on which HID axis, and
// that depends on the model's Mixes page. So no axis order is assumed: this
// watches what the radio sends and reports it, and the mapping is written down
// afterwards rather than guessed.

#include "fdt/radio.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace {

using namespace std::chrono_literals;

std::string bar(int16_t value, int width = 21) {
  // -32768..32767 onto a centred bar, so a stick at rest sits in the middle.
  const int centre = width / 2;
  const int pos = static_cast<int>(std::lround((value / 32767.0) * centre)) + centre;
  std::string out(static_cast<size_t>(width), '-');
  out[static_cast<size_t>(centre)] = '|';
  out[static_cast<size_t>(std::clamp(pos, 0, width - 1))] = '#';
  return out;
}

int listDevices() {
  const auto devices = fdt::Radio::enumerate();
  if (devices.empty()) {
    std::cout << "No joysticks found.\n\n"
                 "On the T-Pro: hold the power button, choose USB Joystick (not USB\n"
                 "Storage, which is the SD card, nor USB Serial). macOS should then\n"
                 "show it as \"Jumper TPro V2 Joystick\".\n";
    return 1;
  }
  std::cout << devices.size() << " joystick(s):\n";
  for (const auto& d : devices) {
    std::cout << "  [" << d.index << "] " << d.name << "\n"
              << "      " << d.axes << " axes, " << d.buttons << " buttons, " << d.hats
              << " hats\n"
              << "      guid " << d.guid << "\n";
  }
  return 0;
}

int watch(int index) {
  fdt::Radio radio(index);
  const auto& d = radio.device();
  std::cout << "watching [" << d.index << "] " << d.name << " -- Ctrl-C to stop\n";
  std::cout << "move every stick and flip every switch; note which line responds\n\n";

  for (;;) {
    radio.poll();
    const auto s = radio.snapshot();

    std::cout << "\033[H\033[J";  // home + clear, so it updates in place
    std::cout << d.name << "\n\n";
    for (size_t i = 0; i < s.axes.size(); ++i) {
      std::cout << "  axis " << std::setw(2) << i << "  " << std::setw(7) << s.axes[i] << "  "
                << bar(s.axes[i]) << "\n";
    }
    if (!s.buttons.empty()) {
      std::cout << "\n  buttons ";
      for (size_t i = 0; i < s.buttons.size(); ++i) {
        std::cout << (s.buttons[i] ? "[" + std::to_string(i) + "]" : " . ");
      }
      std::cout << "\n";
    }
    std::cout.flush();
    std::this_thread::sleep_for(50ms);
  }
}

int calibrate(int index, double seconds) {
  const bool interactive = isatty(fileno(stdout));
  fdt::Radio radio(index);
  const auto& d = radio.device();

  std::cout << "Calibrating [" << d.index << "] " << d.name << "\n\n";
  std::cout << "For the next " << seconds << " seconds:\n"
               "  * move BOTH sticks to all four corners, slowly, a few times\n"
               "  * flip EVERY switch through all its positions\n"
               "  * then centre the sticks and leave them alone for the last second\n\n"
               "Starting in 3... " << std::flush;
  std::this_thread::sleep_for(3s);
  std::cout << "go.\n";

  const size_t n = static_cast<size_t>(d.axes);
  std::vector<int16_t> lo(n, 32767), hi(n, -32768);
  std::vector<bool> ever_on(static_cast<size_t>(d.buttons), false);
  std::vector<bool> ever_off(static_cast<size_t>(d.buttons), false);

  const auto start = std::chrono::steady_clock::now();
  const auto finish = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(seconds));
  const auto settle_from = finish - 1s;

  std::vector<long long> centre_sum(n, 0);
  long long centre_samples = 0;

  while (std::chrono::steady_clock::now() < finish) {
    radio.poll();
    const bool settling = std::chrono::steady_clock::now() >= settle_from;
    for (size_t i = 0; i < n; ++i) {
      const int16_t v = radio.axis(static_cast<int>(i));
      lo[i] = std::min(lo[i], v);
      hi[i] = std::max(hi[i], v);
      if (settling) centre_sum[i] += v;
    }
    if (settling) ++centre_samples;
    for (int b = 0; b < d.buttons; ++b) {
      (radio.button(b) ? ever_on : ever_off)[static_cast<size_t>(b)] = true;
    }
    std::this_thread::sleep_for(5ms);

    // Only draw the in-place counter on a terminal; piped or redirected, the
    // carriage returns do nothing and it becomes hundreds of lines of noise.
    if (interactive) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      std::cout << "\r  " << std::fixed << std::setprecision(1) << elapsed << " s" << std::flush;
    }
  }
  if (interactive) std::cout << "\r                    \r";

  std::cout << "\nAxis travel:\n";
  std::cout << "  axis      min      max    centre   travel   looks like\n";
  for (size_t i = 0; i < n; ++i) {
    const long centre = centre_samples ? static_cast<long>(centre_sum[i] / centre_samples) : 0;
    const int travel = hi[i] - lo[i];
    std::string guess = "unused";
    if (travel > 20000) {
      // A stick returns to the middle; a throttle or switch rests at an end.
      guess = (std::abs(centre) < 6000) ? "self-centring stick" : "throttle or switch";
    } else if (travel > 3000) {
      guess = "partial travel -- move it further?";
    }
    std::cout << "  " << std::setw(4) << i << " " << std::setw(8) << lo[i] << " " << std::setw(8)
              << hi[i] << " " << std::setw(9) << centre << " " << std::setw(8) << travel << "   "
              << guess << "\n";
  }

  int moved = 0;
  for (int b = 0; b < d.buttons; ++b) {
    if (ever_on[static_cast<size_t>(b)] && ever_off[static_cast<size_t>(b)]) ++moved;
  }
  std::cout << "\nButtons that changed state: " << moved << " of " << d.buttons << "\n";

  std::cout << "\nNext: tell me which axis moved for roll, pitch, throttle and yaw,\n"
               "and which one is your arm switch. I will write config/radio.yaml\n"
               "from these numbers rather than assuming an order.\n";
  return 0;
}

/// Sample every axis for `seconds`, returning per-axis (min, max) and the mean
/// over the final `settle` seconds.
struct Sweep {
  std::vector<int16_t> lo, hi;
  std::vector<long> settled;
};

Sweep sample(fdt::Radio& radio, double seconds, double settle) {
  const size_t n = static_cast<size_t>(radio.device().axes);
  Sweep s{std::vector<int16_t>(n, 32767), std::vector<int16_t>(n, -32768), std::vector<long>(n, 0)};
  std::vector<long long> sum(n, 0);
  long long settle_samples = 0;

  const auto start = std::chrono::steady_clock::now();
  const auto finish = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(seconds));
  const auto settle_from = finish - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                        std::chrono::duration<double>(settle));
  while (std::chrono::steady_clock::now() < finish) {
    radio.poll();
    const bool settling = std::chrono::steady_clock::now() >= settle_from;
    for (size_t i = 0; i < n; ++i) {
      const int16_t v = radio.axis(static_cast<int>(i));
      s.lo[i] = std::min(s.lo[i], v);
      s.hi[i] = std::max(s.hi[i], v);
      if (settling) sum[i] += v;
    }
    if (settling) ++settle_samples;
    std::this_thread::sleep_for(5ms);
  }
  for (size_t i = 0; i < n; ++i) {
    s.settled[i] = settle_samples ? static_cast<long>(sum[i] / settle_samples) : 0;
  }
  return s;
}

void countdown(const std::string& what, int seconds) {
  std::cout << "\n  >>> " << what << "\n      starting in ";
  for (int i = 3; i > 0; --i) {
    std::cout << i << "... " << std::flush;
    std::this_thread::sleep_for(1s);
  }
  std::cout << "GO (" << seconds << "s)" << std::endl;
}

struct Assignment {
  std::string name;
  std::string move_prompt;
  std::string hold_prompt;
  /// Does the hold position correspond to the HIGH end (2000 us) or the LOW
  /// end (1000 us)? Throttle is held at IDLE, which is the low end -- assuming
  /// every hold was the high end silently reversed the throttle.
  bool hold_is_high = true;
  int axis = -1;
  int16_t lo = 0, hi = 0;
  bool inverted = false;
};

int wizard(int index) {
  fdt::Radio radio(index);
  std::cout << "Guided setup for [" << index << "] " << radio.device().name << "\n"
            << "One control at a time. Do nothing else while each one is running.\n";

  // Prompts name the PHYSICAL control, not just its function. Which stick does
  // what depends on the radio's mode, so the Mode 2 layout is stated and the
  // function is given alongside it -- "the roll stick" alone is ambiguous on a
  // transmitter with two identical sticks.
  std::cout << "\nAssuming MODE 2 (the usual default):\n"
               "    LEFT stick  = throttle (up/down) and yaw (left/right)\n"
               "    RIGHT stick = pitch (up/down) and roll (left/right)\n"
               "  On Mode 1, pitch and throttle are swapped. Go by the FUNCTION named\n"
               "  in each prompt; the stick named after it is only a hint.\n"
               "  ARM is a SWITCH (SA-SD on the shoulders), not a stick.\n";

  std::vector<Assignment> controls = {
      {"roll",     "ROLL: move the RIGHT stick fully LEFT and RIGHT, several times",
                   "now HOLD the RIGHT stick fully RIGHT", true, -1, 0, 0, false},
      {"pitch",    "PITCH: move the RIGHT stick fully UP and DOWN, several times",
                   "now HOLD the RIGHT stick fully UP (nose up)", true, -1, 0, 0, false},
      {"throttle", "THROTTLE: move the LEFT stick fully UP and DOWN, several times",
                   "now HOLD the LEFT stick fully DOWN (idle, motors off)", false, -1, 0, 0, false},
      {"yaw",      "YAW: move the LEFT stick fully LEFT and RIGHT, several times",
                   "now HOLD the LEFT stick fully RIGHT", true, -1, 0, 0, false},
      {"arm",      "ARM: flip your ARM SWITCH (not a stick) back and forth, several times",
                   "now leave the ARM SWITCH in the ARMED position", true, -1, 0, 0, false},
  };

  std::vector<bool> taken(static_cast<size_t>(radio.device().axes), false);

  for (auto& c : controls) {
    countdown(c.move_prompt, 5);
    const Sweep sweep = sample(radio, 5.0, 0.1);

    int best = -1, best_travel = 0;
    for (size_t i = 0; i < sweep.lo.size(); ++i) {
      if (taken[i]) continue;
      const int travel = sweep.hi[i] - sweep.lo[i];
      if (travel > best_travel) { best_travel = travel; best = static_cast<int>(i); }
    }
    if (best < 0 || best_travel < 8000) {
      std::cout << "      no axis moved enough for " << c.name
                << " (largest travel " << best_travel << ").\n"
                   "      That control is probably not exported by EdgeTX's USB Joystick\n"
                   "      settings. Skipping it.\n";
      continue;
    }
    c.axis = best;
    c.lo = sweep.lo[static_cast<size_t>(best)];
    c.hi = sweep.hi[static_cast<size_t>(best)];
    taken[static_cast<size_t>(best)] = true;
    std::cout << "      -> axis " << best << "  range " << c.lo << " .. " << c.hi << "\n";

    countdown(c.hold_prompt, 3);
    const Sweep held = sample(radio, 3.0, 1.0);
    const long at_extreme = held.settled[static_cast<size_t>(best)];

    // The hold often reaches further than the sweep did, so fold it into the
    // range. Without this a stick that was not quite maxed during the sweep
    // gets a short range and is scaled wrong for the rest of its travel.
    c.lo = std::min(c.lo, static_cast<int16_t>(std::clamp<long>(at_extreme, -32768, 32767)));
    c.hi = std::max(c.hi, static_cast<int16_t>(std::clamp<long>(at_extreme, -32768, 32767)));

    const long midpoint = (static_cast<long>(c.lo) + c.hi) / 2;
    const bool held_at_high_end = at_extreme > midpoint;
    // Inverted when the held position is NOT at the end it should map to.
    c.inverted = (held_at_high_end != c.hold_is_high);
    std::cout << "      held at " << at_extreme << " ("
              << (held_at_high_end ? "high" : "low") << " end, expected "
              << (c.hold_is_high ? "high" : "low") << ") -> "
              << (c.inverted ? "INVERTED" : "normal") << "\n";
  }

  std::cout << "\n\n--- result ---\n";
  for (const auto& c : controls) {
    if (c.axis < 0) {
      std::cout << "  " << std::setw(9) << c.name << "  NOT FOUND\n";
    } else {
      std::cout << "  " << std::setw(9) << c.name << "  axis " << c.axis << "  range " << c.lo
                << " .. " << c.hi << (c.inverted ? "  inverted" : "") << "\n";
    }
  }
  std::cout << "\nPaste this back and I will write config/radio.yaml from it.\n";
  return 0;
}

/// Live readout of the MAPPED channels, so the mapping can be checked before
/// anything is flown with it. Reading 1000-2000 us with the right control
/// moving the right channel is the last cheap check before a quad is armed.
int showRc(int index, const std::string& radio_config, const std::string& quad_config) {
  const fdt::RadioConfig map = fdt::loadRadioConfig(radio_config);
  const fdt::QuadConfig quad = fdt::loadQuadConfig(quad_config);
  const fdt::ArmSwitch& arm = quad.firmware.arm_switch;

  fdt::Radio radio(index);
  std::cout << "mapped channels from " << radio_config << "\n"
            << "arm band " << arm.range_start_us << "-" << arm.range_end_us
            << " us on AUX" << arm.aux << ", min_check is 1050\n"
            << "Ctrl-C to stop\n";

  for (;;) {
    radio.poll();
    const fdt::sitl::RcChannels rc = radio.toRcChannels(map.channels, arm);

    std::cout << "\033[H\033[J" << radio.device().name << "\n\n";
    for (const auto& [name, channel] : map.named) {
      const uint16_t us = rc.us[channel];
      std::cout << "  " << std::setw(9) << name << "  ch" << channel << "  " << std::setw(5) << us
                << " us  " << bar(static_cast<int16_t>((us - 1500) * 65), 21);
      if (name == "throttle") {
        std::cout << (us < 1050 ? "   idle (can arm)" : "   ABOVE min_check");
      } else if (channel == static_cast<size_t>(arm.aux) + 3) {
        std::cout << (arm.isArmed(us) ? "   >>> ARMED <<<" : "   disarmed");
      }
      std::cout << "\n";
    }
    std::cout << "\nmove a control and check the RIGHT row responds.\n";
    std::cout.flush();
    std::this_thread::sleep_for(50ms);
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string mode = "list";
  int index = 0;
  double seconds = 20.0;
  std::string radio_config = "config/radio.yaml";
  std::string quad_config = "config/quad.yaml";

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--watch") mode = "watch";
    else if (a == "--calibrate") mode = "calibrate";
    else if (a == "--wizard") mode = "wizard";
    else if (a == "--rc") mode = "rc";
    else if (a == "--radio-config" && i + 1 < argc) radio_config = argv[++i];
    else if (a == "--config" && i + 1 < argc) quad_config = argv[++i];
    else if (a == "--index" && i + 1 < argc) index = std::stoi(argv[++i]);
    else if (a == "--seconds" && i + 1 < argc) seconds = std::stod(argv[++i]);
    else if (a == "-h" || a == "--help") {
      std::cout << "usage: fdt_radio_probe [--watch | --calibrate | --wizard | --rc]\n"
                   "                       [--index N] [--seconds S]\n"
                   "                       [--radio-config PATH] [--config PATH]\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << a << "\n";
      return 2;
    }
  }

  try {
    if (mode == "list") return listDevices();
    if (mode == "watch") return watch(index);
    if (mode == "wizard") return wizard(index);
    if (mode == "rc") return showRc(index, radio_config, quad_config);
    return calibrate(index, seconds);
  } catch (const fdt::RadioError& e) {
    std::cerr << "radio error: " << e.what() << "\n";
    return 1;
  } catch (const fdt::ConfigError& e) {
    std::cerr << "config error: " << e.what() << "\n";
    return 1;
  }
}
