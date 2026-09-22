// Turning a three-axis sensor readback into "which axis responded, and with
// which sign".
//
// This is the step that converts a measurement into the axis/sign table in
// docs/sitl_interface.md section 5, which then becomes a conversion at the
// SITL boundary. CLAUDE.md calls axis sign and index the likeliest source of
// bugs in this project, so it lives here rather than inside the probe's
// translation unit -- a helper that decides the mapping needs a test.
#pragma once

#include <array>
#include <cstdint>
#include <cstdlib>
#include <string>

namespace fdt::probe {

inline const char* axisName(int i) { return i == 0 ? "X" : (i == 1 ? "Y" : "Z"); }

/// Which axis responded, and whether the answer is unambiguous.
struct AxisHit {
  int axis = -1;  ///< index of the largest |value| above the threshold, or -1
  int over = 0;   ///< how many axes exceeded the threshold
};

/// The axis of `v` with the largest magnitude strictly above `threshold`.
///
/// Deliberately reports `over` as well as `axis`. An earlier version kept the
/// LAST axis above the threshold rather than the largest, so when two axes
/// responded -- cross-coupling, a residual from the previous sweep step, or a
/// genuine axis mix -- it named whichever had the higher index and discarded
/// the other silently. A caller transcribing that into a conversion would have
/// got a wrong mapping with no warning.
inline AxisHit strongestAxis(const std::array<int16_t, 3>& v, int threshold) {
  AxisHit hit;
  int best = threshold;
  for (int i = 0; i < 3; ++i) {
    const int magnitude = std::abs(static_cast<int>(v[static_cast<size_t>(i)]));
    if (magnitude <= threshold) continue;
    ++hit.over;
    if (magnitude > best) {
      best = magnitude;
      hit.axis = i;
    }
  }
  return hit;
}

/// One row of the sweep table: which Betaflight axis the excited fdm axis
/// landed on, its sign, and a loud marker when the row is not trustworthy.
inline std::string interpret(int fdm_axis, const std::array<int16_t, 3>& v, int threshold) {
  const AxisHit hit = strongestAxis(v, threshold);
  if (hit.axis < 0) return "NO RESPONSE";

  const int16_t value = v[static_cast<size_t>(hit.axis)];
  std::string note = std::string("fdm ") + axisName(fdm_axis) + " -> BF " + axisName(hit.axis) +
                     (value > 0 ? " (same sign)" : " (NEGATED)");
  if (hit.over > 1) {
    note += "  [AMBIGUOUS: " + std::to_string(hit.over) +
            " axes responded, do not transcribe this row]";
  }
  return note;
}

}  // namespace fdt::probe
