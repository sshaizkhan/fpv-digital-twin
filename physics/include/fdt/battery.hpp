// Battery: open-circuit voltage curve plus internal resistance, so that a
// punch-out sags the pack and limits the voltage the motors can see.
#pragma once

#include "fdt/quad_config.hpp"

namespace fdt {

class BatteryModel {
 public:
  explicit BatteryModel(const Battery& config);

  /// Pack voltage with no load, at the current state of charge.
  double openCircuitVoltage() const;

  /// Pack voltage actually available to the motors: OCV minus I*R for the
  /// current draw most recently passed to step(). The sag is evaluated with
  /// the previous step's current rather than solved simultaneously with the
  /// motor model; at 1-2 kHz the lag is far below anything measurable.
  double terminalVoltage() const;

  double stateOfCharge() const { return soc_; }
  double current() const { return current_; }

  /// Draw `current` amps for `dt` seconds. State of charge is clamped to
  /// [0, 1]; the model does not simulate a pack being damaged by over-draw.
  void step(double dt, double current);

  void reset();

 private:
  Battery config_;
  double soc_ = 1.0;
  double current_ = 0.0;
};

}  // namespace fdt
