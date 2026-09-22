#include "fdt/battery.hpp"

#include <algorithm>

namespace fdt {

BatteryModel::BatteryModel(const Battery& config) : config_(config) { reset(); }

void BatteryModel::reset() {
  soc_ = std::clamp(config_.initial_soc.value, 0.0, 1.0);
  current_ = 0.0;
}

double BatteryModel::openCircuitVoltage() const { return config_.packOcv(soc_); }

double BatteryModel::terminalVoltage() const {
  return std::max(0.0, openCircuitVoltage() - current_ * config_.internal_resistance.value);
}

void BatteryModel::step(double dt, double current) {
  current_ = current;
  const double amp_hours = current * dt / 3600.0;
  soc_ = std::clamp(soc_ - amp_hours / config_.capacity.value, 0.0, 1.0);
}

}  // namespace fdt
