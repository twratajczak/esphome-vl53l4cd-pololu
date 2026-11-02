#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace vl53l4cd_pololu {

struct RangingData {
  uint8_t range_status;
  uint8_t number_of_spad;
  uint16_t signal_rate_kcps;
  uint16_t ambient_rate_kcps;
  uint16_t sigma_mm;
  uint16_t range_mm;
  uint16_t signal_per_spad_kcps;
  uint16_t ambient_per_spad_kcps;
};

class VL53L4CDPololuComponent : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_distance_sensor(sensor::Sensor *s) { this->distance_sensor_ = s; }
  void set_range_status_sensor(sensor::Sensor *s) { this->range_status_sensor_ = s; }
  void set_signal_rate_sensor(sensor::Sensor *s) { this->signal_rate_sensor_ = s; }
  void set_ambient_rate_sensor(sensor::Sensor *s) { this->ambient_rate_sensor_ = s; }
  void set_frames_sensor(sensor::Sensor *s) { this->frames_sensor_ = s; }
  void set_recovery_count_sensor(sensor::Sensor *s) { this->recovery_count_sensor_ = s; }

  void set_timing_budget(uint8_t budget_ms) { this->timing_budget_ms_ = budget_ms; }
  void set_inter_measurement(uint32_t inter_ms) { this->inter_measurement_ms_ = inter_ms; }
  void set_update_interval(uint32_t interval_ms) { this->update_interval_ms_ = interval_ms; }

 protected:
  bool init_sensor_();
  bool set_range_timing_(uint8_t timing_budget_ms, uint32_t inter_measurement_ms);
  void start_continuous_();
  void stop_continuous_();
  bool data_ready_();
  void clear_interrupt_();
  uint16_t read_measurement_();
  void read_results_();
  void perform_freeze_recovery_();

  bool write_reg_(uint16_t reg, uint8_t value);
  bool write_reg_16bit_(uint16_t reg, uint16_t value);
  bool write_reg_32bit_(uint16_t reg, uint32_t value);
  uint8_t read_reg_(uint16_t reg);
  uint16_t read_reg_16bit_(uint16_t reg);
  uint32_t read_reg_32bit_(uint16_t reg);

  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *range_status_sensor_{nullptr};
  sensor::Sensor *signal_rate_sensor_{nullptr};
  sensor::Sensor *ambient_rate_sensor_{nullptr};
  sensor::Sensor *frames_sensor_{nullptr};
  sensor::Sensor *recovery_count_sensor_{nullptr};

  uint8_t timing_budget_ms_{50};
  uint32_t inter_measurement_ms_{0}; // 0 = continuous mode
  uint32_t update_interval_ms_{60000};

  RangingData ranging_data_;
  uint32_t last_publish_time_{0};
  uint32_t last_frame_time_{0};
  uint32_t total_frames_{0};
  uint32_t total_recoveries_{0};
  bool sensor_initialized_{false};
};

}  // namespace vl53l4cd_pololu
}  // namespace esphome

