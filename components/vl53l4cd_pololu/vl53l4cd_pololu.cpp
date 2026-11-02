#include "vl53l4cd_pololu.h"
#include "esphome/core/log.h"

namespace esphome {
namespace vl53l4cd_pololu {

static const char *const TAG = "vl53l4cd_pololu";

// VL53L4CD Hardware Register Addresses
static const uint16_t IDENTIFICATION__MODEL_ID = 0x010F;
static const uint16_t FIRMWARE__SYSTEM_STATUS = 0x00E5;
static const uint16_t INTERMEASUREMENT_MS = 0x006C;
static const uint16_t SYSTEM_START = 0x0087;
static const uint16_t RESULT__OSC_CALIBRATE_VAL = 0x00DE;
static const uint16_t RANGE_CONFIG_A = 0x005E;
static const uint16_t RANGE_CONFIG_B = 0x0060;
static const uint16_t VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
static const uint16_t RESULT__RANGE_STATUS = 0x0089;

void VL53L4CDPololuComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up VL53L4CD...");
  if (this->init_sensor_()) {
    ESP_LOGI(TAG, "VL53L4CD initialized successfully.");
    this->sensor_initialized_ = true;
    this->last_frame_time_ = millis();
    this->start_continuous_();
  } else {
    ESP_LOGE(TAG, "Failed to initialize VL53L4CD! Will retry via freeze recovery loop.");
  }
}

void VL53L4CDPololuComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "VL53L4CD (Pololu Port):");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  Timing Budget: %u ms", this->timing_budget_ms_);
  ESP_LOGCONFIG(TAG, "  Inter-measurement Period: %u ms", this->inter_measurement_ms_);
  ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", this->update_interval_ms_);
  LOG_SENSOR("  ", "Distance Sensor", this->distance_sensor_);
  LOG_SENSOR("  ", "Range Status Sensor", this->range_status_sensor_);
  LOG_SENSOR("  ", "Signal Rate Sensor", this->signal_rate_sensor_);
  LOG_SENSOR("  ", "Ambient Rate Sensor", this->ambient_rate_sensor_);
  LOG_SENSOR("  ", "Frames Sensor", this->frames_sensor_);
  LOG_SENSOR("  ", "Recovery Count Sensor", this->recovery_count_sensor_);
}

void VL53L4CDPololuComponent::loop() {
  uint32_t now = millis();

  if (this->sensor_initialized_ && this->data_ready_()) {
    this->clear_interrupt_();
    this->read_results_();
    this->total_frames_++;
    this->last_frame_time_ = now;
  }

  if (this->sensor_initialized_ && (now - this->last_publish_time_ >= this->update_interval_ms_)) {
    this->last_publish_time_ = now;

    if (this->distance_sensor_ != nullptr) this->distance_sensor_->publish_state(this->ranging_data_.range_mm);
    if (this->range_status_sensor_ != nullptr) this->range_status_sensor_->publish_state(this->ranging_data_.range_status);
    if (this->signal_rate_sensor_ != nullptr) this->signal_rate_sensor_->publish_state(this->ranging_data_.signal_rate_kcps);
    if (this->ambient_rate_sensor_ != nullptr) this->ambient_rate_sensor_->publish_state(this->ranging_data_.ambient_rate_kcps);
    if (this->frames_sensor_ != nullptr) this->frames_sensor_->publish_state(this->total_frames_);
    if (this->recovery_count_sensor_ != nullptr) this->recovery_count_sensor_->publish_state(this->total_recoveries_);
  }

  if (now - this->last_frame_time_ > 5000) {
    this->perform_freeze_recovery_();
  }
}

bool VL53L4CDPololuComponent::init_sensor_() {

  // check model ID and module type registers (values specified in datasheet)
  if (readReg16Bit(IDENTIFICATION__MODEL_ID) != 0xEBAA) { return false; }

  // from VL53L4CD_SensorInit()

  // "Wait for boot"
  // check last_status in case we get a NACK to try to deal with it correctly
  startTimeout();
  while (readReg(FIRMWARE__SYSTEM_STATUS) != 0x3 || last_status != 0)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return false;
    }
  }

  // "Load default configuration"

  // "0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch"
  writeReg(0x2D, fast_mode_plus ? 0x12 : 0x00);

  // "0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)"
  writeReg(0x2E, io_2v8 ? 0x01 : 0x00);

  // "0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)"
  writeReg(0x2F, io_2v8 ? 0x01 : 0x00);

  // rest of default configuration copied from VL53L4CD_api.c (comments are also
  // copied verbatim and refer to ULD functions), but starting at 0x30 instead
  // of 0x2D since we handle the first 3 regs above based on this function's
  // args
  static const uint8_t VL53L4CD_DEFAULT_CONFIGURATION[] PROGMEM = {
    0x11, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low
    (bits 3:0 must be 0x1), use SetInterruptPolarity() */
    0x02, /* 0x31 : bit 1 = interrupt depending on the polarity,
    use CheckForDataReady() */
    0x00, /* 0x32 : not user-modifiable */
    0x02, /* 0x33 : not user-modifiable */
    0x08, /* 0x34 : not user-modifiable */
    0x00, /* 0x35 : not user-modifiable */
    0x08, /* 0x36 : not user-modifiable */
    0x10, /* 0x37 : not user-modifiable */
    0x01, /* 0x38 : not user-modifiable */
    0x01, /* 0x39 : not user-modifiable */
    0x00, /* 0x3a : not user-modifiable */
    0x00, /* 0x3b : not user-modifiable */
    0x00, /* 0x3c : not user-modifiable */
    0x00, /* 0x3d : not user-modifiable */
    0xff, /* 0x3e : not user-modifiable */
    0x00, /* 0x3f : not user-modifiable */
    0x0F, /* 0x40 : not user-modifiable */
    0x00, /* 0x41 : not user-modifiable */
    0x00, /* 0x42 : not user-modifiable */
    0x00, /* 0x43 : not user-modifiable */
    0x00, /* 0x44 : not user-modifiable */
    0x00, /* 0x45 : not user-modifiable */
    0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high,
    2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
    0x0b, /* 0x47 : not user-modifiable */
    0x00, /* 0x48 : not user-modifiable */
    0x00, /* 0x49 : not user-modifiable */
    0x02, /* 0x4a : not user-modifiable */
    0x14, /* 0x4b : not user-modifiable */
    0x21, /* 0x4c : not user-modifiable */
    0x00, /* 0x4d : not user-modifiable */
    0x00, /* 0x4e : not user-modifiable */
    0x05, /* 0x4f : not user-modifiable */
    0x00, /* 0x50 : not user-modifiable */
    0x00, /* 0x51 : not user-modifiable */
    0x00, /* 0x52 : not user-modifiable */
    0x00, /* 0x53 : not user-modifiable */
    0xc8, /* 0x54 : not user-modifiable */
    0x00, /* 0x55 : not user-modifiable */
    0x00, /* 0x56 : not user-modifiable */
    0x38, /* 0x57 : not user-modifiable */
    0xff, /* 0x58 : not user-modifiable */
    0x01, /* 0x59 : not user-modifiable */
    0x00, /* 0x5a : not user-modifiable */
    0x08, /* 0x5b : not user-modifiable */
    0x00, /* 0x5c : not user-modifiable */
    0x00, /* 0x5d : not user-modifiable */
    0x01, /* 0x5e : not user-modifiable */
    0xcc, /* 0x5f : not user-modifiable */
    0x07, /* 0x60 : not user-modifiable */
    0x01, /* 0x61 : not user-modifiable */
    0xf1, /* 0x62 : not user-modifiable */
    0x05, /* 0x63 : not user-modifiable */
    0x00, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB),
    use SetSigmaThreshold(), default value 90 mm  */
    0xa0, /* 0x65 : Sigma threshold LSB */
    0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB),
    use SetSignalThreshold() */
    0x80, /* 0x67 : Min count Rate LSB */
    0x08, /* 0x68 : not user-modifiable */
    0x38, /* 0x69 : not user-modifiable */
    0x00, /* 0x6a : not user-modifiable */
    0x00, /* 0x6b : not user-modifiable */
    0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register,
    use SetIntermeasurementInMs() */
    0x00, /* 0x6d : Intermeasurement period */
    0x0f, /* 0x6e : Intermeasurement period */
    0x89, /* 0x6f : Intermeasurement period LSB */
    0x00, /* 0x70 : not user-modifiable */
    0x00, /* 0x71 : not user-modifiable */
    0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB),
    use SetD:tanceThreshold() */
    0x00, /* 0x73 : distance threshold high LSB */
    0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB),
    use SetD:tanceThreshold() */
    0x00, /* 0x75 : distance threshold low LSB */
    0x00, /* 0x76 : not user-modifiable */
    0x01, /* 0x77 : not user-modifiable */
    0x07, /* 0x78 : not user-modifiable */
    0x05, /* 0x79 : not user-modifiable */
    0x06, /* 0x7a : not user-modifiable */
    0x06, /* 0x7b : not user-modifiable */
    0x00, /* 0x7c : not user-modifiable */
    0x00, /* 0x7d : not user-modifiable */
    0x02, /* 0x7e : not user-modifiable */
    0xc7, /* 0x7f : not user-modifiable */
    0xff, /* 0x80 : not user-modifiable */
    0x9B, /* 0x81 : not user-modifiable */
    0x00, /* 0x82 : not user-modifiable */
    0x00, /* 0x83 : not user-modifiable */
    0x00, /* 0x84 : not user-modifiable */
    0x01, /* 0x85 : not user-modifiable */
    0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
    0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(),
    If you want an automatic start after VL53L4CD_init() call,
      put 0x40 in location 0x87 */
  };

  // Load the rest of the default config (0x30-0x87) with a few block writes.
  // The Wire library for AVR uses a 32-byte tx buffer, and 2 bytes are needed
  // for the starting register address of each write, so we can write 30
  // registers at a time.
  const uint8_t block_size = 30;
  for (uint8_t start_reg = 0x30; start_reg <= 0x87; start_reg += block_size)
  {
    bus->beginTransmission(address);
    bus->write(0);    // reg high byte
    bus->write(start_reg); // reg low byte
    for (uint8_t reg = start_reg; (reg < start_reg + block_size) && (reg <= 0x87); reg++)
    {
      bus->write(pgm_read_byte(VL53L4CD_DEFAULT_CONFIGURATION + reg - 0x30));
    }
    last_status = bus->endTransmission();
    if (last_status) { return false; }
  }

  // "Start VHV"
  writeReg(SYSTEM_START, 0x40);
  startTimeout();
  while (!dataReady())
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return false;
    }
  }

  clearInterrupt();
  stopContinuous();
  writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
  writeReg(0x0B, 0);
  writeReg16Bit(0x24, 0x500);

  return this->set_range_timing_(this->timing_budget_ms_, this->inter_measurement_ms_);
}

bool VL53L4CDPololuComponent::set_range_timing_(uint8_t timing_budget_ms, uint32_t inter_measurement_ms) {
  // timing budget must be 10 to 200 ms
  if (timing_budget_ms < 10 || timing_budget_ms > 200)
  {
    return false;
  }

  // inter-measurement period must be either 0 or > timing budget
  // UM2931 lists a max of 5000 ms, but the ULD doesn't seem to enforce this
  if (inter_measurement_ms != 0 && inter_measurement_ms <= timing_budget_ms)
  {
    return false;
  }

  uint16_t osc_frequency = readReg16Bit(0x06);
  if (osc_frequency == 0)
  {
    return false;
  }

  uint32_t timing_budget_us = (uint32_t)timing_budget_ms * 1000;
  uint32_t macro_period_us = ((uint32_t)2304 * (0x40000000 / osc_frequency)) >> 6;

  if (inter_measurement_ms == 0)
  {
    // continuous mode
    writeReg32Bit(INTERMEASUREMENT_MS, 0);

    timing_budget_us -= 2500;
  }
  else
  {
    // autonomous low power mode
    uint16_t clock_pll = readReg16Bit(RESULT__OSC_CALIBRATE_VAL) & 0x3FF;
    // note that this value of 1.055 (inter_measurement_factor) from
    // VL53L4CD_SetRangeTiming is different than the value of 1.065
    // (clock_pll_factor) used in VL53L4CD_GetRangeTiming()
    uint32_t inter_measurement = 1.055 * inter_measurement_ms * clock_pll;
    writeReg32Bit(INTERMEASUREMENT_MS, inter_measurement);

    timing_budget_us -= 4300;
    timing_budget_us /= 2;
  }

  timing_budget_us <<= 12;

  uint32_t tmp;
  uint16_t ls_byte;
  uint8_t ms_byte;

  tmp = (macro_period_us * 16) >> 6;
  ls_byte = ((timing_budget_us + (tmp >> 1)) / tmp) - 1;
  ms_byte = 0;
  while (ls_byte > 0xFF)
  {
    ls_byte >>= 1;
    ms_byte++;
  }
  writeReg16Bit(RANGE_CONFIG_A, (uint16_t)ms_byte << 8 | ls_byte);

  tmp = (macro_period_us * 12) >> 6;
  ls_byte = ((timing_budget_us + (tmp >> 1)) / tmp) - 1;
  ms_byte = 0;
  while (ls_byte > 0xFF)
  {
    ls_byte >>= 1;
    ms_byte++;
  }
  writeReg16Bit(RANGE_CONFIG_B, (uint16_t)ms_byte << 8 | ls_byte);

  return true;
}

void VL53L4CDPololuComponent::start_continuous_() {
  if (this->read_reg_32bit_(INTERMEASUREMENT_MS) == 0) {
    this->write_reg_(SYSTEM_START, 0x21); // Continuous Mode
  } else {
    this->write_reg_(SYSTEM_START, 0x40); // Autonomous Low Power Mode
  }
}

void VL53L4CDPololuComponent::stop_continuous_() {
  this->write_reg_(SYSTEM_START, 0x80);
}

bool VL53L4CDPololuComponent::data_ready_() {
  return (this->read_reg_(0x00E1) & 0x01) == 0;
}

void VL53L4CDPololuComponent::clear_interrupt_() {
  this->write_reg_(0x0086, 0x01);
}

void VL53L4CDPololuComponent::read_results_() {
  // Mapping array to translate raw status codes down to ULD generic indicators
  static const uint8_t status_rtn[24] = {
    255, 255, 255, 5,   2,   4,   1,   7,   3,   0,   255, 255,
    9,   13,  255, 255, 255, 255, 10,  6,   255, 255, 11,  12
  };

  uint8_t buffer[15];
  // 0x0089 is the address for RESULT__RANGE_STATUS
  if (!this->read_register(0x0089, buffer, 15)) {
    ESP_LOGE("vl53l4cd_pololu", "Failed to perform 15-byte results block read over I2C!");
    return;
  }

  // Parse status code from the first byte (register 0x0089)
  uint8_t status = buffer[0] & 0x1F;
  if (status < 24) {
    status = status_rtn[status];
  }
  this->ranging_data_.range_status = status;

  // SPAD tracking counts (register 0x008C)
  this->ranging_data_.number_of_spad = buffer[3];

  // Signal and Ambient rates are returned in KCPS (Kilo Counts Per Second)
  // Shift high bytes and multiply by 8 per ST specification sheets
  this->ranging_data_.signal_rate_kcps = ((uint16_t)buffer[5] << 8 | buffer[6]) * 8;
  this->ranging_data_.ambient_rate_kcps = ((uint16_t)buffer[7] << 8 | buffer[8]) * 8;

  // Sigma estimation metric (registers 0x0092, 0x0093)
  this->ranging_data_.sigma_mm = ((uint16_t)buffer[9] << 8 | buffer[10]) / 4;

  // Final targeted distance calculation in millimeters (registers 0x0096, 0x0097)
  this->ranging_data_.range_mm = (uint16_t)buffer[13] << 8 | buffer[14];

  // Prevent divide-by-zero errors when compiling spad density parameters
  if (this->ranging_data_.number_of_spad != 0) {
    this->ranging_data_.signal_per_spad_kcps = this->ranging_data_.signal_rate_kcps / this->ranging_data_.number_of_spad;
    this->ranging_data_.ambient_per_spad_kcps = this->ranging_data_.ambient_rate_kcps / this->ranging_data_.number_of_spad;
  } else {
    this->ranging_data_.signal_per_spad_kcps = 0;
    this->ranging_data_.ambient_per_spad_kcps = 0;
  }
}

void VL53L4CDPololuComponent::perform_freeze_recovery_() {
  ESP_LOGW(TAG, "Sensor freeze detected! Executing targeted live recovery re-initialization pipeline.");
  this->total_recoveries_++;
  this->sensor_initialized_ = false;
  
  this->stop_continuous_();
  delay(10);
  
  if (this->init_sensor_()) {
    ESP_LOGI(TAG, "Sensor self-healed and returned to baseline service levels.");
    this->sensor_initialized_ = true;
    this->start_continuous_();
  } else {
    ESP_LOGE(TAG, "Self-heal attempt failed. Stalling for next evaluation window.");
  }
  this->last_frame_time_ = millis();
}

bool VL53L4CDPololuComponent::write_reg_(uint16_t reg, uint8_t value) {
  return this->write_byte(reg, value);
}

bool VL53L4CDPololuComponent::write_reg_16bit_(uint16_t reg, uint16_t value) {
  return this->write_byte_16(reg, value);
}

bool VL53L4CDPololuComponent::write_reg_32bit_(uint16_t reg, uint32_t value) {
  uint8_t buffer[4];
  buffer[0] = (uint8_t)(value >> 24);
  buffer[1] = (uint8_t)(value >> 16);
  buffer[2] = (uint8_t)(value >> 8);
  buffer[3] = (uint8_t)(value);
  return this->write_register(reg, buffer, 4);
}

uint8_t VL53L4CDPololuComponent::read_reg_(uint16_t reg) {
  uint8_t value = 0;
  this->read_byte(reg, &value);
  return value;
}

uint16_t VL53L4CDPololuComponent::read_reg_16bit_(uint16_t reg) {
  uint16_t value = 0;
  this->read_byte_16(reg, &value);
  return value;
}

uint32_t VL53L4CDPololuComponent::read_reg_32bit_(uint16_t reg) {
  uint8_t buffer[4] = {0, 0, 0, 0};
  if (!this->read_register(reg, buffer, 4)) {
    return 0; // Return 0 if the I2C transfer fails
  }
  return ((uint32_t)buffer[0] << 24) | 
         ((uint32_t)buffer[1] << 16) | 
         ((uint32_t)buffer[2] << 8)  | 
         buffer[3];
}

}  // namespace vl53l4cd_pololu
}  // namespace esphome

