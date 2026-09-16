#include "sen6x.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cinttypes>
#include <cmath>
#include <cstring>

namespace esphome::sen6x {

static const char *const TAG = "sen6x";

static constexpr uint8_t POLL_RETRIES = 24;     // 24 attempts
static constexpr uint32_t I2C_READ_DELAY = 20;  // 20 ms to wait for I2C read to complete
static constexpr uint32_t CMD_EXEC_DELAY = 20;  // execution time of set commands (datasheet section 4.8)
static constexpr uint32_t POLL_INTERVAL = 50;   // 50 ms between poll attempts
// Numeric timeout IDs. Each chain is sequential, so only one timeout per ID is active at a time.
static constexpr uint32_t TIMEOUT_POLL = 1;
static constexpr uint32_t TIMEOUT_SETUP_STEP = 2;
static constexpr uint32_t TIMEOUT_STARTUP = 3;
// The VOC state sequences get their own ID: update() cancels TIMEOUT_POLL on every cycle, which
// would otherwise abort a stop/write/start dance halfway through and leave the device idle.
static constexpr uint32_t TIMEOUT_ACTION = 4;

// Post-command wait times from the datasheet: start execution (4.8.1), stop execution (4.8.2),
// fan cleaning (4.8.22), SHT heater (4.8.23), CO2 conditioning after start on SEN63C/SEN69C (4.8.1),
// device reset (4.8.21)
static constexpr uint32_t START_MEASUREMENT_DELAY = 50;
static constexpr uint32_t STOP_MEASUREMENT_DELAY = 1400;
static constexpr uint32_t FAN_CLEANING_DELAY = 10000;
static constexpr uint32_t SHT_HEATER_DELAY = 20000;
static constexpr uint32_t CO2_CONDITIONING_DELAY = 24000;
static constexpr uint32_t DEVICE_RESET_DELAY = 1200;
static constexpr uint16_t SEN6X_CMD_GET_DATA_READY_STATUS = 0x0202;
static constexpr uint16_t SEN6X_CMD_GET_FIRMWARE_VERSION = 0xD100;
static constexpr uint16_t SEN6X_CMD_GET_PRODUCT_NAME = 0xD014;
static constexpr uint16_t SEN6X_CMD_GET_SERIAL_NUMBER = 0xD033;
static constexpr uint16_t SEN6X_CMD_READ_AND_CLEAR_DEVICE_STATUS = 0xD210;

static constexpr uint16_t SEN6X_CMD_READ_MEASUREMENT = 0x0300;  // SEN66 only!
static constexpr uint16_t SEN6X_CMD_READ_MEASUREMENT_SEN62 = 0x04A3;
static constexpr uint16_t SEN6X_CMD_READ_MEASUREMENT_SEN63C = 0x0471;
static constexpr uint16_t SEN6X_CMD_READ_MEASUREMENT_SEN65 = 0x0446;
static constexpr uint16_t SEN6X_CMD_READ_MEASUREMENT_SEN68 = 0x0467;
static constexpr uint16_t SEN6X_CMD_READ_MEASUREMENT_SEN69C = 0x04B5;

static constexpr uint16_t SEN6X_CMD_READ_NUMBER_CONCENTRATION = 0x0316;
static constexpr uint16_t SEN6X_CMD_START_MEASUREMENTS = 0x0021;
static constexpr uint16_t SEN6X_CMD_STOP_MEASUREMENTS = 0x0104;
static constexpr uint16_t SEN6X_CMD_START_FAN_CLEANING = 0x5607;
static constexpr uint16_t SEN6X_CMD_ACTIVATE_SHT_HEATER = 0x6765;
static constexpr uint16_t SEN6X_CMD_RESET = 0xD304;
static constexpr uint16_t SEN6X_CMD_VOC_ALGORITHM_TUNING = 0x60D0;
static constexpr uint16_t SEN6X_CMD_NOX_ALGORITHM_TUNING = 0x60E1;
static constexpr uint16_t SEN6X_CMD_TEMPERATURE_COMPENSATION = 0x60B2;
static constexpr uint16_t SEN6X_CMD_RHT_ACCELERATION_MODE = 0x6100;
static constexpr uint16_t SEN6X_CMD_CO2_AUTOMATIC_SELF_CAL = 0x6711;
static constexpr uint16_t SEN6X_CMD_AMBIENT_PRESSURE = 0x6720;
static constexpr uint16_t SEN6X_CMD_SENSOR_ALTITUDE = 0x6736;
static constexpr uint16_t SEN6X_CMD_VOC_ALGORITHM_STATE = 0x6181;

// Wait windows are stored as (start, duration) rather than as a deadline. A deadline
// compared with `(int32_t) (millis() - deadline) < 0` inverts once the elapsed time passes
// 2^31 ms, so after ~24.8 days of uptime every command would read as still waiting and the
// actions would reject forever. Comparing elapsed against the duration keeps the unsigned
// difference bounded by the window itself, and clearing an expired window means a later
// millis() rollover cannot re-open it.
static void start_wait_window(uint32_t &started_at, uint32_t &duration, uint32_t duration_ms) {
  started_at = millis();
  duration = duration_ms;
}

static bool wait_window_active(uint32_t &started_at, uint32_t &duration) {
  if (duration == 0)
    return false;
  if (millis() - started_at < duration)
    return true;
  duration = 0;
  return false;
}

static inline void set_read_command_and_words(SEN6XComponent::Sen6xType type, uint16_t &read_cmd, uint8_t &read_words) {
  read_cmd = SEN6X_CMD_READ_MEASUREMENT;
  read_words = 9;
  switch (type) {
    case SEN6XComponent::SEN62:
      read_cmd = SEN6X_CMD_READ_MEASUREMENT_SEN62;
      read_words = 6;
      break;
    case SEN6XComponent::SEN63C:
      read_cmd = SEN6X_CMD_READ_MEASUREMENT_SEN63C;
      read_words = 7;
      break;
    case SEN6XComponent::SEN65:
      read_cmd = SEN6X_CMD_READ_MEASUREMENT_SEN65;
      read_words = 8;
      break;
    case SEN6XComponent::SEN66:
      read_cmd = SEN6X_CMD_READ_MEASUREMENT;
      read_words = 9;
      break;
    case SEN6XComponent::SEN68:
      read_cmd = SEN6X_CMD_READ_MEASUREMENT_SEN68;
      read_words = 9;
      break;
    case SEN6XComponent::SEN69C:
      read_cmd = SEN6X_CMD_READ_MEASUREMENT_SEN69C;
      read_words = 10;
      break;
    default:
      break;
  }
}

void SEN6XComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up sen6x...");

  // the sensor needs 100 ms to enter the idle state
  this->set_timeout(100, [this]() {
    // A soft reboot of the host (OTA, watchdog, reset button) leaves the sensor powered and still
    // measuring, and the reset below is accepted in idle mode only (datasheet section 4.8.21) — as
    // is most of the configuration that follows it. Stopping first costs one settling window on a
    // warm boot and nothing on a cold one, where an idle device has nothing to stop.
    const bool stopped = this->write_command(SEN6X_CMD_STOP_MEASUREMENTS);
    this->set_timeout(stopped ? STOP_MEASUREMENT_DELAY : 0, [this]() {
      // Reset the sensor to ensure a clean state regardless of prior commands or power issues
      if (!this->write_command(SEN6X_CMD_RESET)) {
        ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
        this->mark_failed(LOG_STR(ESP_LOG_MSG_COMM_FAIL));
        return;
      }

      // The reset runs for its full execution time before the device accepts anything again (4.8.21)
      this->set_timeout(DEVICE_RESET_DELAY, [this]() {
        // Step 1: Read serial number (~25ms with I2C delay)
        uint16_t raw_serial_number[16];
        if (!this->get_register(SEN6X_CMD_GET_SERIAL_NUMBER, raw_serial_number, 16, 20)) {
          ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
          this->mark_failed(LOG_STR(ESP_LOG_MSG_COMM_FAIL));
          return;
        }
        this->serial_number_ = SEN6XComponent::sensirion_convert_to_string_in_place(raw_serial_number, 16);
        ESP_LOGI(TAG, "Serial number: %s", this->serial_number_.c_str());

        // Step 2: Read product name in next loop iteration
        this->set_timeout(0, [this]() {
          uint16_t raw_product_name[16];
          if (!this->get_register(SEN6X_CMD_GET_PRODUCT_NAME, raw_product_name, 16, 20)) {
            ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
            this->mark_failed(LOG_STR(ESP_LOG_MSG_COMM_FAIL));
            return;
          }

          this->product_name_ = SEN6XComponent::sensirion_convert_to_string_in_place(raw_product_name, 16);

          Sen6xType inferred_type = this->infer_type_from_product_name_(this->product_name_);
          if (this->sen6x_type_ == UNKNOWN) {
            this->sen6x_type_ = inferred_type;
            if (inferred_type == UNKNOWN) {
              ESP_LOGE(TAG, "Unknown product '%s'", this->product_name_.c_str());
              this->mark_failed();
              return;
            }
            ESP_LOGD(TAG, "Type inferred from product: %s", this->product_name_.c_str());
          } else if (this->sen6x_type_ != inferred_type && inferred_type != UNKNOWN) {
            ESP_LOGW(TAG, "Configured type (used) mismatches product '%s'", this->product_name_.c_str());
          }
          ESP_LOGI(TAG, "Product: %s", this->product_name_.c_str());

          // Validate configured sensors against detected type and disable unsupported ones
          const bool has_voc_nox = (this->sen6x_type_ == SEN65 || this->sen6x_type_ == SEN66 ||
                                    this->sen6x_type_ == SEN68 || this->sen6x_type_ == SEN69C);
          // Tracked separately from voc_sensor_: the device runs the VOC engine whether or not the
          // index is published, so saving and restoring its state stays available either way
          this->voc_supported_ = has_voc_nox;
          const bool has_co2 =
              (this->sen6x_type_ == SEN63C || this->sen6x_type_ == SEN66 || this->sen6x_type_ == SEN69C);
          const bool has_hcho = (this->sen6x_type_ == SEN68 || this->sen6x_type_ == SEN69C);
          if (this->voc_sensor_ && !has_voc_nox) {
            ESP_LOGE(TAG, "VOC requires SEN65, SEN66, SEN68, or SEN69C");
            this->voc_sensor_ = nullptr;
          }
          if (this->nox_sensor_ && !has_voc_nox) {
            ESP_LOGE(TAG, "NOx requires SEN65, SEN66, SEN68, or SEN69C");
            this->nox_sensor_ = nullptr;
          }
          if (this->co2_sensor_ && !has_co2) {
            ESP_LOGE(TAG, "CO2 requires SEN63C, SEN66, or SEN69C");
            this->co2_sensor_ = nullptr;
          }
          if (this->hcho_sensor_ && !has_hcho) {
            ESP_LOGE(TAG, "Formaldehyde requires SEN68 or SEN69C");
            this->hcho_sensor_ = nullptr;
          }
#ifdef USE_BINARY_SENSOR
          if (this->gas_error_binary_sensor_ && !has_voc_nox) {
            ESP_LOGE(TAG, "VOC requires SEN65, SEN66, SEN68, or SEN69C");
            this->gas_error_binary_sensor_ = nullptr;
          }
          if (this->co2_error_binary_sensor_ && !has_co2) {
            ESP_LOGE(TAG, "CO2 requires SEN63C, SEN66, or SEN69C");
            this->co2_error_binary_sensor_ = nullptr;
          }
          if (this->hcho_error_binary_sensor_ && !has_hcho) {
            ESP_LOGE(TAG, "Formaldehyde requires SEN68 or SEN69C");
            this->hcho_error_binary_sensor_ = nullptr;
          }
          this->has_status_sensors_ =
              this->fan_error_binary_sensor_ != nullptr || this->fan_speed_warning_binary_sensor_ != nullptr ||
              this->rht_error_binary_sensor_ != nullptr || this->gas_error_binary_sensor_ != nullptr ||
              this->co2_error_binary_sensor_ != nullptr || this->hcho_error_binary_sensor_ != nullptr ||
              this->pm_error_binary_sensor_ != nullptr;
#endif

          // Step 3: Read firmware version and start measurements in next loop iteration
          this->set_timeout(0, [this]() {
            uint16_t raw_firmware_version = 0;
            if (!this->get_register(SEN6X_CMD_GET_FIRMWARE_VERSION, raw_firmware_version, 20)) {
              ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
              this->mark_failed(LOG_STR(ESP_LOG_MSG_COMM_FAIL));
              return;
            }
            this->firmware_version_major_ = (raw_firmware_version >> 8) & 0xFF;
            this->firmware_version_minor_ = raw_firmware_version & 0xFF;
            ESP_LOGI(TAG, "Firmware: %u.%u", this->firmware_version_major_, this->firmware_version_minor_);

            // Step 4: write configuration commands one at a time, then start measurements.
            // Delay the first step so it doesn't run in the same loop tick as the read above.
            this->set_timeout(TIMEOUT_SETUP_STEP, CMD_EXEC_DELAY, [this]() { this->run_next_setup_step_(); });
          });
        });
      });
    });
  });
}

// One configuration write per invocation, spaced by CMD_EXEC_DELAY. Cases without a
// configured value fall through; each taken case must advance setup_step_index_ so the
// next invocation resumes at the following step. These writes are optional, so a failure
// only warns and the chain continues to the mandatory start-measurements write.
void SEN6XComponent::run_next_setup_step_() {
  switch (this->setup_step_index_) {
    // Tuning writes are skipped when setup() disabled the sensor for this variant
    case 0:
      this->setup_step_index_++;
      if (this->voc_sensor_ != nullptr && this->voc_tuning_params_.has_value()) {
        this->write_tuning_parameters_(SEN6X_CMD_VOC_ALGORITHM_TUNING, this->voc_tuning_params_.value());
        break;
      }
      [[fallthrough]];
    case 1:
      this->setup_step_index_++;
      if (this->nox_sensor_ != nullptr && this->nox_tuning_params_.has_value()) {
        this->write_tuning_parameters_(SEN6X_CMD_NOX_ALGORITHM_TUNING, this->nox_tuning_params_.value());
        break;
      }
      [[fallthrough]];
    case 2:
      this->setup_step_index_++;
      if (this->temperature_compensation_.has_value()) {
        this->write_temperature_compensation_(this->temperature_compensation_.value());
        break;
      }
      [[fallthrough]];
    case 3:
      this->setup_step_index_++;
      if (this->temperature_acceleration_.has_value()) {
        this->write_temperature_acceleration_(this->temperature_acceleration_.value());
        break;
      }
      [[fallthrough]];
    // CO2 settings are skipped when setup() disabled the CO2 sensor for this variant
    case 4:
      this->setup_step_index_++;
      if (this->co2_sensor_ != nullptr && this->co2_asc_.has_value()) {
        this->write_setup_register_(SEN6X_CMD_CO2_AUTOMATIC_SELF_CAL, this->co2_asc_.value() ? 1 : 0);
        break;
      }
      [[fallthrough]];
    case 5:
      this->setup_step_index_++;
      if (this->co2_sensor_ != nullptr && this->altitude_compensation_.has_value()) {
        this->write_setup_register_(SEN6X_CMD_SENSOR_ALTITUDE, this->altitude_compensation_.value());
        break;
      }
      [[fallthrough]];
    case 6:
      this->setup_step_index_++;
      if (this->co2_sensor_ != nullptr && this->ambient_pressure_.has_value()) {
        if (this->write_setup_register_(SEN6X_CMD_AMBIENT_PRESSURE, this->ambient_pressure_.value()))
          this->last_ambient_pressure_ = this->ambient_pressure_;
        break;
      }
      [[fallthrough]];
    // Last step before the measurement starts, and the only place it can go: a VOC state write is
    // accepted in idle mode only and is applied once, when the next measurement starts (4.8.28)
    case 7:
      this->setup_step_index_++;
      if (this->load_voc_state_and_restore_())
        break;
      [[fallthrough]];
    default:
      this->finish_setup_();
      return;
  }
  this->set_timeout(TIMEOUT_SETUP_STEP, CMD_EXEC_DELAY, [this]() { this->run_next_setup_step_(); });
}

void SEN6XComponent::set_temperature_compensation(float offset, float normalized_offset_slope, uint16_t time_constant) {
  this->temperature_compensation_ =
      TemperatureCompensation{static_cast<int16_t>(lroundf(offset * 200.0f)),
                              static_cast<int16_t>(lroundf(normalized_offset_slope * 10000.0f)), time_constant};
}

void SEN6XComponent::set_temperature_acceleration(float k, float p, float t1, float t2) {
  this->temperature_acceleration_ =
      TemperatureAcceleration{static_cast<uint16_t>(lroundf(k * 10.0f)), static_cast<uint16_t>(lroundf(p * 10.0f)),
                              static_cast<uint16_t>(lroundf(t1 * 10.0f)), static_cast<uint16_t>(lroundf(t2 * 10.0f))};
}

bool SEN6XComponent::write_temperature_compensation_(const TemperatureCompensation &compensation) {
  // Word 4 selects compensation slot 0; other slots are not exposed
  uint16_t params[4] = {static_cast<uint16_t>(compensation.offset),
                        static_cast<uint16_t>(compensation.normalized_offset_slope), compensation.time_constant, 0};
  return this->write_config_words_(SEN6X_CMD_TEMPERATURE_COMPENSATION, params, 4);
}

bool SEN6XComponent::write_temperature_acceleration_(const TemperatureAcceleration &acceleration) {
  uint16_t params[4] = {acceleration.k, acceleration.p, acceleration.t1, acceleration.t2};
  return this->write_config_words_(SEN6X_CMD_RHT_ACCELERATION_MODE, params, 4);
}

bool SEN6XComponent::write_setup_register_(uint16_t i2c_command, uint16_t value) {
  return this->write_config_words_(i2c_command, &value, 1);
}

void SEN6XComponent::finish_setup_() {
  if (!this->write_command(SEN6X_CMD_START_MEASUREMENTS)) {
    ESP_LOGE(TAG, "Write 0x%04X failed, error %d", SEN6X_CMD_START_MEASUREMENTS, this->last_error_);
    this->mark_failed(LOG_STR(ESP_LOG_MSG_COMM_FAIL));
    return;
  }

  this->set_timeout(TIMEOUT_STARTUP, this->startup_delay_ms_, [this]() { this->startup_complete_ = true; });
  this->initialized_ = true;
  this->measuring_ = true;
  start_wait_window(this->command_wait_started_at_, this->command_wait_ms_, START_MEASUREMENT_DELAY);
  // SEN63C/SEN69C condition the CO2 sensor for 24 s after a start; block restarts until then
  if (this->sen6x_type_ == SEN63C || this->sen6x_type_ == SEN69C) {
    start_wait_window(this->co2_restart_started_at_, this->co2_restart_ms_, CO2_CONDITIONING_DELAY);
  }
  ESP_LOGD(TAG, "Initialized");
}

// Writes one optional configuration command. A failure warns and returns false, but does
// not stop setup: the sensor still measures with that setting left at its default.
bool SEN6XComponent::write_config_words_(uint16_t i2c_command, const uint16_t *data, uint8_t len) {
  if (!this->write_command(i2c_command, data, len)) {
    ESP_LOGE(TAG, "Write 0x%04X failed, error %d", i2c_command, this->last_error_);
    this->status_set_warning();
    return false;
  }
  return true;
}

bool SEN6XComponent::write_tuning_parameters_(uint16_t i2c_command, const GasTuning &tuning) {
  uint16_t params[6] = {tuning.index_offset,
                        tuning.learning_time_offset_hours,
                        tuning.learning_time_gain_hours,
                        tuning.gating_max_duration_minutes,
                        tuning.std_initial,
                        tuning.gain_factor};
  return this->write_config_words_(i2c_command, params, 6);
}

void SEN6XComponent::dump_config() {
  ESP_LOGCONFIG(TAG,
                "sen6x:\n"
                "  Product: %s\n"
                "  Serial: %s\n"
                "  Firmware: %u.%u\n"
                "  Address: 0x%02X",
                this->product_name_.c_str(), this->serial_number_.c_str(), this->firmware_version_major_,
                this->firmware_version_minor_, this->address_);
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG,
                "  Startup delay: %" PRIu32 " ms\n"
                "  Restore VOC state on boot: %s",
                this->startup_delay_ms_, YESNO(this->restore_voc_state_on_boot_));
  if (this->temperature_compensation_.has_value()) {
    const auto &comp = this->temperature_compensation_.value();
    ESP_LOGCONFIG(TAG, "  Temperature compensation: offset=%.2f slope=%.4f time_constant=%us", comp.offset / 200.0f,
                  comp.normalized_offset_slope / 10000.0f, comp.time_constant);
  }
  if (this->temperature_acceleration_.has_value()) {
    const auto &accel = this->temperature_acceleration_.value();
    ESP_LOGCONFIG(TAG, "  Temperature acceleration: K=%.1f P=%.1f T1=%.1f T2=%.1f", accel.k / 10.0f, accel.p / 10.0f,
                  accel.t1 / 10.0f, accel.t2 / 10.0f);
  }
  // Gate on the variant, not co2_sensor_: dump_config can run before the async setup
  // chain identifies the device and disables unsupported sensors
  const bool co2_supported = this->sen6x_type_ == SEN63C || this->sen6x_type_ == SEN66 || this->sen6x_type_ == SEN69C;
  if (co2_supported) {
    if (this->co2_asc_.has_value()) {
      ESP_LOGCONFIG(TAG, "  CO2 automatic self-calibration: %s", ONOFF(this->co2_asc_.value()));
    }
    if (this->altitude_compensation_.has_value()) {
      ESP_LOGCONFIG(TAG, "  Altitude compensation: %u m", this->altitude_compensation_.value());
    }
    if (this->ambient_pressure_source_ != nullptr) {
      ESP_LOGCONFIG(TAG, "  Ambient pressure compensation source: %s",
                    this->ambient_pressure_source_->get_name().c_str());
    } else if (this->ambient_pressure_.has_value()) {
      ESP_LOGCONFIG(TAG, "  Ambient pressure compensation: %u hPa", this->ambient_pressure_.value());
    }
  }
  LOG_SENSOR("  ", "PM  1.0", this->pm_1_0_sensor_);
  LOG_SENSOR("  ", "PM  2.5", this->pm_2_5_sensor_);
  LOG_SENSOR("  ", "PM  4.0", this->pm_4_0_sensor_);
  LOG_SENSOR("  ", "PM 10.0", this->pm_10_0_sensor_);
  LOG_SENSOR("  ", "PMC  0.5", this->pmc_0_5_sensor_);
  LOG_SENSOR("  ", "PMC  1.0", this->pmc_1_0_sensor_);
  LOG_SENSOR("  ", "PMC  2.5", this->pmc_2_5_sensor_);
  LOG_SENSOR("  ", "PMC  4.0", this->pmc_4_0_sensor_);
  LOG_SENSOR("  ", "PMC 10.0", this->pmc_10_0_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
  LOG_SENSOR("  ", "VOC", this->voc_sensor_);
  LOG_SENSOR("  ", "NOx", this->nox_sensor_);
  LOG_SENSOR("  ", "HCHO", this->hcho_sensor_);
  LOG_SENSOR("  ", "CO2", this->co2_sensor_);
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Fan error", this->fan_error_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "Fan speed warning", this->fan_speed_warning_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "RH&T error", this->rht_error_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "Gas error", this->gas_error_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "CO2 error", this->co2_error_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "HCHO error", this->hcho_error_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "PM error", this->pm_error_binary_sensor_);
#endif
}

void SEN6XComponent::update() {
  if (!this->initialized_) {
    return;
  }
  if (!this->measuring_) {
    // No poll chain runs while idle, so a queued VOC state read is serviced here instead
    this->service_pending_voc_save_();
    return;
  }
  if (this->voc_sequence_active_) {
    // A VOC state read or restore owns the bus; skip this cycle rather than interleave with it
    ESP_LOGD(TAG, "VOC state command in progress, skipping poll");
    return;
  }

  // Cancel any in-flight polling from a previous update() cycle before touching the bus.
  this->cancel_timeout(TIMEOUT_POLL);

  bool wrote_pressure = false;
  if (this->ambient_pressure_source_ != nullptr && this->co2_sensor_ != nullptr) {
    wrote_pressure = this->update_ambient_pressure_compensation_(this->ambient_pressure_source_->state);
  }

  set_read_command_and_words(this->sen6x_type_, this->read_cmd_, this->read_words_);

  // Polling uses chained timeouts to guarantee each I2C operation completes
  // before the next begins. The flow is:
  //
  //   poll_data_ready_()
  //     -> write_command (data ready status)
  //     -> timeout I2C_READ_DELAY
  //       -> read_data (check ready flag)
  //       -> if not ready: timeout POLL_INTERVAL -> poll_data_ready_() (retry)
  //       -> if ready: read_measurements_()
  //                      -> write_command (read measurement)
  //                      -> timeout I2C_READ_DELAY
  //                        -> parse_and_publish_measurements_()
  //
  // All timeouts share a single ID (TIMEOUT_POLL) since only one is active
  // at a time. cancel_timeout in update() stops any in-flight chain.
  this->poll_retries_remaining_ = POLL_RETRIES;
  if (wrote_pressure) {
    // Give the pressure set command its execution time before the chain writes again
    this->set_timeout(TIMEOUT_POLL, CMD_EXEC_DELAY, [this]() { this->start_poll_chain_(); });
  } else {
    this->start_poll_chain_();
  }
}

// Entry point for one update cycle's I2C chain. With status sensors configured the status
// register is read first; that path continues into the poll chain on its own.
void SEN6XComponent::start_poll_chain_() {
#ifdef USE_BINARY_SENSOR
  if (this->has_status_sensors_) {
    this->read_device_status_();
    return;
  }
#endif
  this->poll_data_ready_();
}

void SEN6XComponent::poll_data_ready_() {
  if (this->poll_retries_remaining_ == 0) {
    this->status_set_warning();
    ESP_LOGD(TAG, "Data not ready");
    return;
  }
  ESP_LOGV(TAG, "Data ready polling attempt %u",
           static_cast<unsigned>(POLL_RETRIES - this->poll_retries_remaining_ + 1));
  this->poll_retries_remaining_--;

  if (!this->write_command(SEN6X_CMD_GET_DATA_READY_STATUS)) {
    this->status_set_warning();
    ESP_LOGD(TAG, "write data ready status error (%d)", this->last_error_);
    return;
  }

  this->set_timeout(TIMEOUT_POLL, I2C_READ_DELAY, [this]() {
    uint16_t raw_read_status;
    if (!this->read_data(&raw_read_status, 1)) {
      this->status_set_warning();
      ESP_LOGD(TAG, "read data ready status error (%d)", this->last_error_);
      return;
    }

    if ((raw_read_status & 0x0001) == 0) {
      // Not ready yet; schedule next attempt after POLL_INTERVAL.
      this->set_timeout(TIMEOUT_POLL, POLL_INTERVAL, [this]() { this->poll_data_ready_(); });
      return;
    }

    this->read_measurements_();
  });
}

void SEN6XComponent::read_measurements_() {
  if (!this->write_command(this->read_cmd_)) {
    this->status_set_warning();
    ESP_LOGD(TAG, "Read measurement failed (%d)", this->last_error_);
    return;
  }

  this->set_timeout(TIMEOUT_POLL, I2C_READ_DELAY, [this]() { this->parse_and_publish_measurements_(); });
}

void SEN6XComponent::parse_and_publish_measurements_() {
  uint16_t measurements[10];

  if (!this->read_data(measurements, this->read_words_)) {
    this->status_set_warning();
    ESP_LOGD(TAG, "Read data failed (%d)", this->last_error_);
    return;
  }
  int8_t voc_index = -1;
  int8_t nox_index = -1;
  int8_t hcho_index = -1;
  int8_t co2_index = -1;
  bool co2_uint16 = false;
  switch (this->sen6x_type_) {
    case SEN62:
      break;
    case SEN63C:
      co2_index = 6;
      break;
    case SEN65:
      voc_index = 6;
      nox_index = 7;
      break;
    case SEN66:
      voc_index = 6;
      nox_index = 7;
      co2_index = 8;
      co2_uint16 = true;
      break;
    case SEN68:
      voc_index = 6;
      nox_index = 7;
      hcho_index = 8;
      break;
    case SEN69C:
      voc_index = 6;
      nox_index = 7;
      hcho_index = 8;
      co2_index = 9;
      break;
    default:
      break;
  }

  float pm_1_0 = measurements[0] / 10.0f;
  if (measurements[0] == 0xFFFF)
    pm_1_0 = NAN;
  float pm_2_5 = measurements[1] / 10.0f;
  if (measurements[1] == 0xFFFF)
    pm_2_5 = NAN;
  float pm_4_0 = measurements[2] / 10.0f;
  if (measurements[2] == 0xFFFF)
    pm_4_0 = NAN;
  float pm_10_0 = measurements[3] / 10.0f;
  if (measurements[3] == 0xFFFF)
    pm_10_0 = NAN;
  float humidity = static_cast<int16_t>(measurements[4]) / 100.0f;
  if (measurements[4] == 0x7FFF)
    humidity = NAN;
  float temperature = static_cast<int16_t>(measurements[5]) / 200.0f;
  if (measurements[5] == 0x7FFF)
    temperature = NAN;

  float voc = NAN;
  float nox = NAN;
  float hcho = NAN;
  float co2 = NAN;

  if (voc_index >= 0) {
    voc = static_cast<int16_t>(measurements[voc_index]) / 10.0f;
    if (measurements[voc_index] == 0x7FFF)
      voc = NAN;
  }
  if (nox_index >= 0) {
    nox = static_cast<int16_t>(measurements[nox_index]) / 10.0f;
    if (measurements[nox_index] == 0x7FFF)
      nox = NAN;
  }

  if (hcho_index >= 0) {
    const uint16_t hcho_raw = measurements[hcho_index];
    hcho = hcho_raw / 10.0f;
    if (hcho_raw == 0xFFFF)
      hcho = NAN;
  }

  if (co2_index >= 0) {
    if (co2_uint16) {
      const uint16_t co2_raw = measurements[co2_index];
      co2 = static_cast<float>(co2_raw);
      if (co2_raw == 0xFFFF)
        co2 = NAN;
    } else {
      const int16_t co2_raw = static_cast<int16_t>(measurements[co2_index]);
      co2 = static_cast<float>(co2_raw);
      if (co2_raw == 0x7FFF)
        co2 = NAN;
    }
  }

  if (!this->startup_complete_) {
    ESP_LOGD(TAG, "Startup delay, ignoring values");
    this->status_clear_warning();
    this->finish_poll_cycle_();
    return;
  }

  if (this->pm_1_0_sensor_ != nullptr)
    this->pm_1_0_sensor_->publish_state(pm_1_0);
  if (this->pm_2_5_sensor_ != nullptr)
    this->pm_2_5_sensor_->publish_state(pm_2_5);
  if (this->pm_4_0_sensor_ != nullptr)
    this->pm_4_0_sensor_->publish_state(pm_4_0);
  if (this->pm_10_0_sensor_ != nullptr)
    this->pm_10_0_sensor_->publish_state(pm_10_0);
  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(temperature);
  if (this->humidity_sensor_ != nullptr)
    this->humidity_sensor_->publish_state(humidity);
  if (this->voc_sensor_ != nullptr)
    this->voc_sensor_->publish_state(voc);
  if (this->nox_sensor_ != nullptr)
    this->nox_sensor_->publish_state(nox);
  if (this->hcho_sensor_ != nullptr)
    this->hcho_sensor_->publish_state(hcho);
  if (this->co2_sensor_ != nullptr)
    this->co2_sensor_->publish_state(co2);

  this->status_clear_warning();

  if (this->pmc_0_5_sensor_ != nullptr || this->pmc_1_0_sensor_ != nullptr || this->pmc_2_5_sensor_ != nullptr ||
      this->pmc_4_0_sensor_ != nullptr || this->pmc_10_0_sensor_ != nullptr) {
    this->read_number_concentration_();
    return;
  }
  this->finish_poll_cycle_();
}

void SEN6XComponent::read_number_concentration_() {
  if (!this->write_command(SEN6X_CMD_READ_NUMBER_CONCENTRATION)) {
    this->status_set_warning();
    ESP_LOGD(TAG, "Read measurement failed (%d)", this->last_error_);
    return;
  }

  this->set_timeout(TIMEOUT_POLL, I2C_READ_DELAY, [this]() { this->parse_and_publish_number_concentration_(); });
}

void SEN6XComponent::parse_and_publish_number_concentration_() {
  uint16_t measurements[5];

  if (!this->read_data(measurements, 5)) {
    this->status_set_warning();
    ESP_LOGD(TAG, "Read data failed (%d)", this->last_error_);
    return;
  }

  sensor::Sensor *sensors[5] = {this->pmc_0_5_sensor_, this->pmc_1_0_sensor_, this->pmc_2_5_sensor_,
                                this->pmc_4_0_sensor_, this->pmc_10_0_sensor_};
  for (size_t i = 0; i < 5; i++) {
    if (sensors[i] != nullptr)
      sensors[i]->publish_state(measurements[i] == 0xFFFF ? NAN : measurements[i] / 10.0f);
  }

  this->finish_poll_cycle_();
}

// Returns true if a pressure write was issued to the device
bool SEN6XComponent::update_ambient_pressure_compensation_(float pressure_hpa) {
  // Range-check before narrowing so out-of-unit sources (e.g. Pa) can't wrap into range
  if (std::isnan(pressure_hpa) || pressure_hpa < 700.0f || pressure_hpa > 1200.0f) {
    if (!std::isnan(pressure_hpa) && !this->pressure_range_warned_) {
      ESP_LOGW(TAG, "Ambient pressure out of range: %.0f hPa", pressure_hpa);
      this->pressure_range_warned_ = true;
    }
    return false;
  }
  this->pressure_range_warned_ = false;
  uint16_t value = static_cast<uint16_t>(lroundf(pressure_hpa));
  if (this->last_ambient_pressure_.has_value() && this->last_ambient_pressure_.value() == value)
    return false;
  if (!this->write_command(SEN6X_CMD_AMBIENT_PRESSURE, &value, 1)) {
    this->status_set_warning();
    ESP_LOGD(TAG, "Write ambient pressure failed (%d)", this->last_error_);
    return false;
  }
  this->last_ambient_pressure_ = value;
  return true;
}

// True while the previous command's execution or settling time has not elapsed
bool SEN6XComponent::command_blocked_() {
  return wait_window_active(this->command_wait_started_at_, this->command_wait_ms_);
}

void SEN6XComponent::start_measurement() {
  if (!this->initialized_ || this->measuring_)
    return;
  // The CO2 conditioning window only blocks restarting, not other commands
  if (this->command_blocked_() || wait_window_active(this->co2_restart_started_at_, this->co2_restart_ms_)) {
    ESP_LOGW(TAG, "Device busy");
    return;
  }
  if (!this->write_command(SEN6X_CMD_START_MEASUREMENTS)) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Start measurement failed (%d)", this->last_error_);
    return;
  }
  this->measuring_ = true;
  start_wait_window(this->command_wait_started_at_, this->command_wait_ms_, START_MEASUREMENT_DELAY);
  if (this->sen6x_type_ == SEN63C || this->sen6x_type_ == SEN69C) {
    start_wait_window(this->co2_restart_started_at_, this->co2_restart_ms_, CO2_CONDITIONING_DELAY);
  }
  // Values need the warm-up period again after a restart
  this->startup_complete_ = false;
  this->set_timeout(TIMEOUT_STARTUP, this->startup_delay_ms_, [this]() { this->startup_complete_ = true; });
}

void SEN6XComponent::stop_measurement() {
  if (!this->initialized_ || !this->measuring_)
    return;
  if (this->command_blocked_()) {
    ESP_LOGW(TAG, "Device busy");
    return;
  }
  // Stop any in-flight polling before the device goes idle
  this->cancel_timeout(TIMEOUT_POLL);
  if (!this->write_command(SEN6X_CMD_STOP_MEASUREMENTS)) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Stop measurement failed (%d)", this->last_error_);
    return;
  }
  this->measuring_ = false;
  start_wait_window(this->command_wait_started_at_, this->command_wait_ms_, STOP_MEASUREMENT_DELAY);
}

void SEN6XComponent::start_fan_cleaning() {
  if (!this->initialized_)
    return;
  // The device only accepts this command in idle mode (datasheet section 4.8.22)
  if (this->measuring_) {
    ESP_LOGW(TAG, "Fan cleaning requires idle mode; stop the measurement first");
    return;
  }
  if (this->command_blocked_()) {
    ESP_LOGW(TAG, "Device busy");
    return;
  }
  if (!this->write_command(SEN6X_CMD_START_FAN_CLEANING)) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Fan cleaning failed (%d)", this->last_error_);
    return;
  }
  start_wait_window(this->command_wait_started_at_, this->command_wait_ms_, FAN_CLEANING_DELAY);
}

void SEN6XComponent::activate_sht_heater() {
  if (!this->initialized_)
    return;
  // The device only accepts this command in idle mode (datasheet section 4.8.23)
  if (this->measuring_) {
    ESP_LOGW(TAG, "SHT heater requires idle mode; stop the measurement first");
    return;
  }
  if (this->command_blocked_()) {
    ESP_LOGW(TAG, "Device busy");
    return;
  }
  if (!this->write_command(SEN6X_CMD_ACTIVATE_SHT_HEATER)) {
    this->status_set_warning();
    ESP_LOGW(TAG, "SHT heater failed (%d)", this->last_error_);
    return;
  }
  start_wait_window(this->command_wait_started_at_, this->command_wait_ms_, SHT_HEATER_DELAY);
}

// VOC algorithm state (datasheet sections 4.8.27, 4.8.28 and, for the reset, 4.8.21).
//
// Get (0x6181) is accepted in idle and in measurement mode, so a save is a plain read that only
// has to keep off the bus while the poll chain owns it. Set (0x6181) is accepted in idle mode
// only and is applied exactly once, when the next measurement starts, so a restore has to stop
// the measurement, write, and start again. The state survives a stop/start on its own; only a
// device reset or a power cycle clears it. None of this happens on a timer: the component keeps
// the mechanism and YAML decides when to use it.

// Loads the saved state during setup and, when enabled, hands it to the device. Returns true when
// a command was written, so the setup chain spends one step on it. Runs while the device is idle,
// just before finish_setup_() starts the measurement that applies the state.
bool SEN6XComponent::load_voc_state_and_restore_() {
  if (!this->voc_supported_)
    return false;

  // The preference key needs the serial number, which is only known after setup step 1. Hashing
  // the serial (and not the config version) keeps the state across an OTA, which is exactly the
  // short interruption it exists to bridge, and keeps two sensors on one node apart.
  //
  // Once per boot, never per call: make_preference() hands back a backend allocated with new that
  // nothing ever frees, and on the ESP8266 backend it also claims a fresh flash slot, moving the
  // stored blob out from under the copy already saved. reset_voc_algorithm() replays this whole
  // setup chain, so without the guard every reset would leak an object and orphan the state.
  if (!this->voc_pref_ready_) {
    this->voc_pref_ = global_preferences->make_preference<uint16_t[4]>(fnv1a_hash(this->serial_number_), true);
    this->voc_pref_ready_ = true;
  }

  uint16_t stored[4] = {0};
  if (!this->voc_pref_.load(&stored))
    return false;
  // Preferences cannot be erased, so clear_voc_state_() stores an all-zero blob to mean "nothing
  // saved"; an unwritten preference reads back the same way
  if ((stored[0] | stored[1] | stored[2] | stored[3]) == 0)
    return false;

  memcpy(this->voc_state_, stored, sizeof(this->voc_state_));
  this->voc_state_valid_ = true;
  if (!this->restore_voc_state_on_boot_) {
    ESP_LOGD(TAG, "Saved VOC state kept but not restored (restore on boot disabled)");
    return false;
  }

  ESP_LOGI(TAG, "Restoring VOC state: %04X %04X %04X %04X", this->voc_state_[0], this->voc_state_[1],
           this->voc_state_[2], this->voc_state_[3]);
  return this->write_config_words_(SEN6X_CMD_VOC_ALGORITHM_STATE, this->voc_state_, 4);
}

// End of one update cycle's I2C chain: the point at which a queued state read cannot collide
void SEN6XComponent::finish_poll_cycle_() { this->service_pending_voc_save_(); }

void SEN6XComponent::save_voc_state() {
  if (!this->initialized_ || !this->voc_supported_) {
    ESP_LOGW(TAG, "VOC state unavailable");
    return;
  }
  // Queued rather than run here: the poll chain may hold the bus. Measuring cycles service the
  // flag at the end of the chain and idle cycles from update(), so the trigger always takes
  this->voc_save_pending_ = true;
  this->service_pending_voc_save_();
}

void SEN6XComponent::service_pending_voc_save_() {
  if (!this->voc_save_pending_ || this->voc_sequence_active_ || this->command_blocked_())
    return;

  if (!this->write_command(SEN6X_CMD_VOC_ALGORITHM_STATE)) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Read VOC state failed (%d)", this->last_error_);
    return;  // stays queued, retried on the next cycle
  }
  this->voc_save_pending_ = false;
  // Held until the read completes so a restore or reset pressed meanwhile is refused rather than
  // silently cancelling this timeout, which shares its ID
  this->voc_sequence_active_ = true;

  this->set_timeout(TIMEOUT_ACTION, CMD_EXEC_DELAY, [this]() {
    this->voc_sequence_active_ = false;
    uint16_t state[4];
    if (!this->read_data(state, 4)) {
      this->status_set_warning();
      ESP_LOGW(TAG, "Read VOC state failed (%d)", this->last_error_);
      return;
    }
    memcpy(this->voc_state_, state, sizeof(this->voc_state_));
    this->voc_state_valid_ = true;
    if (!this->voc_pref_.save(&this->voc_state_)) {
      ESP_LOGW(TAG, "Storing VOC state failed");
      return;
    }
    // A manual save is usually the last thing before a reboot, so commit now rather than waiting
    // for the preferences component's next flush
    global_preferences->sync();
    ESP_LOGI(TAG, "Saved VOC state: %04X %04X %04X %04X", this->voc_state_[0], this->voc_state_[1], this->voc_state_[2],
             this->voc_state_[3]);
  });
}

void SEN6XComponent::restore_voc_state() {
  if (!this->initialized_ || !this->voc_supported_) {
    ESP_LOGW(TAG, "VOC state unavailable");
    return;
  }
  if (!this->voc_state_valid_) {
    ESP_LOGW(TAG, "No saved VOC state");
    return;
  }
  // The CO2 conditioning window would block the restart at the end of the sequence, so it is
  // checked here rather than stopping the measurement and then failing to start it again
  if (this->voc_sequence_active_ || this->command_blocked_() ||
      wait_window_active(this->co2_restart_started_at_, this->co2_restart_ms_)) {
    ESP_LOGW(TAG, "Device busy");
    return;
  }

  const bool was_measuring = this->measuring_;
  uint32_t wait = 0;
  this->voc_sequence_active_ = true;
  if (was_measuring) {
    // stop_measurement() cancels the in-flight poll and opens the 1400 ms settling window
    this->stop_measurement();
    if (this->measuring_) {
      this->voc_sequence_active_ = false;
      return;
    }
    // One command-execution time past the datasheet's 1400 ms, so the next write is never on the
    // boundary of the window it has to clear
    wait = STOP_MEASUREMENT_DELAY + CMD_EXEC_DELAY;
  }

  this->set_timeout(TIMEOUT_ACTION, wait, [this, was_measuring]() {
    const bool written = this->write_config_words_(SEN6X_CMD_VOC_ALGORITHM_STATE, this->voc_state_, 4);
    this->set_timeout(TIMEOUT_ACTION, CMD_EXEC_DELAY, [this, was_measuring, written]() {
      this->voc_sequence_active_ = false;
      if (was_measuring) {
        // Starting the measurement is what applies the state
        this->start_measurement();
      }
      if (!written)
        return;
      if (was_measuring) {
        ESP_LOGI(TAG, "Restored VOC state");
      } else {
        ESP_LOGI(TAG, "Restored VOC state; applied at the next measurement start");
      }
    });
  });
}

void SEN6XComponent::reset_voc_algorithm() {
  if (!this->initialized_) {
    ESP_LOGW(TAG, "Not initialized");
    return;
  }
  if (this->voc_sequence_active_ || this->command_blocked_()) {
    ESP_LOGW(TAG, "Device busy");
    return;
  }

  uint32_t wait = 0;
  this->voc_sequence_active_ = true;
  if (this->measuring_) {
    this->stop_measurement();
    if (this->measuring_) {
      this->voc_sequence_active_ = false;
      return;
    }
    wait = STOP_MEASUREMENT_DELAY + CMD_EXEC_DELAY;
  }

  this->set_timeout(TIMEOUT_ACTION, wait, [this]() {
    // A device reset is the only way to clear the VOC engine without a power cycle: the state
    // survives a stop/start (datasheet section 4.8.27). It is accepted in idle mode only.
    if (!this->write_command(SEN6X_CMD_RESET)) {
      this->status_set_warning();
      ESP_LOGW(TAG, "Device reset failed (%d)", this->last_error_);
      this->voc_sequence_active_ = false;
      this->set_timeout(TIMEOUT_ACTION, CMD_EXEC_DELAY, [this]() { this->start_measurement(); });
      return;
    }
    this->initialized_ = false;
    this->measuring_ = false;
    this->startup_complete_ = false;
    this->voc_save_pending_ = false;
    // Otherwise the next boot would restore the calibration that was just discarded
    this->clear_voc_state_();
    // Every setting the component writes is volatile and is back at its default after the reset,
    // including the pressure the poll cycle skips rewriting when unchanged
    this->last_ambient_pressure_.reset();

    // The device needs its full reset execution time before it accepts anything again
    this->set_timeout(TIMEOUT_ACTION, DEVICE_RESET_DELAY + CMD_EXEC_DELAY, [this]() {
      this->voc_sequence_active_ = false;
      ESP_LOGI(TAG, "VOC algorithm reset; reapplying configuration");
      // Replays the whole configuration chain, which ends in finish_setup_() starting the
      // measurement and re-arming the startup and CO2 conditioning windows
      this->setup_step_index_ = 0;
      this->run_next_setup_step_();
    });
  });
}

void SEN6XComponent::clear_voc_state_() {
  memset(this->voc_state_, 0, sizeof(this->voc_state_));
  this->voc_state_valid_ = false;
  // An all-zero blob is the "nothing saved" marker; preferences have no erase
  this->voc_pref_.save(&this->voc_state_);
  global_preferences->sync();
}

#ifdef USE_BINARY_SENSOR
// Reads and clears the status register, then continues into the measurement poll chain.
// Read And Clear (0xD210) is used because error flags are sticky on the device; clearing
// each cycle makes the sensors report whether a problem occurred since the last update.
void SEN6XComponent::read_device_status_() {
  if (!this->write_command(SEN6X_CMD_READ_AND_CLEAR_DEVICE_STATUS)) {
    this->status_set_warning();
    ESP_LOGD(TAG, "Read device status failed (%d)", this->last_error_);
    this->poll_data_ready_();
    return;
  }

  this->set_timeout(TIMEOUT_POLL, I2C_READ_DELAY, [this]() { this->parse_and_publish_device_status_(); });
}

void SEN6XComponent::parse_and_publish_device_status_() {
  uint16_t words[2];

  if (!this->read_data(words, 2)) {
    this->status_set_warning();
    ESP_LOGD(TAG, "Read data failed (%d)", this->last_error_);
    this->poll_data_ready_();
    return;
  }
  const uint32_t status = (static_cast<uint32_t>(words[0]) << 16) | words[1];

  // Bit positions from the device status register (datasheet section 4.3).
  // The CO2 error bit is 9 on SEN66 and 12 on SEN63C/SEN69C.
  const uint32_t co2_mask = this->sen6x_type_ == SEN66 ? (1UL << 9) : (1UL << 12);
  const struct {
    binary_sensor::BinarySensor *sensor;
    uint32_t mask;
  } statuses[] = {
      {this->fan_error_binary_sensor_, 1UL << 4}, {this->fan_speed_warning_binary_sensor_, 1UL << 21},
      {this->rht_error_binary_sensor_, 1UL << 6}, {this->gas_error_binary_sensor_, 1UL << 7},
      {this->co2_error_binary_sensor_, co2_mask}, {this->hcho_error_binary_sensor_, 1UL << 10},
      {this->pm_error_binary_sensor_, 1UL << 11},
  };
  for (const auto &entry : statuses) {
    if (entry.sensor != nullptr)
      entry.sensor->publish_state((status & entry.mask) != 0);
  }

  this->poll_data_ready_();
}
#endif

SEN6XComponent::Sen6xType SEN6XComponent::infer_type_from_product_name_(const std::string &product_name) {
  if (product_name == "SEN62")
    return SEN62;
  if (product_name == "SEN63C")
    return SEN63C;
  if (product_name == "SEN65")
    return SEN65;
  if (product_name == "SEN66")
    return SEN66;
  if (product_name == "SEN68")
    return SEN68;
  if (product_name == "SEN69C")
    return SEN69C;
  return UNKNOWN;
}

}  // namespace esphome::sen6x
