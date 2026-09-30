#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/optional.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/sensirion_common/i2c_sensirion.h"
#include <array>
#include <utility>
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

namespace esphome::sen6x {

// The NOx algorithm requires std_initial to stay at 50 (Sensirion datasheet)
static constexpr uint16_t NOX_STD_INITIAL = 50;

// Raw parameter block for the VOC/NOx algorithm tuning commands
struct GasTuning {
  uint16_t index_offset;
  uint16_t learning_time_offset_hours;
  uint16_t learning_time_gain_hours;
  uint16_t gating_max_duration_minutes;
  uint16_t std_initial;
  uint16_t gain_factor;
};

// Raw values in the device's fixed-point encoding (offset x200, slope x10000)
struct TemperatureCompensation {
  int16_t offset;
  int16_t normalized_offset_slope;
  uint16_t time_constant;
};

// Raw values in the device's fixed-point encoding (all x10)
struct TemperatureAcceleration {
  uint16_t k;
  uint16_t p;
  uint16_t t1;
  uint16_t t2;
};

class SEN6XComponent final : public PollingComponent, public sensirion_common::SensirionI2CDevice {
  SUB_SENSOR(pm_1_0)
  SUB_SENSOR(pm_2_5)
  SUB_SENSOR(pm_4_0)
  SUB_SENSOR(pm_10_0)
  SUB_SENSOR(pmc_0_5)
  SUB_SENSOR(pmc_1_0)
  SUB_SENSOR(pmc_2_5)
  SUB_SENSOR(pmc_4_0)
  SUB_SENSOR(pmc_10_0)
  SUB_SENSOR(temperature)
  SUB_SENSOR(humidity)
  SUB_SENSOR(voc)
  SUB_SENSOR(nox)
  SUB_SENSOR(raw_voc)
  SUB_SENSOR(co2)
  SUB_SENSOR(hcho)
#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(fan_error)
  SUB_BINARY_SENSOR(fan_speed_warning)
  SUB_BINARY_SENSOR(rht_error)
  SUB_BINARY_SENSOR(gas_error)
  SUB_BINARY_SENSOR(co2_error)
  SUB_BINARY_SENSOR(hcho_error)
  SUB_BINARY_SENSOR(pm_error)
#endif

 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void dump_config() override;
  void update() override;

  enum Sen6xType { SEN62, SEN63C, SEN65, SEN66, SEN68, SEN69C, UNKNOWN };

  void set_type(const std::string &type) { sen6x_type_ = infer_type_from_product_name_(type); }
  void set_voc_algorithm_tuning(uint16_t index_offset, uint16_t learning_time_offset_hours,
                                uint16_t learning_time_gain_hours, uint16_t gating_max_duration_minutes,
                                uint16_t std_initial, uint16_t gain_factor) {
    this->voc_tuning_params_ = GasTuning{
        index_offset, learning_time_offset_hours, learning_time_gain_hours, gating_max_duration_minutes, std_initial,
        gain_factor};
  }
  void set_nox_algorithm_tuning(uint16_t index_offset, uint16_t learning_time_offset_hours,
                                uint16_t learning_time_gain_hours, uint16_t gating_max_duration_minutes,
                                uint16_t gain_factor) {
    this->nox_tuning_params_ = GasTuning{index_offset,
                                         learning_time_offset_hours,
                                         learning_time_gain_hours,
                                         gating_max_duration_minutes,
                                         NOX_STD_INITIAL,
                                         gain_factor};
  }
  void set_startup_delay(uint32_t delay_ms) { this->startup_delay_ms_ = delay_ms; }
  void set_temperature_compensation(float offset, float normalized_offset_slope, uint16_t time_constant);
  void set_temperature_acceleration(float k, float p, float t1, float t2);
  void set_automatic_self_calibration(bool enabled) { this->co2_asc_ = enabled; }
  void set_altitude_compensation(uint16_t altitude) { this->altitude_compensation_ = altitude; }
  void set_ambient_pressure_compensation(uint16_t pressure_hpa) { this->ambient_pressure_ = pressure_hpa; }
  void set_ambient_pressure_source(sensor::Sensor *pressure) { this->ambient_pressure_source_ = pressure; }
  void start_measurement();
  void stop_measurement();
  void start_fan_cleaning();
  void activate_sht_heater();
  void set_restore_voc_state_on_boot(bool restore) { this->restore_voc_state_on_boot_ = restore; }
  // VOC algorithm state (datasheet sections 4.8.27, 4.8.28, 4.8.21). Saving and restoring are
  // always explicit: nothing here runs on a timer, it is driven from YAML.
  void save_voc_state();
  // Reads the state from the device without storing it; see get_latest_voc_state()
  void read_voc_state();
  void restore_voc_state();
  void reset_voc_algorithm();
  bool has_voc_state() const { return this->voc_state_valid_; }
  // The four words of the stored VOC state as the device hands them out; meaningful while has_voc_state().
  //
  // The datasheet documents the state only as 8 opaque bytes. They match the two states of Sensirion's gas
  // index algorithm (github.com/Sensirion/gas-index-algorithm, GasIndexAlgorithm_get_states), each a
  // big-endian 16.16 fixed-point number learned from the raw VOC signal:
  //   words 0-1  mean of the raw signal, stored less 20000 (GasIndexAlgorithm_VOC_SRAW_MINIMUM)
  //   words 2-3  standard deviation of the raw signal
  // From them the algorithm turns a raw reading into the VOC Index as
  //   (reading - mean) / -(std + 220) x 230
  // where 220 is GasIndexAlgorithm_SRAW_STD_BONUS_VOC and 230 the default gain factor (VOC tuning, 0x60D0).
  // A sigmoid then maps the result onto the 0..500 index scale, 0 landing on index_offset (100 by default),
  // and a low-pass filter smooths it.
  std::array<uint16_t, 4> get_voc_state() const {
    return {this->voc_state_[0], this->voc_state_[1], this->voc_state_[2], this->voc_state_[3]};
  }
  // Runs whenever the stored VOC state changes: after each save, when it is read back from flash
  // during setup, and when a VOC algorithm reset clears it
  template<typename F> void add_on_voc_state_stored_callback(F &&callback) {
    this->voc_state_stored_callback_.add(std::forward<F>(callback));
  }
  // The most recent VOC state known, in the same layout as get_voc_state(): from the last read or save,
  // or the stored copy until the first of those after boot. Kept apart from the stored copy, which is
  // what a restore hands back.
  bool has_latest_voc_state() const { return this->voc_state_latest_valid_; }
  std::array<uint16_t, 4> get_latest_voc_state() const {
    return {this->voc_state_latest_[0], this->voc_state_latest_[1], this->voc_state_latest_[2],
            this->voc_state_latest_[3]};
  }
  // Runs whenever the latest VOC state changes: after each read or save, when the stored copy is read
  // back during setup, and when a VOC algorithm reset clears it
  template<typename F> void add_on_voc_state_update_callback(F &&callback) {
    this->voc_state_update_callback_.add(std::forward<F>(callback));
  }

 protected:
  Sen6xType infer_type_from_product_name_(const std::string &product_name);
  bool command_blocked_();
  void run_next_setup_step_();
  void finish_setup_();
  bool write_config_words_(uint16_t i2c_command, const uint16_t *data, uint8_t len);
  bool write_tuning_parameters_(uint16_t i2c_command, const GasTuning &tuning);
  bool write_setup_register_(uint16_t i2c_command, uint16_t value);
  bool write_temperature_compensation_(const TemperatureCompensation &compensation);
  bool write_temperature_acceleration_(const TemperatureAcceleration &acceleration);
  bool update_ambient_pressure_compensation_(float pressure_hpa);
  void poll_data_ready_();
  void read_measurements_();
  void parse_and_publish_measurements_();
  void read_raw_values_();
  void parse_and_publish_raw_values_();
  void read_number_concentration_or_finish_();
  void read_number_concentration_();
  void parse_and_publish_number_concentration_();
  void start_poll_chain_();
  void finish_poll_cycle_();
  bool load_voc_state_and_restore_();
  void service_pending_voc_state_read_();
  void set_latest_voc_state_(const uint16_t *state);
  void clear_voc_state_();
#ifdef USE_BINARY_SENSOR
  void read_device_status_();
  void parse_and_publish_device_status_();
#endif

  std::string product_name_;
  std::string serial_number_;
  optional<GasTuning> voc_tuning_params_;
  optional<GasTuning> nox_tuning_params_;
  optional<TemperatureAcceleration> temperature_acceleration_;
  optional<TemperatureCompensation> temperature_compensation_;
  sensor::Sensor *ambient_pressure_source_{nullptr};
  uint32_t startup_delay_ms_{60000};
  // Post-command wait windows from the datasheet, held as (start, duration) rather than a
  // deadline so the elapsed comparison stays bounded — see wait_window_active() in sen6x.cpp
  uint32_t command_wait_started_at_{0};
  uint32_t command_wait_ms_{0};
  uint32_t co2_restart_started_at_{0};
  uint32_t co2_restart_ms_{0};
  Sen6xType sen6x_type_{UNKNOWN};
  optional<uint16_t> altitude_compensation_;
  optional<uint16_t> ambient_pressure_;
  // Last pressure written to the device, used to skip redundant writes
  optional<uint16_t> last_ambient_pressure_;
  uint16_t read_cmd_{0};
  optional<bool> co2_asc_;
  uint8_t setup_step_index_{0};
  uint8_t firmware_version_major_{0};
  uint8_t firmware_version_minor_{0};
  uint8_t poll_retries_remaining_{0};
  uint8_t read_words_{0};
  bool initialized_{false};
  bool measuring_{false};
  bool pressure_range_warned_{false};
#ifdef USE_BINARY_SENSOR
  bool has_status_sensors_{false};
#endif
  bool startup_complete_{false};
  // VOC algorithm state, an opaque 8-byte blob the device hands out and takes back
  bool voc_supported_{false};
  bool voc_state_valid_{false};
  // make_preference() allocates a backend that is never freed, so it is called once per boot
  bool voc_pref_ready_{false};
  bool voc_save_pending_{false};
  bool voc_read_pending_{false};
  bool voc_state_latest_valid_{false};
  // Set while a stop/write/start sequence owns the bus, so nothing else writes underneath it
  bool voc_sequence_active_{false};
  // Set from the first bus write of a poll cycle until finish_poll_cycle_(), so a queued VOC state
  // read waits for the chain instead of landing between one of its writes and the matching read
  bool poll_active_{false};
  bool restore_voc_state_on_boot_{true};
  uint16_t voc_state_[4]{0};
  uint16_t voc_state_latest_[4]{0};
  ESPPreferenceObject voc_pref_;
  CallbackManager<void()> voc_state_stored_callback_;
  CallbackManager<void()> voc_state_update_callback_;
};

}  // namespace esphome::sen6x
