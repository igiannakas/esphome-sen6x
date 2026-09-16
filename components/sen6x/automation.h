#pragma once

#include "esphome/core/automation.h"
#include "sen6x.h"

namespace esphome::sen6x {

template<typename... Ts> class StartMeasurementAction final : public Action<Ts...> {
 public:
  explicit StartMeasurementAction(SEN6XComponent *sen6x) : sen6x_(sen6x) {}

  void play(const Ts &...x) override { this->sen6x_->start_measurement(); }

 protected:
  SEN6XComponent *sen6x_;
};

template<typename... Ts> class StopMeasurementAction final : public Action<Ts...> {
 public:
  explicit StopMeasurementAction(SEN6XComponent *sen6x) : sen6x_(sen6x) {}

  void play(const Ts &...x) override { this->sen6x_->stop_measurement(); }

 protected:
  SEN6XComponent *sen6x_;
};

template<typename... Ts> class StartFanCleaningAction final : public Action<Ts...> {
 public:
  explicit StartFanCleaningAction(SEN6XComponent *sen6x) : sen6x_(sen6x) {}

  void play(const Ts &...x) override { this->sen6x_->start_fan_cleaning(); }

 protected:
  SEN6XComponent *sen6x_;
};

template<typename... Ts> class ActivateHeaterAction final : public Action<Ts...> {
 public:
  explicit ActivateHeaterAction(SEN6XComponent *sen6x) : sen6x_(sen6x) {}

  void play(const Ts &...x) override { this->sen6x_->activate_sht_heater(); }

 protected:
  SEN6XComponent *sen6x_;
};

template<typename... Ts> class SaveVocStateAction final : public Action<Ts...> {
 public:
  explicit SaveVocStateAction(SEN6XComponent *sen6x) : sen6x_(sen6x) {}

  void play(const Ts &...x) override { this->sen6x_->save_voc_state(); }

 protected:
  SEN6XComponent *sen6x_;
};

template<typename... Ts> class RestoreVocStateAction final : public Action<Ts...> {
 public:
  explicit RestoreVocStateAction(SEN6XComponent *sen6x) : sen6x_(sen6x) {}

  void play(const Ts &...x) override { this->sen6x_->restore_voc_state(); }

 protected:
  SEN6XComponent *sen6x_;
};

template<typename... Ts> class ResetVocAlgorithmAction final : public Action<Ts...> {
 public:
  explicit ResetVocAlgorithmAction(SEN6XComponent *sen6x) : sen6x_(sen6x) {}

  void play(const Ts &...x) override { this->sen6x_->reset_voc_algorithm(); }

 protected:
  SEN6XComponent *sen6x_;
};

}  // namespace esphome::sen6x
