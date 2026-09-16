#pragma once

#include "esphome/components/button/button.h"
#include "esphome/components/sen6x/sen6x.h"
#include "esphome/core/component.h"

namespace esphome::sen6x {

class SEN6XSaveVocStateButton final : public button::Button, public Parented<SEN6XComponent> {
 public:
  SEN6XSaveVocStateButton() = default;

 protected:
  void press_action() override;
};

class SEN6XRestoreVocStateButton final : public button::Button, public Parented<SEN6XComponent> {
 public:
  SEN6XRestoreVocStateButton() = default;

 protected:
  void press_action() override;
};

class SEN6XResetVocAlgorithmButton final : public button::Button, public Parented<SEN6XComponent> {
 public:
  SEN6XResetVocAlgorithmButton() = default;

 protected:
  void press_action() override;
};

}  // namespace esphome::sen6x
