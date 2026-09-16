#include "sen6x_button.h"
#include "esphome/core/log.h"

namespace esphome::sen6x {

static const char *const TAG = "sen6x.button";

void SEN6XSaveVocStateButton::press_action() {
  ESP_LOGD(TAG, "Saving VOC state");
  this->parent_->save_voc_state();
}

void SEN6XRestoreVocStateButton::press_action() {
  ESP_LOGD(TAG, "Restoring VOC state");
  this->parent_->restore_voc_state();
}

void SEN6XResetVocAlgorithmButton::press_action() {
  ESP_LOGD(TAG, "Resetting VOC algorithm");
  this->parent_->reset_voc_algorithm();
}

}  // namespace esphome::sen6x
