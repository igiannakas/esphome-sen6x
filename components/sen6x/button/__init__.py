import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG
from esphome.types import ConfigType

from ..sensor import SEN6XComponent, sen6x_ns

CONF_SEN6X_ID = "sen6x_id"
CONF_SAVE_VOC_STATE = "save_voc_state"
CONF_RESTORE_VOC_STATE = "restore_voc_state"
CONF_RESET_VOC_ALGORITHM = "reset_voc_algorithm"

SEN6XSaveVocStateButton = sen6x_ns.class_("SEN6XSaveVocStateButton", button.Button)
SEN6XRestoreVocStateButton = sen6x_ns.class_(
    "SEN6XRestoreVocStateButton", button.Button
)
SEN6XResetVocAlgorithmButton = sen6x_ns.class_(
    "SEN6XResetVocAlgorithmButton", button.Button
)

BUTTON_MAP = {
    CONF_SAVE_VOC_STATE: (SEN6XSaveVocStateButton, "mdi:content-save-cog"),
    CONF_RESTORE_VOC_STATE: (SEN6XRestoreVocStateButton, "mdi:backup-restore"),
    CONF_RESET_VOC_ALGORITHM: (SEN6XResetVocAlgorithmButton, "mdi:restart-alert"),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEN6X_ID): cv.use_id(SEN6XComponent),
        **{
            cv.Optional(key): button.button_schema(
                class_,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon=icon,
            )
            for key, (class_, icon) in BUTTON_MAP.items()
        },
    }
)


async def to_code(config: ConfigType) -> None:
    parent = await cg.get_variable(config[CONF_SEN6X_ID])
    for key in BUTTON_MAP:
        if cfg := config.get(key):
            b = await button.new_button(cfg)
            await cg.register_parented(b, parent)
