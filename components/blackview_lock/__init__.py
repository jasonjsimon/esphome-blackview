import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

# This component no longer depends on ble_client
DEPENDENCIES = []

blackview_lock_ns = cg.esphome_ns.namespace('blackview_lock')
BlackviewLock = blackview_lock_ns.class_('BlackviewLock', cg.PollingComponent)

CONFIG_SCHEMA = cv.polling_component_schema('60s').extend({
    cv.GenerateID(): cv.declare_id(BlackviewLock),
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
