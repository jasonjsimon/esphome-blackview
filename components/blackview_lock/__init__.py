import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32_ble_client
from esphome.const import CONF_ID

DEPENDENCIES = ['esp32_ble_client']

blackview_lock_ns = cg.esphome_ns.namespace('blackview_lock')
BlackviewLock = blackview_lock_ns.class_('BlackviewLock', cg.Component, esp32_ble_client.BLEClientBase)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BlackviewLock),
    cv.Required("ble_client_id"): cv.use_id(esp32_ble_client.BLEClientBase),
}).extend(cv.COMPONENT_SCHEMA)

# Reverting to the simpler, more robust helper function for code generation
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await esp32_ble_client.register_ble_node(var, config)
