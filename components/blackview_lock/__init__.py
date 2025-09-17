import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client
from esphome.const import CONF_ID

DEPENDENCIES = ["ble_client"]

blackview_lock_ns = cg.esphome_ns.namespace("blackview_lock")
BlackviewLock = blackview_lock_ns.class_("BlackviewLock", cg.Component, ble_client.BLEClientNode)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BlackviewLock),
    cv.Required("ble_client_id"): cv.use_id(ble_client.BLEClient),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    parent = await cg.get_variable(config["ble_client_id"])
    cg.add(parent.register_ble_node(var))
