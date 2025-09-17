import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, text_sensor, binary_sensor, sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["ble_client"]

blackview_lock_ns = cg.esphome_ns.namespace("blackview_lock")
BlackviewLock = blackview_lock_ns.class_("BlackviewLock", cg.Component, ble_client.BLEClientNode)

CONF_BLE_CLIENT_ID = "ble_client_id"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BlackviewLock),
    cv.Required(CONF_BLE_CLIENT_ID): cv.use_id(ble_client.BLEClient),

    cv.Optional("session_key"): text_sensor.text_sensor_schema(),
    cv.Optional("last_notify"): text_sensor.text_sensor_schema(),
    cv.Optional("key_received"): binary_sensor.binary_sensor_schema(),
    cv.Optional("connected"): binary_sensor.binary_sensor_schema(),
    cv.Optional("notify_count"): sensor.sensor_schema(accuracy_decimals=0),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_BLE_CLIENT_ID])
    cg.add(parent.register_ble_node(var))

    if "session_key" in config:
        ts = await text_sensor.new_text_sensor(config["session_key"])
        cg.add(var.set_session_key_text_sensor(ts))

    if "last_notify" in config:
        ts = await text_sensor.new_text_sensor(config["last_notify"])
        cg.add(var.set_last_notify_text_sensor(ts))

    if "key_received" in config:
        bs = await binary_sensor.new_binary_sensor(config["key_received"])
        cg.add(var.set_key_received_binary_sensor(bs))

    if "connected" in config:
        bs = await binary_sensor.new_binary_sensor(config["connected"])
        cg.add(var.set_connected_binary_sensor(bs))

    if "notify_count" in config:
        s = await sensor.new_sensor(config["notify_count"])
        cg.add(var.set_notify_count_sensor(s))
