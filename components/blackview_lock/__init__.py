import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, text_sensor, binary_sensor, sensor
from esphome.const import CONF_ID

DEPENDENCIES = ['ble_client']

blackview_lock_ns = cg.esphome_ns.namespace('blackview_lock')
BlackviewLock = blackview_lock_ns.class_('BlackviewLock', cg.Component, ble_client.BLEClientNode)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BlackviewLock),
    cv.Required("ble_client_id"): cv.use_id(ble_client.BLEClient),

    # NEW optional fields you tried to use in YAML
    cv.Optional("fallback_write_handle"): cv.hex_uint16_t,
    cv.Optional("fallback_notify_handle"): cv.hex_uint16_t,
    cv.Optional("fallback_cccd_handle"): cv.hex_uint16_t,
    cv.Optional("prefer_write_no_rsp", default=False): cv.boolean,

    # (Optional) UUIDs if we ever want to change them without code
    cv.Optional("write_uuid", default="00002b11-0000-1000-8000-00805f9b34fb"): cv.string,
    cv.Optional("notify_uuid", default="00002b10-0000-1000-8000-00805f9b34fb"): cv.string,

    # already-supported sensors/bools
    cv.Optional("session_key"): text_sensor.text_sensor_schema(),
    cv.Optional("last_notify"): text_sensor.text_sensor_schema(),
    cv.Optional("key_received"): binary_sensor.binary_sensor_schema(),
    cv.Optional("connected"): binary_sensor.binary_sensor_schema(),
    cv.Optional("notify_count"): sensor.sensor_schema(unit_of_measurement="msgs", accuracy_decimals=0),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    parent = await cg.get_variable(config["ble_client_id"])
    cg.add(parent.register_ble_node(var))

    # NEW: plumb optional fields into the C++ object
    if "fallback_write_handle" in config:
        cg.add(var.set_fallback_write_handle(config["fallback_write_handle"]))
    if "fallback_notify_handle" in config:
        cg.add(var.set_fallback_notify_handle(config["fallback_notify_handle"]))
    if "fallback_cccd_handle" in config:
        cg.add(var.set_fallback_cccd_handle(config["fallback_cccd_handle"]))
    if "prefer_write_no_rsp" in config:
        cg.add(var.set_prefer_write_no_rsp(config["prefer_write_no_rsp"]))
    if "write_uuid" in config:
        cg.add(var.set_write_uuid(config["write_uuid"]))
    if "notify_uuid" in config:
        cg.add(var.set_notify_uuid(config["notify_uuid"]))

    # sensors
    if "session_key" in config:
        sens = await text_sensor.new_text_sensor(config["session_key"])
        cg.add(var.set_session_key_sensor(sens))
    if "last_notify" in config:
        sens = await text_sensor.new_text_sensor(config["last_notify"])
        cg.add(var.set_last_notify_sensor(sens))
    if "key_received" in config:
        bs = await binary_sensor.new_binary_sensor(config["key_received"])
        cg.add(var.set_key_received_sensor(bs))
    if "connected" in config:
        bs = await binary_sensor.new_binary_sensor(config["connected"])
        cg.add(var.set_connected_sensor(bs))
    if "notify_count" in config:
        sc = await sensor.new_sensor(config["notify_count"])
        cg.add(var.set_notify_count_sensor(sc))
