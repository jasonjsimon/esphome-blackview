import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, binary_sensor, sensor, text_sensor
from esphome.const import CONF_ID

CODEOWNERS = [""]

blackview_ns = cg.esphome_ns.namespace("blackview_lock")
BlackviewLock = blackview_ns.class_("BlackviewLock", cg.Component, ble_client.BLEClientNode)

CONF_BLE_CLIENT_ID = "ble_client_id"
CONF_PREFER_WRITE_NO_RSP = "prefer_write_no_rsp"
CONF_WRITE_UUID = "write_uuid"
CONF_NOTIFY_UUID = "notify_uuid"
CONF_FALLBACK_WRITE_HANDLE = "fallback_write_handle"
CONF_FALLBACK_NOTIFY_HANDLE = "fallback_notify_handle"
CONF_FALLBACK_CCCD_HANDLE = "fallback_cccd_handle"

CONF_SESSION_KEY = "session_key"
CONF_LAST_NOTIFY = "last_notify"
CONF_KEY_RECEIVED = "key_received"
CONF_CONNECTED = "connected"
CONF_NOTIFY_COUNT = "notify_count"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BlackviewLock),
    cv.GenerateID(CONF_BLE_CLIENT_ID): cv.use_id(ble_client.BLEClient),

    cv.Optional(CONF_PREFER_WRITE_NO_RSP, default=False): cv.boolean,
    cv.Optional(CONF_WRITE_UUID, default="00002b11-0000-1000-8000-00805f9b34fb"): cv.uuid,
    cv.Optional(CONF_NOTIFY_UUID, default="00002b10-0000-1000-8000-00805f9b34fb"): cv.uuid,

    cv.Optional(CONF_FALLBACK_WRITE_HANDLE): cv.hex_uint16_t,
    cv.Optional(CONF_FALLBACK_NOTIFY_HANDLE): cv.hex_uint16_t,
    cv.Optional(CONF_FALLBACK_CCCD_HANDLE): cv.hex_uint16_t,

    cv.Optional(CONF_SESSION_KEY): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_LAST_NOTIFY): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_KEY_RECEIVED): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_NOTIFY_COUNT): sensor.sensor_schema(),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # IMPORTANT: ble_client expects the full config dict here
    await ble_client.register_ble_node(var, config)

    cg.add(var.set_prefer_write_no_rsp(config[CONF_PREFER_WRITE_NO_RSP]))
    cg.add(var.set_write_uuid(str(config[CONF_WRITE_UUID]).lower()))
    cg.add(var.set_notify_uuid(str(config[CONF_NOTIFY_UUID]).lower()))

    if CONF_FALLBACK_WRITE_HANDLE in config:
        cg.add(var.set_fallback_write_handle(config[CONF_FALLBACK_WRITE_HANDLE]))
    if CONF_FALLBACK_NOTIFY_HANDLE in config:
        cg.add(var.set_fallback_notify_handle(config[CONF_FALLBACK_NOTIFY_HANDLE]))
    if CONF_FALLBACK_CCCD_HANDLE in config:
        cg.add(var.set_fallback_cccd_handle(config[CONF_FALLBACK_CCCD_HANDLE]))

    if CONF_SESSION_KEY in config:
        t1 = await text_sensor.new_text_sensor(config[CONF_SESSION_KEY])
        cg.add(var.set_session_key_text_sensor(t1))

    if CONF_LAST_NOTIFY in config:
        t2 = await text_sensor.new_text_sensor(config[CONF_LAST_NOTIFY])
        cg.add(var.set_last_notify_text_sensor(t2))

    if CONF_KEY_RECEIVED in config:
        b1 = await binary_sensor.new_binary_sensor(config[CONF_KEY_RECEIVED])
        cg.add(var.set_key_received_binary_sensor(b1))

    if CONF_CONNECTED in config:
        b2 = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
        cg.add(var.set_connected_binary_sensor(b2))

    if CONF_NOTIFY_COUNT in config:
        s1 = await sensor.new_sensor(config[CONF_NOTIFY_COUNT])
        cg.add(var.set_notify_count_sensor(s1))
