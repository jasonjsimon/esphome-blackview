#pragma once

#include "esphome.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"

#include <esp_gattc_api.h>
#include <esp_gatt_defs.h>

namespace esphome {
namespace blackview_lock {

using namespace esphome::ble_client;
using esphome::text_sensor::TextSensor;
using esphome::binary_sensor::BinarySensor;
using esphome::sensor::Sensor;

static const char *const TAG = "blackview_lock";

// 16-bit UUIDs (full base: 0000xxxx-0000-1000-8000-00805f9b34fb)
static constexpr uint16_t UUID_NOTIFY_CHAR = 0x2B10;
static constexpr uint16_t UUID_WRITE_CHAR  = 0x2B11;
static constexpr uint16_t UUID_CCCD_DESCR  = 0x2902;

// Keep early HELLO to win the race; set to 0 to disable if needed
static constexpr uint16_t FALLBACK_WRITE_HANDLE = 0x000E;

enum HelloVariant : uint8_t {
  HELLO_STD_LE = 0,        // header + payload + CRC LE (default)
  HELLO_STD_NO_RSP = 1,    // header + payload + CRC LE, WRITE_NO_RSP
  HELLO_STD_BE = 2,        // header + payload + CRC BE
  HELLO_NO_HEADER_LE = 3   // payload + CRC LE (no 55 AA AA 55)
};

class BlackviewLock : public Component, public BLEClientNode {
 public:
  // Expose sensors to YAML
  void set_session_key_text_sensor(TextSensor *t) { session_key_text_ = t; }
  void set_last_notify_text_sensor(TextSensor *t) { last_notify_text_ = t; }
  void set_key_received_binary_sensor(BinarySensor *b) { key_received_ = b; }
  void set_connected_binary_sensor(BinarySensor *b) { connected_ = b; }
  void set_notify_count_sensor(Sensor *s) { notify_count_ = s; }

  void setup() override { ESP_LOGI(TAG, "Setup complete"); }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "BlackviewLock:");
    ESP_LOGCONFIG(TAG, "  write_handle_  : 0x%04X", write_handle_);
    ESP_LOGCONFIG(TAG, "  notify_handle_ : 0x%04X", notify_handle_);
    ESP_LOGCONFIG(TAG, "  cccd_handle_   : 0x%04X", cccd_handle_);
  }

  // callable from HA buttons
  void request_handshake() { send_hello_variant_(HELLO_STD_LE, "user"); }
  void request_handshake_mode(uint8_t mode) { send_hello_variant_((HelloVariant) mode, "user"); }

  // gentle auto-retry during the pairing window
  void loop() override {
    if (parent() != nullptr && parent()->connected() && !got_key_) {
      const uint32_t now = millis();

      // one gentle retry cadence
      if (auto_tries_ < kMaxAutoTries && now - last_hello_ms_ > kHelloIntervalMs) {
        send_hello_variant_(HELLO_STD_LE, "auto");
        auto_tries_++;
        last_hello_ms_ = now;
      }

      // if pairing likely closed without key, disconnect so the lock can re-enter pairing cleanly
      if (now - connect_open_ms_ > kPairingWatchdogMs) {
        ESP_LOGW(TAG, "No key after %u ms, closing connection to reset pairing window", kPairingWatchdogMs);
        esp_ble_gattc_close(parent()->get_gattc_if(), parent()->get_conn_id());
      }
    }
  }

  void gattc_event_handler(esp_gattc_cb_event_t event,
                           esp_gatt_if_t gattc_if,
                           esp_ble_gattc
