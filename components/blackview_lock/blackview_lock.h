#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"

#include <algorithm>
#include <cstring>
#include <cstdio>

extern "C" {
  #include "esp_gattc_api.h"
  #include "esp_random.h"
}

namespace esphome {
namespace blackview_lock {

static const char *const TAG = "blackview_lock";

// ESP-IDF spelling shim
#if defined(ESP_GATTC_SEARCH_CMP_EVT) && !defined(ESP_GATTC_SEARCH_CMPL_EVT)
#define ESP_GATTC_SEARCH_CMPL_EVT ESP_GATTC_SEARCH_CMP_EVT
#endif
#if defined(ESP_GATTC_SEARCH_CMPL_EVT) && !defined(ESP_GATTC_SEARCH_CMP_EVT)
#define ESP_GATTC_SEARCH_CMP_EVT ESP_GATTC_SEARCH_CMPL_EVT
#endif

class BlackviewLock : public Component, public ble_client::BLEClientNode {
 public:
  // Diagnostics hooks (wired in YAML)
  void set_session_key_text_sensor(text_sensor::TextSensor *t) { session_key_text_ = t; }
  void set_last_notify_text_sensor(text_sensor::TextSensor *t) { last_notify_text_ = t; }
  void set_key_received_binary_sensor(binary_sensor::BinarySensor *b) { key_received_ = b; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *b) { connected_ = b; }
  void set_notify_count_sensor(sensor::Sensor *s) { notify_count_ = s; }

  // No-ops to satisfy generated main.cpp without touching YAML
  void set_prefer_write_no_rsp(bool /*v*/) {}
  void set_write_uuid(const char * /*uuid*/) {}
  void set_notify_uuid(const char * /*uuid*/) {}

  // Control
  void request_handshake() { wants_handshake_ = true; }
  void request_handshake_mode(int mode) { handshake_mode_ = mode; wants_handshake_ = true; }

  // Component
  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Blackview Lock Bridge:");
    ESP_LOGCONFIG(TAG, "  Write handle:  0x%04X", write_handle_);
    ESP_LOGCONFIG(TAG, "  Notify handle: 0x%04X", notify_handle_);
    ESP_LOGCONFIG(TAG, "  CCCD handle:   0x%04X", cccd_handle_);
  }

  void loop() override {
    const uint32_t now = millis();
    // CHANGED: settle delay from 400ms -> 1200ms
    const bool post_cccd_settle = (cccd_enabled_ms_ == 0) || (now - cccd_enabled_ms_ >= 1200);
    if (connected_flag_ && notify_enabled_ && post_cccd_settle &&
        (wants_handshake_ || (now - last_hello_ms_ >= hello_interval_ms_))) {
      auto *p = this->parent();
      if (p != nullptr) {
        send_hello_(p->get_gattc_if(), p->get_conn_id(), "auto-v1");
      }
    }
  }

  // BLEClientNode
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_OPEN_EVT:
      case ESP_GATTC_CONNECT_EVT: {
        connected_flag_ = true;
        if (connected_) connected_->publish_state(true);
        ESP_LOGI(TAG, "Connected.");
        break;
      }

      case ESP_GATTC_SEARCH_CMPL_EVT: {
        ESP_LOGI(TAG, "Service discovery complete");
        ESP_LOGI(TAG, "Service discovery complete; resolving handles...");
        resolve_handles_();
        ESP_LOGI(TAG, "Resolved: write_handle=0x%04X, notify_handle=0x%04X", write_handle_, notify_handle_);
        ESP_LOGI(TAG, "CCCD handle 0x%04X; enabling notifications…", cccd_handle_);
        enable_notify_(gattc_if, param->search_cmpl.conn_id);
        break;
      }

      case ESP_GATTC_WRITE_DESCR_EVT: {
        if (param->write.handle == cccd_handle_ && param->write.status == ESP_GATT_OK) {
          ESP_LOGD(TAG, "Descriptor write OK (handle 0x%04X)", param->write.handle);
          notify_enabled_ = true;
          cccd_enabled_ms_ = millis();
          wants_handshake_ = true;  // loop will send after settle delay
        } else {
          ESP_LOGW(TAG, "Descriptor write failed (handle 0x%04X, status %d)", param->write.handle, (int) param->write.status);
        }
        break;
      }

      case ESP_GATTC_WRITE_CHAR_EVT: {
        ESP_LOGI(TAG, "Write OK (handle 0x%04X)", param->write.handle);
        break;
      }

      case ESP_GATTC_NOTIFY_EVT: {
        notify_counter_++;
        if (notify_count_) notify_count_->publish_state(static_cast<float>(notify_counter_));

        // Hex dump (cap 32 bytes)
        char hex[2 * 32 + 1] = {0};
        size_t n = std::min<size_t>(param->notify.value_len, 32);
        for (size_t i = 0; i < n; i++) std::sprintf(hex + 2 * i, "%02X", param->notify.value[i]);
        if (last_notify_text_) last_notify_text_->publish_state(hex);

        if (param->notify.value_len >= 8 && key_received_ && !key_received_seen_) {
          key_received_seen_ = true;
          key_received_->publish_state(true);
        }
        break;
      }

      case ESP_GATTC_DISCONNECT_EVT: {
        connected_flag_ = false;
        notify_enabled_ = false;
        cccd_enabled_ms_ = 0;
        if (connected_) connected_->publish_state(false);
        ESP_LOGD(TAG, "Disconnected, reason 0x%02X", param->disconnect.reason);
        break;
      }

      default:
        break;
    }
  }

 protected:
  // CRC-16/XMODEM (poly 0x1021, init 0x0000)
  static uint16_t crc_xmodem_(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
      crc ^= (uint16_t)data[i] << 8;
      for (int b = 0; b < 8; b++) {
        if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
        else              crc <<= 1;
      }
    }
    return crc;
  }

  void enable_notify_(esp_gatt_if_t gattc_if, uint16_t conn_id) {
    if (!cccd_handle_) return;
    // CHANGED: enable INDICATIONS (0x0002) instead of NOTIFICATIONS (0x0001)
    const uint8_t en[2] = {0x02, 0x00};
    esp_ble_gattc_write_char_descr(gattc_if, conn_id, cccd_handle_, sizeof(en), (uint8_t *)en,
                                   ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  }

  // Build & write HELLO v1 (18 bytes total):
  //   Header [A5 E5] + Length [00 0E] + Payload(12) + CRC16(payload) big-endian
  void send_hello_(esp_gatt_if_t gattc_if, uint16_t conn_id, const char *label) {
    if (!write_handle_ || gattc_if == ESP_GATT_IF_NONE) return;

    // 12-byte payload: version markers + 10 random bytes
    uint8_t payload[12];
    payload[0] = 0x03;
    payload[1] = 0x04;
    uint32_t r1 = esp_random();
    uint32_t r2 = esp_random();
    uint32_t r3 = esp_random();
    payload[2]  = (uint8_t)(r1 & 0xFF);
    payload[3]  = (uint8_t)((r1 >> 8) & 0xFF);
    payload[4]  = (uint8_t)((r1 >> 16) & 0xFF);
    payload[5]  = (uint8_t)((r1 >> 24) & 0xFF);
    payload[6]  = (uint8_t)(r2 & 0xFF);
    payload[7]  = (uint8_t)((r2 >> 8) & 0xFF);
    payload[8]  = (uint8_t)((r2 >> 16) & 0xFF);
    payload[9]  = (uint8_t)((r2 >> 24) & 0xFF);
    payload[10] = (uint8_t)(r3 & 0xFF);
    payload[11] = (uint8_t)((r3 >> 8) & 0xFF);

    const uint16_t crc = crc_xmodem_(payload, sizeof(payload));

    uint8_t msg[18];
    msg[0] = 0xA5; msg[1] = 0xE5;
    msg[2] = 0x00; msg[3] = 0x0E;  // length includes CRC (14 = 12+2)
    std::memcpy(&msg[4], payload, sizeof(payload));
    msg[16] = (uint8_t)((crc >> 8) & 0xFF);  // big-endian
    msg[17] = (uint8_t)(crc & 0xFF);

    ESP_LOGD(TAG, "[%s] HELLO v1 to handle 0x%04X (18 bytes)", label ? label : "?", write_handle_);

    // Keep NO_RSP for now; if this still fails we'll try WRITE_TYPE_RSP next
    esp_err_t err = esp_ble_gattc_write_char(
        gattc_if, conn_id, write_handle_, sizeof(msg), msg,
        ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);

    if (err != ESP_OK) {
      ESP_LOGW(TAG, "HELLO write failed: err=%d", (int) err);
    }

    last_hello_ms_ = millis();
    wants_handshake_ = false;
  }

  void resolve_handles_() {
    write_handle_  = 0x0009;
    notify_handle_ = 0x000B;
    cccd_handle_   = 0x000C;
  }

  // State
  text_sensor::TextSensor *session_key_text_{nullptr};
  text_sensor::TextSensor *last_notify_text_{nullptr};
  binary_sensor::BinarySensor *key_received_{nullptr};
  binary_sensor::BinarySensor *connected_{nullptr};
  sensor::Sensor *notify_count_{nullptr};

  uint16_t write_handle_{0};
  uint16_t notify_handle_{0};
  uint16_t cccd_handle_{0};

  bool connected_flag_{false};
  bool notify_enabled_{false};
  bool key_received_seen_{false};

  uint32_t notify_counter_{0};
  bool     wants_handshake_{false};
  int      handshake_mode_{0};
  uint32_t last_hello_ms_{0};
  uint32_t cccd_enabled_ms_{0};
  static constexpr uint32_t hello_interval_ms_{8000};
};

}  // namespace blackview_lock
}  // namespace esphome
