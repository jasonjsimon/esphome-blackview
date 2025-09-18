#pragma once
#include <cstring>
#include <cstdio>
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"

#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_defs.h"

namespace esphome {
namespace blackview_lock {

static const char *const TAG = "blackview_lock";

// Blackview lock: Service 0x1910, write char 0x2B11 (handle 0x0009), notify char 0x2B10 (0x000B), CCCD 0x000C.
class BlackviewLock : public Component, public ble_client::BLEClientNode {
 public:
  // Entity wiring
  void set_session_key_text_sensor(text_sensor::TextSensor *t) { session_key_text_ = t; }
  void set_last_notify_text_sensor(text_sensor::TextSensor *t) { last_notify_text_ = t; }
  void set_key_received_binary_sensor(binary_sensor::BinarySensor *b) { key_received_ = b; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *b) { connected_ = b; }
  void set_notify_count_sensor(sensor::Sensor *s) { notify_count_ = s; }

  // Back-compat no-ops so you don’t need to edit YAML / main.cpp
  void set_prefer_write_no_rsp(bool) {}
  void set_write_uuid(const char *) {}
  void set_notify_uuid(const char *) {}

  // Public API
  void request_handshake() { wants_handshake_ = true; }
  void request_handshake_mode(int mode) {
    handshake_mode_[0] = static_cast<uint8_t>(mode & 0xFF);
    wants_handshake_ = true;
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Blackview Lock");
    ESP_LOGCONFIG(TAG, "  Write handle:  0x%04X", write_handle_);
    ESP_LOGCONFIG(TAG, "  Notify handle: 0x%04X", notify_handle_);
    ESP_LOGCONFIG(TAG, "  CCCD handle:   0x%04X", cccd_handle_);
  }

  // BLE GATTC events
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_CONNECT_EVT: {
        connected_flag_ = true;
        std::memcpy(remote_bda_, param->connect.remote_bda, sizeof(remote_bda_));
        have_bda_ = true;
        if (connected_) connected_->publish_state(true);
        ESP_LOGI(TAG, "Connected.");
        // Do not force link encryption here.
        break;
      }
      case ESP_GATTC_SEARCH_CMPL_EVT: {
        ESP_LOGI(TAG, "Service discovery complete");
        this->resolve_handles_();
        // Try indications first; if it fails we fall back to notifications.
        this->enable_notify_(gattc_if, param->search_cmpl.conn_id, /*use_indications=*/true);
        // We now defer HELLO slightly until loop() sees post_cccd_hello_due_ms_.
        break;
      }
      case ESP_GATTC_WRITE_DESCR_EVT: {
        if (param->write.status == ESP_GATT_OK) {
          ESP_LOGD(TAG, "Descriptor write OK (handle 0x%04X)", param->write.handle);
          // Defer HELLO ~120ms after CCCD success to avoid immediate controller back-to-back writes.
          post_cccd_hello_due_ms_ = millis() + post_cccd_delay_ms_;
          pending_conn_id_ = param->write.conn_id;
          pending_gattc_if_ = gattc_if;
        } else {
          ESP_LOGW(TAG, "Descriptor write failed (handle 0x%04X, status %d)", param->write.handle, param->write.status);
          // Fallback: indications -> notifications
          if (last_cccd_used_indications_) {
            last_cccd_used_indications_ = false;
            this->enable_notify_(gattc_if, param->write.conn_id, /*use_indications=*/false);
          } else {
            // If even notifications fail, attempt HELLO anyway (some firmwares still accept writes).
            post_cccd_hello_due_ms_ = millis() + post_cccd_delay_ms_;
            pending_conn_id_ = param->write.conn_id;
            pending_gattc_if_ = gattc_if;
          }
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
        char hex[256] = {0};
        size_t n = param->notify.value_len;
        size_t k = 0;
        for (size_t i = 0; i < n && (k + 3) < sizeof(hex); i++) {
          k += std::snprintf(hex + k, sizeof(hex) - k, "%02X", param->notify.value[i]);
        }
        if (last_notify_text_) last_notify_text_->publish_state(hex);
        if (n >= 8 && key_received_ && !key_received_seen_) {
          key_received_seen_ = true;
          key_received_->publish_state(true);
          if (session_key_text_) session_key_text_->publish_state(hex);
        }
        break;
      }
      case ESP_GATTC_DISCONNECT_EVT: {
        connected_flag_ = false;
        if (connected_) connected_->publish_state(false);
        ESP_LOGD(TAG, "Disconnected, reason 0x%X", param->disconnect.reason);
        // Clear any pending HELLO
        post_cccd_hello_due_ms_ = 0;
        break;
      }
      default:
        break;
    }
  }

  void loop() override {
    const uint32_t now = millis();

    // Send the deferred HELLO once the small post-CCCD delay has elapsed.
    if (connected_flag_ && post_cccd_hello_due_ms_ && now >= post_cccd_hello_due_ms_) {
      post_cccd_hello_due_ms_ = 0;
      this->send_hello_(pending_gattc_if_, pending_conn_id_, /*prefer_write_no_rsp=*/true, "auto-v1");
    }

    if (connected_flag_ && (wants_handshake_ || (now - last_hello_ms_ >= hello_interval_ms_))) {
      last_hello_ms_ = now;
      ble_client::BLEClient *p = this->parent();
      if (!p) return;
      const uint16_t conn_id = p->get_conn_id();
      const esp_gatt_if_t gi = static_cast<esp_gatt_if_t>(p->get_gattc_if());
      this->send_hello_(gi, conn_id, /*prefer_write_no_rsp=*/true, wants_handshake_ ? "manual" : "auto");
      wants_handshake_ = false;
    }
  }

 protected:
  void resolve_handles_() {
    write_handle_  = 0x0009;
    notify_handle_ = 0x000B;
    cccd_handle_   = 0x000C;
  }

  void enable_notify_(esp_gatt_if_t gattc_if, uint16_t conn_id, bool use_indications) {
    if (!cccd_handle_) return;
    uint8_t en[2] = { static_cast<uint8_t>(use_indications ? 0x02 : 0x01), 0x00 };
    last_cccd_used_indications_ = use_indications;
    esp_ble_gattc_write_char_descr(gattc_if, conn_id, cccd_handle_, sizeof(en), en,
                                   ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  }

  void send_hello_(esp_gatt_if_t gattc_if, uint16_t conn_id, bool prefer_write_no_rsp, const char *why) {
    if (!write_handle_) return;

    // Alternate the 16B and 18B frames to cover both firmware variants.
    bool use_v1 = toggle_v1_;
    if (wants_handshake_) use_v1 = true;  // on manual request, prefer v1
    toggle_v1_ = !toggle_v1_;

    if (use_v1) {
      // 16-byte frame
      uint8_t buf[16];
      buf[0] = 0xA5; buf[1] = 0xE5; buf[2] = 0x00; buf[3] = 0x0C;
      uint64_t nonce = (uint64_t) millis();
      for (int i = 0; i < 8; i++) buf[4 + i] = (nonce >> (i * 8)) & 0xFF;
      buf[12] = 0x00; buf[13] = 0x00;
      uint16_t crc = crc16_xmodem_(buf, 14);
      buf[14] = (crc >> 8) & 0xFF;
      buf[15] = crc & 0xFF;
      ESP_LOGD(TAG, "[%s-v1] HELLO v1 to handle 0x%04X (16 bytes)", why, write_handle_);
      esp_ble_gattc_write_char(gattc_if, conn_id, write_handle_, sizeof(buf), buf,
                               prefer_write_no_rsp ? ESP_GATT_WRITE_TYPE_NO_RSP : ESP_GATT_WRITE_TYPE_RSP,
                               ESP_GATT_AUTH_REQ_NONE);
    } else {
      // 18-byte frame
      uint8_t buf[18];
      buf[0] = 0xA5; buf[1] = 0xE5; buf[2] = 0x00; buf[3] = 0x0E;
      buf[4] = 0x00; buf[5] = 0x00;
      uint64_t nonce = (uint64_t) millis();
      for (int i = 0; i < 8; i++) buf[6 + i] = (nonce >> (i * 8)) & 0xFF;
      buf[14] = 0x00; buf[15] = 0x00;
      uint16_t crc = crc16_xmodem_(buf, 16);
      buf[16] = (crc >> 8) & 0xFF;
      buf[17] = crc & 0xFF;
      ESP_LOGD(TAG, "[%s] HELLO v0 to handle 0x%04X (18 bytes)", why, write_handle_);
      esp_ble_gattc_write_char(gattc_if, conn_id, write_handle_, sizeof(buf), buf,
                               prefer_write_no_rsp ? ESP_GATT_WRITE_TYPE_NO_RSP : ESP_GATT_WRITE_TYPE_RSP,
                               ESP_GATT_AUTH_REQ_NONE);
    }
  }

  static uint16_t crc16_xmodem_(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
      crc ^= ((uint16_t) data[i]) << 8;
      for (int b = 0; b < 8; b++) {
        if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
        else crc <<= 1;
      }
    }
    return crc;
  }

  // Entities
  text_sensor::TextSensor *session_key_text_{nullptr};
  text_sensor::TextSensor *last_notify_text_{nullptr};
  binary_sensor::BinarySensor *key_received_{nullptr};
  binary_sensor::BinarySensor *connected_{nullptr};
  sensor::Sensor *notify_count_{nullptr};

  // State
  bool connected_flag_{false};
  bool wants_handshake_{false};
  uint8_t handshake_mode_[1]{0};
  bool key_received_seen_{false};

  // Auto hello pacing
  uint32_t last_hello_ms_{0};
  const uint32_t hello_interval_ms_{8000};
  bool toggle_v1_{true};

  // Post-CCCD deferred HELLO
  uint32_t post_cccd_hello_due_ms_{0};
  static constexpr uint32_t post_cccd_delay_ms_{120};
  uint16_t pending_conn_id_{0};
  esp_gatt_if_t pending_gattc_if_{0};

  // Handles
  uint16_t write_handle_{0};
  uint16_t notify_handle_{0};
  uint16_t cccd_handle_{0};

  // Notify count
  uint32_t notify_counter_{0};

  // CCCD tracking
  bool last_cccd_used_indications_{true};

  // Peer address (kept for possible future use)
  esp_bd_addr_t remote_bda_{0};
  bool have_bda_{false};
};

}  // namespace blackview_lock
}  // namespace esphome
