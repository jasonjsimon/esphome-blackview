#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"

extern "C" {
  #include "esp_gattc_api.h"
}

namespace esphome {
namespace blackview_lock {

static const char *const TAG = "blackview_lock";

// Normalize IDF enum spelling differences (CMP vs CMPL).
#ifndef ESP_GATTC_SEARCH_CMPL_EVT
  #ifdef ESP_GATTC_SEARCH_CMP_EVT
    #define ESP_GATTC_SEARCH_CMPL_EVT ESP_GATTC_SEARCH_CMP_EVT
  #endif
#endif

class BlackviewLock : public Component, public ble_client::BLEClientNode {
 public:
  // Wire-up to HA entities
  void set_session_key_text_sensor(text_sensor::TextSensor *t) { session_key_text_ = t; }
  void set_last_notify_text_sensor(text_sensor::TextSensor *t) { last_notify_text_ = t; }
  void set_key_received_binary_sensor(binary_sensor::BinarySensor *b) { key_received_ = b; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *b) { connected_ = b; }
  void set_notify_count_sensor(sensor::Sensor *s) { notify_count_ = s; }

  // Back-compat no-ops so older YAML/main compile cleanly
  void set_prefer_write_no_rsp(bool) {}
  void set_write_uuid(const std::string &) {}
  void set_notify_uuid(const std::string &) {}

  // Manual triggers from template buttons
  void request_handshake() { wants_handshake_ = true; }
  void request_handshake_mode(int mode) { handshake_mode_ = mode; wants_handshake_ = true; }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Blackview Lock:");
    ESP_LOGCONFIG(TAG, "  Write handle:  0x%04X", write_handle_);
    ESP_LOGCONFIG(TAG, "  Notify handle: 0x%04X", notify_handle_);
    ESP_LOGCONFIG(TAG, "  CCCD handle:   0x%04X", cccd_handle_);
  }

 protected:
  // Must be 'void' to match BLEClientNode
  void gattc_event_handler(esp_gattc_cb_event_t event,
                           esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_OPEN_EVT: {
        if (param->open.status == ESP_GATT_OK) {
          connected_flag_ = true;
          if (connected_) connected_->publish_state(true);
          ESP_LOGI(TAG, "Connected! Sending HELLO immediately...");
          this->send_hello_(gattc_if, param->open.conn_id, /*no_rsp*/ true, "fallback");
        } else {
          ESP_LOGW(TAG, "OPEN_EVT status=%d", param->open.status);
        }
        break;
      }

      case ESP_GATTC_SEARCH_CMPL_EVT: {
        if (param->search_cmpl.status == ESP_GATT_OK) {
          ESP_LOGI(TAG, "Service discovery complete; resolving handles...");
          this->resolve_handles_();
          if (cccd_handle_ && notify_handle_) {
            this->enable_notify_(gattc_if, param->search_cmpl.conn_id);
          }
          if (write_handle_) {
            this->send_hello_(gattc_if, param->search_cmpl.conn_id, /*no_rsp*/ true, "resolved");
          }
        } else {
          ESP_LOGW(TAG, "SEARCH_CMPL status=%d", param->search_cmpl.status);
        }
        break;
      }

      case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        // Descriptor write confirmation will arrive as WRITE_DESCR_EVT
        break;
      }

      case ESP_GATTC_NOTIFY_EVT: {
        notify_counter_++;
        if (notify_count_) notify_count_->publish_state(notify_counter_);

        // Hex dump for debugging
        std::string hex;
        hex.reserve(param->notify.value_len * 2);
        for (int i = 0; i < param->notify.value_len; i++) {
          char b[3];
          snprintf(b, sizeof(b), "%02X", param->notify.value[i]);
          hex += b;
        }
        if (last_notify_text_) last_notify_text_->publish_state(hex);

        // Mark key-received on first plausible payload
        if (param->notify.value_len >= 8 && key_received_ && !key_received_seen_) {
          key_received_->publish_state(true);
          key_received_seen_ = true;
        }
        break;
      }

      case ESP_GATTC_WRITE_CHAR_EVT: {
        if (param->write.status == ESP_GATT_OK) {
          ESP_LOGI(TAG, "Write OK (handle 0x%04X)", param->write.handle);
        } else {
          ESP_LOGW(TAG, "Write FAILED (handle 0x%04X) status=%d", param->write.handle, param->write.status);
        }
        break;
      }

      case ESP_GATTC_WRITE_DESCR_EVT: {
        if (param->write.status == ESP_GATT_OK) {
          ESP_LOGD(TAG, "Descriptor write OK (handle 0x%04X)", param->write.handle);
        } else {
          ESP_LOGW(TAG, "Descriptor write FAILED (handle 0x%04X) status=%d", param->write.handle, param->write.status);
        }
        break;
      }

      case ESP_GATTC_CLOSE_EVT:
      case ESP_GATTC_DISCONNECT_EVT: {
        connected_flag_ = false;
        if (connected_) connected_->publish_state(false);
        ESP_LOGI(TAG, "Disconnected.");
        break;
      }

      default:
        break;
    }
  }

  void loop() override {
    const uint32_t now = millis();

    // Only send periodic HELLOs when we know we're connected
    if (connected_flag_ && (wants_handshake_ || (now - last_hello_ms_ >= hello_interval_ms_))) {
      // parent() is safe to use for IDs while connected
      this->send_hello_(this->parent()->get_gattc_if(),
                        this->parent()->get_conn_id(),
                        /*no_rsp*/ true,
                        wants_handshake_ ? "manual" : "auto");
      last_hello_ms_ = now;
      wants_handshake_ = false;
    }
  }

  // For this device family (from btsnoop):
  //   Service 0x1910:
  //     Char 0x2B11 (Write/WriteNR)  -> handle 0x0009
  //     Char 0x2B10 (Notify)         -> handle 0x000B
  //     CCCD                         -> handle 0x000C
  void resolve_handles_() {
    write_handle_  = 0x0009;
    notify_handle_ = 0x000B;
    cccd_handle_   = 0x000C;

    ESP_LOGI(TAG, "Resolved: write_handle=0x%04X, notify_handle=0x%04X", write_handle_, notify_handle_);
    if (cccd_handle_) ESP_LOGI(TAG, "CCCD handle 0x%04X; enabling notifications…", cccd_handle_);
  }

  void enable_notify_(esp_gatt_if_t gattc_if, uint16_t conn_id) {
    if (!cccd_handle_) return;
    uint8_t en[2] = {0x01, 0x00};  // notifications on
    esp_ble_gattc_write_char_descr(gattc_if, conn_id, cccd_handle_, sizeof(en), en,
                                   ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  }

  // Minimal probe/hello payload (18 bytes: 16 + CRC-XMODEM LE 2B)
  void send_hello_(esp_gatt_if_t gattc_if, uint16_t conn_id, bool no_rsp, const char *label) {
    if (!write_handle_) return;

    uint8_t buf[18] = {0};
    uint16_t crc = crc_xmodem_(buf, 16);
    buf[16] = static_cast<uint8_t>(crc & 0xFF);
    buf[17] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    esp_ble_gattc_write_char(gattc_if, conn_id, write_handle_, sizeof(buf), buf,
                             no_rsp ? ESP_GATT_WRITE_TYPE_NO_RSP : ESP_GATT_WRITE_TYPE_RSP,
                             ESP_GATT_AUTH_REQ_NONE);

    ESP_LOGD(TAG, "[%s] HELLO v0 to handle 0x%04X (18 bytes)", label, write_handle_);
  }

  static uint16_t crc_xmodem_(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
      crc ^= static_cast<uint16_t>(data[i]) << 8;
      for (int b = 0; b < 8; b++) {
        if (crc & 0x8000)
          crc = (crc << 1) ^ 0x1021;
        else
          crc <<= 1;
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
  uint16_t write_handle_{0x0000};
  uint16_t notify_handle_{0x0000};
  uint16_t cccd_handle_{0x0000};
  uint32_t notify_counter_{0};
  uint32_t last_hello_ms_{0};
  uint32_t hello_interval_ms_{8000};
  bool wants_handshake_{false};
  bool key_received_seen_{false};
  bool connected_flag_{false};
  int handshake_mode_{0};  // currently unused, reserved
};

}  // namespace blackview_lock
}  // namespace esphome
