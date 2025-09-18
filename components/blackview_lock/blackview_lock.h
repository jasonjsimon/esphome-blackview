#pragma once
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

// 0x1910 service with two characteristics 0x2B11 (write) and 0x2B10 (notify)
class BlackviewLock : public Component, public ble_client::BLEClientNode {
 public:
  // entity wiring
  void set_session_key_text_sensor(text_sensor::TextSensor *t) { session_key_text_ = t; }
  void set_last_notify_text_sensor(text_sensor::TextSensor *t) { last_notify_text_ = t; }
  void set_key_received_binary_sensor(binary_sensor::BinarySensor *b) { key_received_ = b; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *b) { connected_ = b; }
  void set_notify_count_sensor(sensor::Sensor *s) { notify_count_ = s; }

  // request a handshake from the main loop
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

  // BLE client callbacks (GATTC)
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_CONNECT_EVT: {
        connected_flag_ = true;
        // cache peer address for optional encryption
        memcpy(remote_bda_, param->connect.remote_bda, sizeof(remote_bda_));
        have_bda_ = true;
        if (connected_) connected_->publish_state(true);
        ESP_LOGI(TAG, "Connected.");
        // Proactively request encryption/bonding; Just Works should auto-accept
        if (have_bda_) {
          esp_ble_set_encryption(remote_bda_, ESP_BLE_SEC_ENCRYPT);
        }
        break;
      }
      case ESP_GATTC_OPEN_EVT: {
        // no-op
        break;
      }
      case ESP_GATTC_SEARCH_CMPL_EVT: {
        ESP_LOGI(TAG, "Service discovery complete");
        this->resolve_handles_();
        // Enable notifications (first try indications=0x0002, fallback to notifications=0x0001 on failure)
        this->enable_notify_(gattc_if, param->search_cmpl.conn_id, /*use_indications=*/true);
        // queue a HELLO soon after CCCD attempt
        pending_auto_hello_ = true;
        pending_conn_id_ = param->search_cmpl.conn_id;
        pending_gattc_if_ = gattc_if;
        break;
      }
      case ESP_GATTC_WRITE_DESCR_EVT: {
        // CCCD write result
        if (param->write.status == ESP_GATT_OK) {
          ESP_LOGD(TAG, "Descriptor write OK (handle 0x%04X)", param->write.handle);
          if (pending_auto_hello_) {
            this->send_hello_(gattc_if, param->write.conn_id, /*prefer_write_no_rsp=*/false, "auto");
            pending_auto_hello_ = false;
          }
        } else {
          ESP_LOGW(TAG, "Descriptor write failed (handle 0x%04X, status %d)", param->write.handle, param->write.status);
          // Try switching CCCD mode once (indications -> notifications)
          if (last_cccd_used_indications_) {
            last_cccd_used_indications_ = false;
            this->enable_notify_(gattc_if, param->write.conn_id, /*use_indications=*/false);
          } else {
            // still try HELLO even without notifications; some firmwares allow command-only
            if (pending_auto_hello_) {
              this->send_hello_(gattc_if, param->write.conn_id, /*prefer_write_no_rsp=*/false, "auto-no-cccd");
              pending_auto_hello_ = false;
            }
          }
        }
        break;
      }
      case ESP_GATTC_WRITE_CHAR_EVT: {
        ESP_LOGI(TAG, "Write OK (handle 0x%04X)", param->write.handle);
        break;
      }
      case ESP_GATTC_NOTIFY_EVT: {
        // incoming notification/indication
        notify_counter_++;
        if (notify_count_) notify_count_->publish_state(static_cast<float>(notify_counter_));
        char hex[256] = {0};
        size_t n = param->notify.value_len;
        size_t k = 0;
        for (size_t i = 0; i < n && (k + 3) < sizeof(hex); i++) {
          k += snprintf(hex + k, sizeof(hex) - k, "%02X", param->notify.value[i]);
        }
        if (last_notify_text_) last_notify_text_->publish_state(hex);
        // first 8 bytes received -> mark "key received" once
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
        break;
      }
      default:
        break;
    } // switch
  }

  void loop() override {
    // Send periodic HELLO if connected
    const uint32_t now = millis();
    if (connected_flag_ && (wants_handshake_ || (now - last_hello_ms_ >= hello_interval_ms_))) {
      last_hello_ms_ = now;
      BLEClient *p = this->parent();
      if (!p) return;
      uint16_t conn_id = p->get_conn_id();
      esp_gatt_if_t gi = static_cast<esp_gatt_if_t>(p->get_gattc_if());
      this->send_hello_(gi, conn_id, /*prefer_write_no_rsp=*/false, wants_handshake_ ? "manual" : "auto");
      wants_handshake_ = false;
    }
  }

 protected:
  // hardcoded default handles discovered from snoop; pairing mode may still require encryption
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

  // Build and send a "HELLO" packet (two variants tried in the field)
  void send_hello_(esp_gatt_if_t gattc_if, uint16_t conn_id, bool /*prefer_write_no_rsp*/, const char *why) {
    if (!write_handle_) return;

    // Choose variant based on current mode (toggle between v0 and v1 every call when auto)
    bool use_v1 = toggle_v1_;
    if (wants_handshake_) {
      // on manual request try v1 first
      use_v1 = true;
    }
    toggle_v1_ = !toggle_v1_;

    if (use_v1) {
      // v1: 16-byte frame: A5 E5 00 0C <nonce 8> 00 00 <crc16>
      uint8_t buf[16];
      buf[0] = 0xA5; buf[1] = 0xE5; buf[2] = 0x00; buf[3] = 0x0C;
      // simple rolling nonce from millis()
      uint64_t nonce = (uint64_t)millis();
      for (int i = 0; i < 8; i++) buf[4 + i] = (nonce >> (i*8)) & 0xFF;
      buf[12] = 0x00; buf[13] = 0x00;
      uint16_t crc = crc16_xmodem_(buf, 14);
      buf[14] = (crc >> 8) & 0xFF;
      buf[15] = crc & 0xFF;
      ESP_LOGD(TAG, "[%s-v1] HELLO v1 to handle 0x%04X (16 bytes)", why, write_handle_);
      esp_ble_gattc_write_char(gattc_if, conn_id, write_handle_, sizeof(buf), buf,
                               ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    } else {
      // v0: 18-byte frame: A5 E5 00 0E 00 00 <nonce 8> 00 00 <crc16>
      uint8_t buf[18];
      buf[0] = 0xA5; buf[1] = 0xE5; buf[2] = 0x00; buf[3] = 0x0E;
      buf[4] = 0x00; buf[5] = 0x00;
      uint64_t nonce = (uint64_t)millis();
      for (int i = 0; i < 8; i++) buf[6 + i] = (nonce >> (i*8)) & 0xFF;
      buf[14] = 0x00; buf[15] = 0x00;
      uint16_t crc = crc16_xmodem_(buf, 16);
      buf[16] = (crc >> 8) & 0xFF;
      buf[17] = crc & 0xFF;
      ESP_LOGD(TAG, "[%s] HELLO v0 to handle 0x%04X (18 bytes)", why, write_handle_);
      esp_ble_gattc_write_char(gattc_if, conn_id, write_handle_, sizeof(buf), buf,
                               ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    }
  }

  static uint16_t crc16_xmodem_(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
      crc ^= ((uint16_t)data[i]) << 8;
      for (int b = 0; b < 8; b++) {
        if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
        else crc <<= 1;
      }
    }
    return crc;
  }

  // wiring
  text_sensor::TextSensor *session_key_text_{nullptr};
  text_sensor::TextSensor *last_notify_text_{nullptr};
  binary_sensor::BinarySensor *key_received_{nullptr};
  binary_sensor::BinarySensor *connected_{nullptr};
  sensor::Sensor *notify_count_{nullptr};

  // state
  bool connected_flag_{false};
  bool wants_handshake_{false};
  uint8_t handshake_mode_[1]{0};  // just store a byte for now
  bool key_received_seen_{false};

  // auto hello pacing
  uint32_t last_hello_ms_{0};
  const uint32_t hello_interval_ms_{8000};
  bool toggle_v1_{true};

  // ATT handles
  uint16_t write_handle_{0};
  uint16_t notify_handle_{0};
  uint16_t cccd_handle_{0};

  // notify count
  uint32_t notify_counter_{0};

  // CCCD attempt tracking
  bool last_cccd_used_indications_{true};

  // connection identifiers for deferred send
  bool pending_auto_hello_{false};
  uint16_t pending_conn_id_{0};
  esp_gatt_if_t pending_gattc_if_{ESP_GATT_IF_NONE};

  // peer address for encryption
  esp_bd_addr_t remote_bda_{0};
  bool have_bda_{false};
};

}  // namespace blackview_lock
}  // namespace esphome
