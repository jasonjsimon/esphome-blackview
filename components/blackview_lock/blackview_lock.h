#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"

#include <esp_gattc_api.h>
#include <esp_gap_ble_api.h>
#include <cstring>
#include <string>

namespace esphome {
namespace blackview_lock {

static const char *const TAG = "blackview_lock";

/**
 * Blackview SE-series lock GATT client node.
 * Flow: CONNECT → DISCOVERY → register_for_notify (stack writes CCCD) → delay → HELLO (v1, fallback v0) → expect NOTIFY.
 */
class BlackviewLock : public Component, public ble_client::BLEClientNode {
 public:
  // ----- YAML compatibility / setters -----
  void set_prefer_write_no_rsp(bool v) { this->prefer_write_no_rsp_ = v; }
  void set_write_uuid(const std::string &u) { this->write_uuid_ = u; }
  void set_notify_uuid(const std::string &u) { this->notify_uuid_ = u; }

  void set_session_key_text_sensor(text_sensor::TextSensor *t) { this->session_key_text_ = t; }
  void set_last_notify_text_sensor(text_sensor::TextSensor *t) { this->last_notify_text_ = t; }
  void set_key_received_binary_sensor(binary_sensor::BinarySensor *b) { this->key_received_ = b; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *b) { this->connected_bin_ = b; }
  void set_notify_count_sensor(sensor::Sensor *s) { this->notify_count_ = s; }

  // ----- Public actions wired from YAML -----
  void request_handshake() {
    ESP_LOGD(TAG, "request_handshake(): start HELLO sequence");
    hello_attempts_ = 0;
    current_hello_version_ = 1;  // try v1 first
    send_hello_(current_hello_version_);
    hello_retry_due_ms_ = millis() + hello_retry_interval_ms_;
  }
  void request_handshake_mode(int mode) {
    current_hello_version_ = (mode == 2) ? 0 : 1;  // mode 2 = v0-first
    ESP_LOGD(TAG, "request_handshake_mode(%d): trying v%d first", mode, (int) current_hello_version_);
    hello_attempts_ = 0;
    send_hello_(current_hello_version_);
    hello_retry_due_ms_ = millis() + hello_retry_interval_ms_;
  }

  // ----- Component lifecycle -----
void setup() override {
  esp_ble_auth_req_t auth_req = (esp_ble_auth_req_t)(ESP_LE_AUTH_BOND | ESP_LE_AUTH_REQ_SC_ONLY);
  uint8_t key_size = 16;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t resp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &resp_key, sizeof(resp_key));
}

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Blackview Lock");
    ESP_LOGCONFIG(TAG, "  Write handle:  0x%04X", write_handle_);
    ESP_LOGCONFIG(TAG, "  Notify handle: 0x%04X", notify_handle_);
    ESP_LOGCONFIG(TAG, "  CCCD handle:   0x%04X", cccd_handle_);
  }

  // ----- Main loop: timers for encryption + HELLO retries -----
  void loop() override {
    const uint32_t now = millis();

    if (!encryption_requested_ && encryption_request_due_ms_ != 0 && now >= encryption_request_due_ms_) {
      encryption_requested_ = true;
      // Best-effort link encryption (no-MITM). Harmless if peer ignores it.
      esp_err_t er = esp_ble_set_encryption(remote_bda_, ESP_BLE_SEC_ENCRYPT_NO_MITM);
      ESP_LOGD(TAG, "esp_ble_set_encryption(NO_MITM) -> %d", (int) er);
    }

    if (post_cccd_hello_due_ms_ != 0 && now >= post_cccd_hello_due_ms_) {
      post_cccd_hello_due_ms_ = 0;
      hello_attempts_ = 0;
      current_hello_version_ = 1;  // v1 first
      send_hello_(current_hello_version_);
      hello_retry_due_ms_ = now + hello_retry_interval_ms_;
    }

    if (hello_retry_due_ms_ != 0 && now >= hello_retry_due_ms_) {
      if (hello_attempts_ >= hello_attempts_max_) {
        ESP_LOGW(TAG, "No notify after %u HELLO attempts; pausing until next reconnect.", hello_attempts_);
        hello_retry_due_ms_ = 0;
      } else {
        current_hello_version_ ^= 1;  // alternate v1/v0
        send_hello_(current_hello_version_);
        hello_retry_due_ms_ = now + hello_retry_interval_ms_;
      }
    }
  }

  // ----- BLE GATTC handler -----
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_CONNECT_EVT: {
        connected_flag_ = true;
        hello_retry_due_ms_ = 0;
        post_cccd_hello_due_ms_ = 0;
        hello_attempts_ = 0;
        notify_registered_ = false;
        encryption_requested_ = false;
        encryption_request_due_ms_ = millis() + 700;  // small delay before asking for encryption

        if (connected_bin_ != nullptr) connected_bin_->publish_state(true);
        std::memcpy(remote_bda_, param->connect.remote_bda, sizeof(remote_bda_));
        have_bda_ = true;

        ESP_LOGD(TAG, "Connected; waiting for service discovery -> register_for_notify -> CCCD -> HELLO.");
        break;
      }

      case ESP_GATTC_OPEN_EVT: {
        ESP_LOGI(TAG, "Connection open");
        break;
      }

      case ESP_GATTC_SEARCH_CMPL_EVT: {
        ESP_LOGI(TAG, "Service discovery complete");
        resolve_handles_();

        // Ask the stack to route NOTIFYs for our handle; ESPHome's layer will write CCCD=0x0001 (notifications)
        if (have_bda_) {
          esp_err_t r = esp_ble_gattc_register_for_notify(gattc_if, remote_bda_, notify_handle_);
          ESP_LOGD(TAG, "register_for_notify returned %d", (int) r);
        }
        break;
      }

      case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (param->reg_for_notify.handle == notify_handle_) {
          if (param->reg_for_notify.status == ESP_GATT_OK) {
            notify_registered_ = true;
            ESP_LOGD(TAG, "REG_FOR_NOTIFY OK for handle 0x%04X", notify_handle_);
          } else {
            ESP_LOGW(TAG, "REG_FOR_NOTIFY failed (status %d) for handle 0x%04X",
                     (int) param->reg_for_notify.status, notify_handle_);
          }
          // We rely on ESPHome/IDF to write CCCD; we just wait for WRITE_DESCR_EVT on our CCCD handle.
        }
        break;
      }

      case ESP_GATTC_WRITE_DESCR_EVT: {
        // CCCD write completes (done by the stack after register_for_notify)
        if (param->write.handle == cccd_handle_) {
          if (param->write.status == ESP_GATT_OK) {
            ESP_LOGD(TAG, "Descriptor write OK (handle 0x%04X)", cccd_handle_);
            // Give peer a bit to arm notifications, then send HELLO
            post_cccd_hello_due_ms_ = millis() + post_cccd_delay_ms_;
          } else {
            // Common transient error (e.g., 128/133) — we don't second-guess the stack here; it will retry if needed.
            ESP_LOGW(TAG, "Descriptor write failed (handle 0x%04X, status %d)",
                     cccd_handle_, (int) param->write.status);
          }
        }
        break;
      }

      case ESP_GATTC_WRITE_CHAR_EVT: {
        if (param->write.handle == write_handle_) {
          ESP_LOGI(TAG, "Write OK (handle 0x%04X)", write_handle_);
        }
        break;
      }

      case ESP_GATTC_NOTIFY_EVT: {
        const bool is_notify = param->notify.is_notify;
        const uint16_t handle = param->notify.handle;
        const uint8_t *data = param->notify.value;
        const uint16_t len = param->notify.value_len;

        hello_retry_due_ms_ = 0;
        hello_attempts_ = 0;

        notify_seen_count_++;
        if (notify_count_ != nullptr) notify_count_->publish_state((float) notify_seen_count_);

        char buf[64] = {0};
        size_t to_dump = len > 20 ? 20 : len, off = 0;
        for (size_t i = 0; i < to_dump && (off + 3) < sizeof(buf); i++) off += snprintf(buf + off, sizeof(buf) - off, "%02X ", data[i]);
        ESP_LOGD(TAG, "%s on handle 0x%04X len=%u first=%s", is_notify ? "NOTIFY" : "INDICATE", handle, (unsigned) len, buf);

        if (last_notify_text_ != nullptr) {
          std::string hex;
          hex.reserve(len * 3);
          char b[4];
          for (uint16_t i = 0; i < len; i++) {
            snprintf(b, sizeof(b), "%02X", data[i]);
            hex += b;
            if (i + 1 != len) hex += " ";
          }
          last_notify_text_->publish_state(hex.c_str());
        }
        // TODO: decode response / derive session key here.
        break;
      }

      case ESP_GATTC_DISCONNECT_EVT: {
        connected_flag_ = false;
        hello_retry_due_ms_ = 0;
        post_cccd_hello_due_ms_ = 0;
        notify_registered_ = false;
        encryption_requested_ = false;
        encryption_request_due_ms_ = 0;
        if (connected_bin_ != nullptr) connected_bin_->publish_state(false);
        ESP_LOGD(TAG, "Disconnected, reason 0x%X", (unsigned) param->disconnect.reason);
        break;
      }

      default:
        break;
    }
  }

 protected:
  // ----- Helpers -----
  void resolve_handles_() {
    // Fixed from traces:
    //   write  = 0x0009
    //   notify = 0x000B
    //   CCCD   = 0x000C
    write_handle_ = 0x0009;
    notify_handle_ = 0x000B;
    cccd_handle_ = 0x000C;

    ESP_LOGI(TAG, "Service discovery complete; resolving handles...");
    ESP_LOGI(TAG, "Resolved: write_handle=0x%04X, notify_handle=0x%04X", write_handle_, notify_handle_);
    ESP_LOGI(TAG, "CCCD handle 0x%04X; stack will enable notifications…", cccd_handle_);
  }

  // --- REPLACE the old send_hello_ function with this one ---

void send_hello_() {
  auto *cli = this->parent();
  if (cli == nullptr) return;

  // The real "HELLO" payload captured from the BTSnoop log (Packet 1635)
  const uint8_t real_hello_payload[] = {
      0xA5, 0xE5, 0x00, 0x0C, 0x03, 0x04, 0x00, 0x00,
      0x00, 0x01, 0x14, 0xB7, 0x80, 0x03, 0xF5, 0x7A
  };

  uint16_t len = sizeof(real_hello_payload);

  ESP_LOGD(TAG, "[auto] Sending real HELLO to handle 0x%04X (%u bytes)", write_handle_, (unsigned) len);

  esp_err_t r = esp_ble_gattc_write_char(cli->get_gattc_if(), cli->get_conn_id(), write_handle_,
                                         len, (uint8_t*)real_hello_payload,
                                         ESP_GATT_WRITE_TYPE_RSP,
                                         ESP_GATT_AUTH_REQ_NO_MITM);
  if (r == ESP_OK) {
    hello_attempts_++;
  } else {
    ESP_LOGW(TAG, "HELLO write error %d", (int) r);
  }
}
  // ----- Members -----
  uint16_t write_handle_{0x0009};
  uint16_t notify_handle_{0x000B};
  uint16_t cccd_handle_{0x000C};

  uint8_t remote_bda_[6]{0};
  bool have_bda_{false};
  bool connected_flag_{false};
  bool notify_registered_{false};

  // Timers/state
  uint32_t post_cccd_hello_due_ms_{0};
  uint32_t hello_retry_due_ms_{0};
  uint8_t current_hello_version_{1};
  uint8_t hello_attempts_{0};
  uint8_t hello_attempts_max_{10};
  const uint32_t post_cccd_delay_ms_{1500};      // was 800
  const uint32_t hello_retry_interval_ms_{1500}; // was 1000

  // Optional encryption nudge
  uint32_t encryption_request_due_ms_{0};
  bool encryption_requested_{false};

  // Stats / publishing
  uint32_t notify_seen_count_{0};

  // Optional sensors
  text_sensor::TextSensor *session_key_text_{nullptr};
  text_sensor::TextSensor *last_notify_text_{nullptr};
  binary_sensor::BinarySensor *key_received_{nullptr};
  binary_sensor::BinarySensor *connected_bin_{nullptr};
  sensor::Sensor *notify_count_{nullptr};

  // Stored (compat with YAML)
  bool prefer_write_no_rsp_{false};
  std::string write_uuid_;
  std::string notify_uuid_;
};

}  // namespace blackview_lock
}  // namespace esphome


