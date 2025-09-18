#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"

#include <esp_gattc_api.h>
#include <cstring>
#include <string>

namespace esphome {
namespace blackview_lock {

static const char *const TAG = "blackview_lock";

/**
 * Blackview SE60/SE-series BLE lock GATT client node.
 *
 * Flow: CONNECT → DISCOVERY → register_for_notify → write CCCD → send HELLO (v1, fallback v0) → expect NOTIFY/INDICATE.
 */
class BlackviewLock : public Component, public ble_client::BLEClientNode {
 public:
  // ===== YAML compatibility setters (stored/no-ops) =====
  void set_prefer_write_no_rsp(bool v) { this->prefer_write_no_rsp_ = v; }
  void set_write_uuid(const std::string &u) { this->write_uuid_ = u; }
  void set_notify_uuid(const std::string &u) { this->notify_uuid_ = u; }

  void set_session_key_text_sensor(text_sensor::TextSensor *t) { this->session_key_text_ = t; }
  void set_last_notify_text_sensor(text_sensor::TextSensor *t) { this->last_notify_text_ = t; }
  void set_key_received_binary_sensor(binary_sensor::BinarySensor *b) { this->key_received_ = b; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *b) { this->connected_bin_ = b; }
  void set_notify_count_sensor(sensor::Sensor *s) { this->notify_count_ = s; }

  // ===== Public actions used by YAML lambdas =====
  void request_handshake() {
    ESP_LOGD(TAG, "request_handshake(): starting immediate HELLO sequence");
    hello_attempts_ = 0;
    current_hello_version_ = 1;  // try v1 first
    send_hello_(current_hello_version_);
    hello_retry_due_ms_ = millis() + hello_retry_interval_ms_;
  }
  void request_handshake_mode(int mode) {
    // mode 1 = v1 first, mode 2 = v0 first (simple hint)
    current_hello_version_ = (mode == 2) ? 0 : 1;
    ESP_LOGD(TAG, "request_handshake_mode(%d): trying v%d first", mode, (int) current_hello_version_);
    hello_attempts_ = 0;
    send_hello_(current_hello_version_);
    hello_retry_due_ms_ = millis() + hello_retry_interval_ms_;
  }

  // ===== Component lifecycle =====
  void setup() override {}
  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Blackview Lock");
    ESP_LOGCONFIG(TAG, "  Write handle:  0x%04X", write_handle_);
    ESP_LOGCONFIG(TAG, "  Notify handle: 0x%04X", notify_handle_);
    ESP_LOGCONFIG(TAG, "  CCCD handle:   0x%04X", cccd_handle_);
  }

  // ===== Main loop: schedule HELLO send / retry =====
  void loop() override {
    const uint32_t now = millis();

    // First HELLO after CCCD enable
    if (post_cccd_hello_due_ms_ != 0 && now >= post_cccd_hello_due_ms_) {
      post_cccd_hello_due_ms_ = 0;
      hello_attempts_ = 0;
      current_hello_version_ = 1;  // v1 first
      send_hello_(current_hello_version_);
      hello_retry_due_ms_ = now + hello_retry_interval_ms_;
    }

    // Alternate HELLOs if no NOTIFY yet
    if (hello_retry_due_ms_ != 0 && now >= hello_retry_due_ms_) {
      if (hello_attempts_ >= hello_attempts_max_) {
        ESP_LOGW(TAG, "No notify after %u HELLO attempts; pausing until next reconnect.", hello_attempts_);
        hello_retry_due_ms_ = 0;
      } else {
        current_hello_version_ ^= 1;  // flip between 1 and 0
        send_hello_(current_hello_version_);
        hello_retry_due_ms_ = now + hello_retry_interval_ms_;
      }
    }
  }

  // ===== BLE GATTC handler =====
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_CONNECT_EVT: {
        connected_flag_ = true;
        hello_retry_due_ms_ = 0;
        post_cccd_hello_due_ms_ = 0;
        hello_attempts_ = 0;
        notify_registered_ = false;
        last_cccd_used_indications_ = true;
        if (connected_bin_ != nullptr) connected_bin_->publish_state(true);

        std::memcpy(remote_bda_, param->connect.remote_bda, sizeof(remote_bda_));
        have_bda_ = true;

        ESP_LOGD(TAG, "Connected; waiting for service discovery -> CCCD -> HELLO (no explicit MITM request).");
        break;
      }

      case ESP_GATTC_OPEN_EVT: {
        ESP_LOGI(TAG, "Connection open");
        break;
      }

      case ESP_GATTC_SEARCH_CMPL_EVT: {
        ESP_LOGI(TAG, "Service discovery complete");
        resolve_handles_();

        // Step 1: register so NOTIFY events are delivered
        if (have_bda_) {
          esp_err_t r = esp_ble_gattc_register_for_notify(gattc_if, remote_bda_, notify_handle_);
          ESP_LOGD(TAG, "register_for_notify returned %d", (int) r);
          if (r != ESP_OK) {
            // proceed to CCCD write anyway
            enable_notify_(gattc_if, param->search_cmpl.conn_id, /*use_indications=*/true);
          }
        } else {
          enable_notify_(gattc_if, param->search_cmpl.conn_id, /*use_indications=*/true);
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
          // Use conn_id from BLEClient wrapper (struct lacks conn_id here)
          auto *cli = this->parent();
          uint16_t conn_id = cli ? cli->get_conn_id() : 0;
          enable_notify_(gattc_if, conn_id, /*use_indications=*/true);
        }
        break;
      }

      case ESP_GATTC_WRITE_DESCR_EVT: {
        // CCCD write result
        if (param->write.handle == cccd_handle_) {
          if (param->write.status == ESP_GATT_OK) {
            ESP_LOGD(TAG, "Descriptor write OK (handle 0x%04X)", cccd_handle_);
            // Schedule first HELLO shortly after CCCD settles
            post_cccd_hello_due_ms_ = millis() + post_cccd_delay_ms_;
          } else {
            // 0x80/133 and friends are common transient errors—flip indication/notification and retry
            ESP_LOGW(TAG, "Descriptor write failed (handle 0x%04X, status %d)",
                     cccd_handle_, (int) param->write.status);

            auto *cli = this->parent();
            uint16_t conn_id = cli ? cli->get_conn_id() : param->write.conn_id;

            if (last_cccd_used_indications_) {
              last_cccd_used_indications_ = false;
              enable_notify_(gattc_if, conn_id, /*use_indications=*/false);
            } else {
              last_cccd_used_indications_ = true;
              enable_notify_(gattc_if, conn_id, /*use_indications=*/true);
            }
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
        // Handles both notifications and indications; see is_notify flag
        const bool is_notify = param->notify.is_notify;
        const uint8_t *data = param->notify.value;
        const uint16_t len = param->notify.value_len;

        // Stop HELLO retries
        hello_retry_due_ms_ = 0;
        hello_attempts_ = 0;

        notify_seen_count_++;
        if (notify_count_ != nullptr) notify_count_->publish_state((float) notify_seen_count_);

        // Log first bytes to help with decoding
        char buf[64] = {0};
        size_t to_dump = len > 20 ? 20 : len;
        size_t off = 0;
        for (size_t i = 0; i < to_dump && (off + 3) < sizeof(buf); i++) {
          off += snprintf(buf + off, sizeof(buf) - off, "%02X ", data[i]);
        }
        ESP_LOGD(TAG, "%s len=%u first=%s", is_notify ? "NOTIFY" : "INDICATE", (unsigned) len, buf);

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

        // TODO: parse handshake/session here. If a session key is derived:
        // if (session_key_text_) session_key_text_->publish_state(key_string);
        // if (key_received_) key_received_->publish_state(true);
        break;
      }

      case ESP_GATTC_DISCONNECT_EVT: {
        connected_flag_ = false;
        hello_retry_due_ms_ = 0;
        post_cccd_hello_due_ms_ = 0;
        notify_registered_ = false;
        if (connected_bin_ != nullptr) connected_bin_->publish_state(false);
        ESP_LOGD(TAG, "Disconnected, reason 0x%X", (unsigned) param->disconnect.reason);
        break;
      }

      default:
        break;
    }
  }

 protected:
  // ===== Helpers =====
  void resolve_handles_() {
    // Fixed handles from your traces:
    //   write  = 0x0009
    //   notify = 0x000B
    //   CCCD   = 0x000C
    write_handle_ = 0x0009;
    notify_handle_ = 0x000B;
    cccd_handle_ = 0x000C;

    ESP_LOGI(TAG, "Service discovery complete; resolving handles...");
    ESP_LOGI(TAG, "Resolved: write_handle=0x%04X, notify_handle=0x%04X", write_handle_, notify_handle_);
    ESP_LOGI(TAG, "CCCD handle 0x%04X; enabling notifications…", cccd_handle_);
  }

  void enable_notify_(esp_gatt_if_t gattc_if, uint16_t conn_id, bool use_indications) {
    // 0x0001 = notifications, 0x0002 = indications
    const uint8_t v = static_cast<uint8_t>(use_indications ? 0x02 : 0x01);
    const uint8_t cccd_val[2] = {v, 0x00};
    last_cccd_used_indications_ = use_indications;

    esp_err_t r = esp_ble_gattc_write_char_descr(gattc_if, conn_id, cccd_handle_,
                                                 sizeof(cccd_val),
                                                 const_cast<uint8_t *>(cccd_val),
                                                 ESP_GATT_WRITE_TYPE_RSP,
                                                 ESP_GATT_AUTH_REQ_NONE);
    ESP_LOGD(TAG, "esp_ble_gattc_write_char_descr returned %d", (int) r);
  }

  void send_hello_(int version /* 1 or 0 */) {
    auto *cli = this->parent();
    if (cli == nullptr) return;

    // v1 = 16 bytes, v0 = 18 bytes (placeholders)
    uint8_t payload[18] = {0};
    uint16_t len = 0;

    if (version == 1) {
      const uint8_t v1[16] = {0x48, 0x45, 0x4C, 0x4C, 0x4F, 0x31, 0x00, 0x00,
                              0xAA, 0x55, 0xAA, 0x55, 0x10, 0x00, 0x00, 0x00};
      std::memcpy(payload, v1, sizeof(v1));
      len = sizeof(v1);
      ESP_LOGD(TAG, "[auto] HELLO v1 to handle 0x%04X (%u bytes)", write_handle_, (unsigned) len);
    } else {
      const uint8_t v0[18] = {0x48, 0x45, 0x4C, 0x4C, 0x4F, 0x30, 0x00, 0x00,
                              0xAA, 0x55, 0xAA, 0x55, 0x12, 0x00, 0x00, 0x00,
                              0x00, 0x00};
      std::memcpy(payload, v0, sizeof(v0));
      len = sizeof(v0);
      ESP_LOGD(TAG, "[auto] HELLO v0 to handle 0x%04X (%u bytes)", write_handle_, (unsigned) len);
    }

    esp_err_t r = esp_ble_gattc_write_char(cli->get_gattc_if(), cli->get_conn_id(), write_handle_,
                                           len, payload,
                                           ESP_GATT_WRITE_TYPE_RSP,
                                           ESP_GATT_AUTH_REQ_NONE);
    if (r == ESP_OK) {
      hello_attempts_++;
    } else {
      ESP_LOGW(TAG, "HELLO write error %d", (int) r);
    }
  }

  // ===== Members =====
  // Handles (fixed for observed FW)
  uint16_t write_handle_{0x0009};
  uint16_t notify_handle_{0x000B};
  uint16_t cccd_handle_{0x000C};

  // BLE peer info
  uint8_t remote_bda_[6]{0};
  bool have_bda_{false};
  bool connected_flag_{false};

  // Notify registration + CCCD state
  bool notify_registered_{false};
  bool last_cccd_used_indications_{true};

  // HELLO scheduling
  uint32_t post_cccd_hello_due_ms_{0};
  uint32_t hello_retry_due_ms_{0};
  uint8_t current_hello_version_{1};
  uint8_t hello_attempts_{0};
  uint8_t hello_attempts_max_{10};             // cap retries
  const uint32_t post_cccd_delay_ms_{800};     // wait a bit after CCCD write
  const uint32_t hello_retry_interval_ms_{1000};

  // Stats / publishing
  uint32_t notify_seen_count_{0};

  // Optional sensors/text sensors set from YAML
  text_sensor::TextSensor *session_key_text_{nullptr};
  text_sensor::TextSensor *last_notify_text_{nullptr};
  binary_sensor::BinarySensor *key_received_{nullptr};
  binary_sensor::BinarySensor *connected_bin_{nullptr};
  sensor::Sensor *notify_count_{nullptr};

  // Stored but unused (compat with YAML/generated code)
  bool prefer_write_no_rsp_{false};
  std::string write_uuid_;
  std::string notify_uuid_;
};

}  // namespace blackview_lock
}  // namespace esphome
