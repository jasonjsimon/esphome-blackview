#pragma once

#include <cstring>
#include <string>

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

extern "C" {
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
}

namespace esphome {
namespace blackview_lock {

static const char *const TAG = "blackview_lock";

class BlackviewLock : public Component, public ble_client::BLEClientNode {
 public:
  // ----- Optional sensors (safe to leave null) -----
  void set_session_key_text_sensor(text_sensor::TextSensor *t) { session_key_text_ = t; }
  void set_last_notify_text_sensor(text_sensor::TextSensor *t) { last_notify_text_ = t; }
  void set_key_received_binary_sensor(binary_sensor::BinarySensor *b) { key_received_ = b; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *b) { connected_ = b; }
  void set_notify_count_sensor(sensor::Sensor *s) { notify_count_ = s; }

  // ----- Back-compat no-ops so existing YAML compiles -----
  void set_prefer_write_no_rsp(bool v) { prefer_write_no_rsp_ = v; }
  void set_write_uuid(const std::string &uuid) { write_uuid_ = uuid; }
  void set_notify_uuid(const std::string &uuid) { notify_uuid_ = uuid; }

  // ----- Manual handshake hints (not required for bring-up) -----
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

  void setup() override {}

  void loop() override {
    const uint32_t now = millis();
    // After CCCD enable + small settle, send HELLO exactly once
    if (post_cccd_hello_due_ms_ != 0 && now >= post_cccd_hello_due_ms_ && !hello_sent_after_cccd_) {
      send_hello_(handshake_version_to_try_);
      hello_sent_after_cccd_ = true;
      post_cccd_hello_due_ms_ = 0;
    }
  }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_CONNECT_EVT: {
        connected_flag_ = true;
        std::memcpy(remote_bda_, param->connect.remote_bda, sizeof(remote_bda_));
        have_bda_ = true;
        hello_sent_after_cccd_ = false;
        post_cccd_hello_due_ms_ = 0;
        encryption_requested_ms_ = millis();
        if (connected_ != nullptr) connected_->publish_state(true);

        // Do NOT call request_encryption(); not available on your ESPHome version.
        // Most devices will initiate security from the peripheral side when needed.
        ESP_LOGD(TAG, "Connected; waiting for service discovery -> CCCD -> HELLO (no explicit MITM request).");
        break;
      }

      case ESP_GATTC_OPEN_EVT: {
        ESP_LOGI(TAG, "Connected.");
        break;
      }

      case ESP_GATTC_SEARCH_CMPL_EVT: {
        ESP_LOGI(TAG, "Service discovery complete");
        resolve_handles_();
        auto *cli = this->parent();
        if (cli == nullptr) break;
        enable_notify_(cli->get_gattc_if(), cli->get_conn_id(), /*use_indications=*/true);
        break;
      }

      case ESP_GATTC_WRITE_DESCR_EVT: {
        if (param->write.handle == cccd_handle_) {
          if (param->write.status == ESP_GATT_OK) {
            ESP_LOGD(TAG, "Descriptor write OK (handle 0x%04X)", cccd_handle_);
            pending_conn_id_ = param->write.conn_id;
            pending_gattc_if_ = gattc_if;
            // Give security/CCCD a moment to settle, then send HELLO
            post_cccd_hello_due_ms_ = millis() + post_cccd_delay_ms_;
          } else {
            ESP_LOGW(TAG, "Descriptor write failed (handle 0x%04X, status %d)",
                     cccd_handle_, (int) param->write.status);
            if (last_cccd_used_indications_) {
              // Retry using notifications if indications fail
              enable_notify_(gattc_if, param->write.conn_id, /*use_indications=*/false);
            }
          }
        }
        break;
      }

      case ESP_GATTC_WRITE_CHAR_EVT: {
        if (param->write.handle == write_handle_) {
          if (param->write.status == ESP_GATT_OK) {
            ESP_LOGI(TAG, "Write OK (handle 0x%04X)", write_handle_);
          } else {
            ESP_LOGW(TAG, "Write failed (handle 0x%04X, status %d)",
                     write_handle_, (int) param->write.status);
          }
        }
        break;
      }

      case ESP_GATTC_DISCONNECT_EVT: {
        connected_flag_ = false;
        if (connected_ != nullptr) connected_->publish_state(false);
        ESP_LOGD(TAG, "Disconnected, reason 0x%X", param->disconnect.reason);

        // Prepare next attempt
        hello_sent_after_cccd_ = false;
        post_cccd_hello_due_ms_ = 0;
        // Alternate HELLO version between attempts (v1 <-> v0)
        handshake_version_to_try_ = (handshake_version_to_try_ == 1) ? 0 : 1;
        break;
      }

      default:
        break;
    }
  }

 protected:
  void resolve_handles_() {
    // Fixed from your discovery logs; adjust here if the firmware changes
    write_handle_ = 0x0009;
    notify_handle_ = 0x000B;
    cccd_handle_  = 0x000C;

    ESP_LOGI(TAG, "Service discovery complete; resolving handles...");
    ESP_LOGI(TAG, "Resolved: write_handle=0x%04X, notify_handle=0x%04X", write_handle_, notify_handle_);
    ESP_LOGI(TAG, "CCCD handle 0x%04X; enabling notifications…", cccd_handle_);
  }

  void enable_notify_(esp_gatt_if_t gattc_if, uint16_t conn_id, bool use_indications) {
    // CCCD: Notification=0x0001, Indication=0x0002
    const uint8_t cccd_val[2] = { static_cast<uint8_t>(use_indications ? 0x02 : 0x01), 0x00 };
    last_cccd_used_indications_ = use_indications;

    esp_err_t err = esp_ble_gattc_write_char_descr(
        gattc_if, conn_id, cccd_handle_,
        sizeof(cccd_val), (uint8_t *) cccd_val,
        ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);

    ESP_LOGD(TAG, "esp_ble_gattc_write_char_descr returned %d", (int) err);
  }

  void send_hello_(int version) {
    // Transport exercise / placeholder payloads
    uint8_t buf[18];
    size_t len = 0;

    if (version == 1) {
      len = 16;
      memset(buf, 0, len);
      buf[0] = 0x01;  // v1 marker
    } else {
      len = 18;
      memset(buf, 0, len);
      buf[0] = 0x00;  // v0 marker
    }

    ESP_LOGD(TAG, "[auto%s] HELLO v%d to handle 0x%04X (%u bytes)",
             last_cccd_used_indications_ ? "-v1" : "", version, write_handle_, (unsigned) len);

    auto *cli = this->parent();
    if (cli == nullptr) return;

    esp_ble_gattc_write_char(cli->get_gattc_if(), cli->get_conn_id(), write_handle_, len, buf,
                             ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  }

  // ----- Optional UI sensors -----
  text_sensor::TextSensor *session_key_text_{nullptr};
  text_sensor::TextSensor *last_notify_text_{nullptr};
  binary_sensor::BinarySensor *key_received_{nullptr};
  binary_sensor::BinarySensor *connected_{nullptr};
  sensor::Sensor *notify_count_{nullptr};

  // ----- Settings retained for YAML compatibility -----
  bool prefer_write_no_rsp_{false};
  std::string write_uuid_{};
  std::string notify_uuid_{};

  // ----- Handshake state -----
  bool wants_handshake_{true};
  int handshake_version_to_try_{1};     // Try v1 first, then alternate
  uint8_t handshake_mode_[1]{0};

  // ----- Known handles (from discovery) -----
  uint16_t write_handle_{0x0009};
  uint16_t notify_handle_{0x000B};
  uint16_t cccd_handle_{0x000C};

  // ----- Notify/indicate tracking -----
  bool last_cccd_used_indications_{true};
  uint32_t post_cccd_delay_ms_{1200};
  uint32_t encryption_settle_ms_{1200};   // kept for logging symmetry only
  uint32_t encryption_requested_ms_{0};
  uint32_t post_cccd_hello_due_ms_{0};
  bool hello_sent_after_cccd_{false};

  // ----- Connection book-keeping -----
  bool connected_flag_{false};
  bool have_bda_{false};
  uint8_t remote_bda_[6]{0};
  uint16_t pending_conn_id_{0};
  esp_gatt_if_t pending_gattc_if_{ESP_GATT_IF_NONE};
};

}  // namespace blackview_lock
}  // namespace esphome
