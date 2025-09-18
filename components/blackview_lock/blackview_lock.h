#pragma once
#ifdef USE_ESP32

#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"

#include "esp_gattc_api.h"  // IDF types

namespace esphome {
namespace blackview_lock {

static const char *const TAG = "blackview_lock";

// Pick exactly one "search complete" event symbol across IDF variants.
#if defined(ESP_GATTC_SEARCH_CMP_EVT)
  #define GATTC_SEARCH_COMPLETE_EVT ESP_GATTC_SEARCH_CMP_EVT
#elif defined(ESP_GATTC_SEARCH_CMPL_EVT)
  #define GATTC_SEARCH_COMPLETE_EVT ESP_GATTC_SEARCH_CMPL_EVT
#else
  #error "Neither ESP_GATTC_SEARCH_CMP_EVT nor ESP_GATTC_SEARCH_CMPL_EVT is defined"
#endif

class BlackviewLock : public Component, public ble_client::BLEClientNode {
 public:
  // ====== Setters wired from codegen (keep names to satisfy generated main.cpp) ======
  void set_session_key_text_sensor(text_sensor::TextSensor *t) { session_key_text_ = t; }
  void set_last_notify_text_sensor(text_sensor::TextSensor *t) { last_notify_text_ = t; }
  void set_key_received_binary_sensor(binary_sensor::BinarySensor *b) { key_received_ = b; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *b) { connected_ = b; }
  void set_notify_count_sensor(sensor::Sensor *s) { notify_count_ = s; }

  void set_prefer_write_no_rsp(bool v) { prefer_write_no_rsp_ = v; }
  void set_write_uuid(const std::string &uuid) { write_uuid_ = uuid; }
  void set_notify_uuid(const std::string &uuid) { notify_uuid_ = uuid; }

  void set_fallback_write_handle(uint16_t h) { fb_write_ = h; }
  void set_fallback_notify_handle(uint16_t h) { fb_notify_ = h; }
  void set_fallback_cccd_handle(uint16_t h) { fb_cccd_ = h; }

  // ====== Component overrides ======
  void loop() override {
    const uint32_t now = millis();
    if (parent() == nullptr)
      return;

    // Periodically re-send HELLO while we haven't seen a key yet
    if (this->connected_flag_ && !this->got_key_) {
      if (now - this->last_hello_ms_ >= 8000) {
        this->last_hello_ms_ = now;
        send_hello_("auto");
      }
    }
  }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_OPEN_EVT: {
        if (param->open.status == ESP_GATT_OK) {
          this->connected_flag_ = true;
          if (connected_ != nullptr) connected_->publish_state(true);
          ESP_LOGI(TAG, "Connection open");

          // If we have fallbacks, try to subscribe & send immediately.
          if (fb_write_ && fb_notify_ && fb_cccd_) {
            early_subscribe_(gattc_if, param->open.conn_id);
            send_hello_("fallback");
          }
        } else {
          ESP_LOGW(TAG, "OPEN status=%d", param->open.status);
        }
        break;
      }

      case GATTC_SEARCH_COMPLETE_EVT: {
        if (param->search_cmpl.status == ESP_GATT_OK) {
          ESP_LOGI(TAG, "Service discovery complete; resolving handles...");

          // If no explicit handles were provided, use the ones from your btsnoop
          if (!fb_write_)  fb_write_  = 0x0009;
          if (!fb_notify_) fb_notify_ = 0x000B;
          if (!fb_cccd_)   fb_cccd_   = 0x000C;

          // Subscribe and send HELLO.
          early_subscribe_(gattc_if, param->search_cmpl.conn_id);
          send_hello_("resolved");
        } else {
          ESP_LOGW(TAG, "SEARCH_CMPL status=%d", param->search_cmpl.status);
        }
        break;
      }

      case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGD(TAG, "Registered for notify, status=%d", param->reg_for_notify.status);
        break;
      }

      case ESP_GATTC_WRITE_DESCR_EVT: {
        ESP_LOGD(TAG, "CCCD write status=%d handle=0x%04X", param->write.status, param->write.handle);
        break;
      }

      case ESP_GATTC_WRITE_CHAR_EVT: {
        if (param->write.status == ESP_GATT_OK) {
          ESP_LOGI(TAG, "Write OK (handle 0x%04X)", param->write.handle);
        } else {
          ESP_LOGW(TAG, "Write FAILED (handle 0x%04X) status=%d", param->write.handle, (int) param->write.status);
        }
        break;
      }

      case ESP_GATTC_NOTIFY_EVT: {
        // Received notification; expose raw hex for now.
        const uint8_t *data = param->notify.value;
        const uint16_t len = param->notify.value_len;

        if (last_notify_text_ != nullptr) {
          char *hex = (char *) malloc(len * 2 + 1);
          for (uint16_t i = 0; i < len; i++) {
            sprintf(hex + (i * 2), "%02x", data[i]);
          }
          hex[len * 2] = 0;
          last_notify_text_->publish_state(std::string(hex));
          free(hex);
        }

        this->notify_counter_++;
        if (notify_count_ != nullptr) notify_count_->publish_state((float) this->notify_counter_);

        // If these notifies carry the session key, flip the flag once we see the A5E5 header.
        if (len >= 2 && data[0] == 0xA5 && data[1] == 0xE5) {
          if (!this->got_key_) {
            this->got_key_ = true;
            if (key_received_ != nullptr) key_received_->publish_state(true);
            if (session_key_text_ != nullptr) session_key_text_->publish_state("received");
            ESP_LOGI(TAG, "Session key notification received (length=%u)", (unsigned) len);
          }
        }
        break;
      }

      case ESP_GATTC_CLOSE_EVT:
      case ESP_GATTC_DISCONNECT_EVT: {
        this->connected_flag_ = false;
        if (connected_ != nullptr) connected_->publish_state(false);
        ESP_LOGI(TAG, "Disconnected");
        this->got_key_ = false;
        this->last_hello_ms_ = 0;
        break;
      }

      default:
        break;
    }
  }

 protected:
  void early_subscribe_(esp_gatt_if_t gattc_if, uint16_t conn_id) {
    // Register for notifications
    esp_ble_gattc_register_for_notify(gattc_if, parent()->get_remote_bda(), fb_notify_);

    // Enable CCCD (notifications)
    uint8_t en[2] = {0x01, 0x00};
    esp_ble_gattc_write_char_descr(gattc_if, conn_id, fb_cccd_, sizeof(en), en,
                                   ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    ESP_LOGI(TAG, "CCCD enable sent (notify=0x%04X, cccd=0x%04X)", fb_notify_, fb_cccd_);
  }

  void send_hello_(const char *tag) {
    if (!this->connected_flag_) return;

    // HELLO observed in btsnoop:
    // A5 E5 00 0C 03 04 00 00 00 01 14 24 80 03 D4 13
    std::vector<uint8_t> buf = {0xA5, 0xE5, 0x00, 0x0C, 0x03, 0x04,
                                0x00, 0x00, 0x00, 0x01, 0x14, 0x24, 0x80, 0x03,
                                0xD4, 0x13};

    auto gattc_if = parent()->get_gattc_if();
    auto conn_id  = parent()->get_conn_id();

    esp_gatt_write_type_t wtype = prefer_write_no_rsp_ ? ESP_GATT_WRITE_TYPE_NO_RSP : ESP_GATT_WRITE_TYPE_RSP;
    esp_ble_gattc_write_char(gattc_if, conn_id, fb_write_, buf.size(), buf.data(), wtype, ESP_GATT_AUTH_REQ_NONE);

    if (session_key_text_ != nullptr) {
      // Expose what we sent for debugging (as hex)
      char *hex = (char *) malloc(buf.size() * 2 + 1);
      for (size_t i = 0; i < buf.size(); i++) sprintf(hex + (i*2), "%02x", buf[i]);
      hex[buf.size()*2] = 0;
      session_key_text_->publish_state(hex);
      free(hex);
    }

    ESP_LOGD(TAG, "[%s] HELLO to handle 0x%04X (%d bytes)", tag, fb_write_, (int) buf.size());
  }

  // ====== State ======
  text_sensor::TextSensor *session_key_text_{nullptr};
  text_sensor::TextSensor *last_notify_text_{nullptr};
  binary_sensor::BinarySensor *key_received_{nullptr};
  binary_sensor::BinarySensor *connected_{nullptr};
  sensor::Sensor *notify_count_{nullptr};

  bool connected_flag_{false};
  bool got_key_{false};
  uint32_t last_hello_ms_{0};
  uint32_t notify_counter_{0};

  // Config
  bool prefer_write_no_rsp_{false};
  std::string write_uuid_{"00002b11-0000-1000-8000-00805f9b34fb"};   // not used when fallbacks present
  std::string notify_uuid_{"00002b10-0000-1000-8000-00805f9b34fb"};  // not used when fallbacks present
  uint16_t fb_write_{0};
  uint16_t fb_notify_{0};
  uint16_t fb_cccd_{0};
};

}  // namespace blackview_lock
}  // namespace esphome

#endif  // USE_ESP32
