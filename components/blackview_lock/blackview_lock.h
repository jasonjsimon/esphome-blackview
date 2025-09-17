#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"

#ifdef USE_ESP32

// Some ESP-IDF variants use *_CMP_* vs *_CMPL_* for this event.
#ifndef ESP_GATTC_SEARCH_CMP_EVT
#define ESP_GATTC_SEARCH_CMP_EVT ESP_GATTC_SEARCH_CMPL_EVT
#endif

namespace esphome {
namespace blackview_lock {

static const char *const TAG = "blackview_lock";

// This component talks to the Blackview/Arale lock over BLE.
// It uses write char 0x2B11 and notify char 0x2B10 found in the btsnoop,
// and enables CCCD on the notify characteristic.
class BlackviewLock : public Component, public ble_client::BLEClientNode {
 public:
  // ----- Entity wiring from the Python schema -----
  void set_session_key_text_sensor(text_sensor::TextSensor *t) { session_key_text_ = t; }
  void set_last_notify_text_sensor(text_sensor::TextSensor *t) { last_notify_text_ = t; }
  void set_key_received_binary_sensor(binary_sensor::BinarySensor *b) { key_received_ = b; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *b) { connected_ = b; }
  void set_notify_count_sensor(sensor::Sensor *s) { notify_count_ = s; }

  // Optional: button action from YAML to force a HELLO write.
  void manual_hello() {
    if (!connected_on_) {
      ESP_LOGW(TAG, "Not connected; can't send manual HELLO.");
      return;
    }
    this->send_hello_on_(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), write_handle_, /*no_rsp=*/false, "manual");
  }

  float get_setup_priority() const override { return setup_priority::DATA; }

  // Keep sending HELLO every few seconds until the lock answers.
  void loop() override {
    const uint32_t now = millis();
    const bool need_hello = connected_on_ && !got_key_ && (now - last_hello_ms_ >= write_interval_ms_);
    if (need_hello) {
      this->send_hello_on_(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), write_handle_, /*no_rsp=*/false, "auto");
    }
  }

  // Core GATT client event hook.
  void gattc_event_handler(esp_gattc_cb_event_t event,
                           esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_OPEN_EVT: {
        if (param->open.status == ESP_GATT_OK) {
          connected_on_ = true;
          if (connected_) connected_->publish_state(true);
          ESP_LOGI(TAG, "Connection open");
          // Use known-good fallback handles immediately, then let discovery
          // confirm (or just continue using these).
          this->early_subscribe_and_hello_(gattc_if, param->open.conn_id);
        } else {
          ESP_LOGW(TAG, "OPEN_EVT status=%d", param->open.status);
        }
        break;
      }

      case ESP_GATTC_SEARCH_CMPL_EVT:
      case ESP_GATTC_SEARCH_CMP_EVT: {
        if (param->search_cmpl.status == ESP_GATT_OK) {
          ESP_LOGI(TAG, "Service discovery complete");
          // We learned from the btsnoop that:
          //  - Primary Service UUID 0x1910 contains:
          //      * 0x2B11 (write)   → typically handle 0x0009
          //      * 0x2B10 (notify)  → typically handle 0x000B, CCCD 0x000C
          // Handles tend to be stable on these locks; keep our defaults,
          // but log what we’re using.
          ESP_LOGI(TAG, "Using handles: write=0x%04X, notify=0x%04X, cccd=0x%04X",
                   write_handle_, notify_handle_, cccd_handle_);
        } else {
          ESP_LOGW(TAG, "SEARCH_CMPL status=%d", param->search_cmpl.status);
        }
        break;
      }

      case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGD(TAG, "Registered for notifications (status=%d)", param->reg_for_notify.status);
        break;
      }

      case ESP_GATTC_WRITE_DESCR_EVT: {
        // CCCD write result
        if (param->write.status == ESP_GATT_OK) {
          ESP_LOGI(TAG, "CCCD write OK (handle 0x%04X)", param->write.handle);
        } else {
          ESP_LOGW(TAG, "CCCD write FAILED (handle 0x%04X) status=%d", param->write.handle, param->write.status);
        }
        break;
      }

      case ESP_GATTC_WRITE_CHAR_EVT: {
        if (param->write.status == ESP_GATT_OK) {
          if (param->write.handle == write_handle_) {
            ESP_LOGI(TAG, "Write OK (handle 0x%04X)", param->write.handle);
          } else {
            ESP_LOGD(TAG, "Write OK (handle 0x%04X)", param->write.handle);
          }
        } else {
          ESP_LOGW(TAG, "Write FAILED (handle 0x%04X) status=%d", param->write.handle, param->write.status);
        }
        break;
      }

      case ESP_GATTC_NOTIFY_EVT: {
        // Notification from 0x2B10 (should be notify_handle_)
        const uint8_t *data = param->notify.value;
        const uint16_t len = param->notify.value_len;
        notify_counter_++;
        if (notify_count_) notify_count_->publish_state(static_cast<float>(notify_counter_));

        // Make a pretty hex string for diagnostics
        char buf[512];
        size_t pos = 0;
        for (uint16_t i = 0; i < len && pos + 3 < sizeof(buf); i++) {
          pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", data[i]);
        }
        if (last_notify_text_) last_notify_text_->publish_state(std::string(buf, pos));

        ESP_LOGD(TAG, "Notify (handle 0x%04X, %u bytes)", param->notify.handle, (unsigned) len);

        // Heuristic: first non-empty notify means lock is alive & answered.
        // Some firmwares send a session key or status frame first; we don’t
        // parse the protocol here yet, but we record that a key/answer arrived.
        if (!got_key_) {
          got_key_ = true;
          if (key_received_) key_received_->publish_state(true);
          // If the frame looks like a 16-byte session key, save it as hex.
          if (len == 16 && session_key_text_) {
            char key_hex[64];
            size_t kpos = 0;
            for (uint16_t i = 0; i < len && kpos + 3 < sizeof(key_hex); i++)
              kpos += snprintf(key_hex + kpos, sizeof(key_hex) - kpos, "%02X", data[i]);
            session_key_text_->publish_state(std::string(key_hex, kpos));
          }
        }
        break;
      }

      case ESP_GATTC_DISCONNECT_EVT: {
        connected_on_ = false;
        got_key_ = false;
        notify_counter_ = 0;
        last_hello_ms_ = 0;
        if (connected_) connected_->publish_state(false);
        ESP_LOGW(TAG, "Disconnected (reason=%d)", param->disconnect.reason);
        break;
      }

      default:
        break;
    }
  }

 private:
  // Known-good defaults (from your btsnoop): 0x2B11 write, 0x2B10 notify, CCCD next handle
  uint16_t write_handle_  = 0x0009;
  uint16_t notify_handle_ = 0x000B;
  uint16_t cccd_handle_   = 0x000C;

  // State & timers
  bool connected_on_ = false;
  bool got_key_ = false;
  uint32_t last_hello_ms_ = 0;
  const uint32_t write_interval_ms_ = 8000;

  // Entities
  text_sensor::TextSensor *session_key_text_ = nullptr;
  text_sensor::TextSensor *last_notify_text_ = nullptr;
  binary_sensor::BinarySensor *key_received_ = nullptr;
  binary_sensor::BinarySensor *connected_ = nullptr;
  sensor::Sensor *notify_count_ = nullptr;
  uint32_t notify_counter_ = 0;

  // CRC-XMODEM (poly 0x1021, init 0x0000), LE append
  static void append_crc_xmodem_le_(std::vector<uint8_t> &buf) {
    uint16_t crc = 0x0000;
    for (auto b : buf) {
      crc ^= (uint16_t)b << 8;
      for (int i = 0; i < 8; i++) {
        if (crc & 0x8000)
          crc = (crc << 1) ^ 0x1021;
        else
          crc = (crc << 1);
      }
    }
    buf.push_back((uint8_t)(crc & 0xFF));
    buf.push_back((uint8_t)((crc >> 8) & 0xFF));
  }

  // Build the 18-byte HELLO (4-byte header + 12 payload + 2 CRC)
  void build_hello_(std::vector<uint8_t> &out) {
    out.clear();
    // Header (observed on wire for this family of locks)
    out.push_back(0x55);
    out.push_back(0xAA);
    out.push_back(0xAA);
    out.push_back(0x55);

    // 8 bytes of nonce/random
    for (int i = 0; i < 8; i++) out.push_back((uint8_t)random_uint32() & 0xFF);

    // Command/tag for "hello"
    out.push_back(0x01);
    out.push_back(0x90);

    // 2 bytes reserved
    out.push_back(0x00);
    out.push_back(0x00);

    // CRC over entire buffer so far
    append_crc_xmodem_le_(out);
  }

  // Register for notify on fallback handle and enable CCCD, then write HELLO
  void early_subscribe_and_hello_(esp_gatt_if_t gattc_if, uint16_t conn_id) {
    // Register for notifications first
    esp_ble_gattc_register_for_notify(gattc_if, this->parent()->get_remote_bda(), notify_handle_);

    // Enable CCCD (notifications = 0x0001)
    uint8_t en[2] = {0x01, 0x00};
    esp_ble_gattc_write_char_descr(gattc_if, conn_id, cccd_handle_, sizeof(en), en,
                                   ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    ESP_LOGI(TAG, "CCCD enable sent (notify=0x%04X, cccd=0x%04X)", notify_handle_, cccd_handle_);

    // Send initial HELLO
    this->send_hello_on_(gattc_if, conn_id, write_handle_, /*no_rsp=*/false, "init");
  }

  // Transmit HELLO to the write handle
  void send_hello_on_(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle, bool no_rsp, const char *reason) {
    std::vector<uint8_t> pkt;
    build_hello_(pkt);

    ESP_LOGD(TAG, "[%s] HELLO to handle 0x%04X (%u bytes)", reason, handle, (unsigned) pkt.size());
    esp_gatt_write_type_t wtype = no_rsp ? ESP_GATT_WRITE_TYPE_NO_RSP : ESP_GATT_WRITE_TYPE_RSP;
    esp_ble_gattc_write_char(gattc_if, conn_id, handle, pkt.size(), pkt.data(), wtype, ESP_GATT_AUTH_REQ_NONE);

    last_hello_ms_ = millis();
  }
};

}  // namespace blackview_lock
}  // namespace esphome

#endif  // USE_ESP32
