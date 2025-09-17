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
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_OPEN_EVT: {
        if (param->open.status == ESP_GATT_OK) {
          connect_open_ms_ = millis();
          auto_tries_ = 0;
          if (connected_) connected_->publish_state(true);
          ESP_LOGI(TAG, "Connected! Sending HELLO immediately...");
          send_hello_on_(gattc_if, param->open.conn_id, (write_handle_ ? write_handle_ : FALLBACK_WRITE_HANDLE),
                         HELLO_STD_LE, "fallback");
          last_hello_ms_ = millis();
        } else {
          ESP_LOGW(TAG, "OPEN_EVT status=%d", param->open.status);
        }
        break;
      }

      case ESP_GATTC_CLOSE_EVT:
      case ESP_GATTC_DISCONNECT_EVT: {
        if (connected_) connected_->publish_state(false);
        got_key_ = false;
        auto_tries_ = 0;
        if (key_received_) key_received_->publish_state(false);
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

      case ESP_GATTC_SEARCH_CMP_EVT:  // some IDF versions use *_CMP_ vs *_CMPL_ – handle both
      case ESP_GATTC_SEARCH_CMPL_EVT: {
        if ((event == ESP_GATTC_SEARCH_CMPL_EVT && param->search_cmpl.status == ESP_GATT_OK) ||
            (event == ESP_GATTC_SEARCH_CMP_EVT)) {
          ESP_LOGI(TAG, "Service discovery complete; resolving handles...");
          this->resolve_handles_and_subscribe_();
          // After CCCD, do one re-poke (don’t spam)
          if (write_handle_ != 0 && !got_key_) {
            delay_hello_after_cccd_ = true;
          }
        } else if (event == ESP_GATTC_SEARCH_CMPL_EVT) {
          ESP_LOGW(TAG, "SEARCH_CMPL status=%d", param->search_cmpl.status);
        }
        break;
      }

      case ESP_GATTC_WRITE_DESCR_EVT: {
        // After CCCD write completes, send one HELLO (if flagged)
        if (delay_hello_after_cccd_ && write_handle_ != 0 && !got_key_) {
          delay_hello_after_cccd_ = false;
          send_hello_on_(parent()->get_gattc_if(), parent()->get_conn_id(), write_handle_, HELLO_STD_LE, "resolved");
          last_hello_ms_ = millis();
        }
        break;
      }

      case ESP_GATTC_NOTIFY_EVT: {
        // Raw notify → hex
        std::string raw = format_hex_pretty(param->notify.value, param->notify.value_len);
        ESP_LOGI(TAG, "NOTIFY (%d bytes) on 0x%04X: %s",
                 param->notify.value_len, param->notify.handle, raw.c_str());
        notify_counter_++;
        if (notify_count_) notify_count_->publish_state((float) notify_counter_);
        if (last_notify_text_) last_notify_text_->publish_state(raw);

        // Tentative key parse: after 0x55AA AA55, take 16 bytes
        if (param->notify.value_len >= 24 &&
            param->notify.value[0] == 0x55 && param->notify.value[1] == 0xAA &&
            param->notify.value[2] == 0xAA && param->notify.value[3] == 0x55) {
          std::string key_hex = format_hex_pretty(&param->notify.value[4], 16);
          if (session_key_text_) session_key_text_->publish_state(key_hex);
          got_key_ = true;
          if (key_received_) key_received_->publish_state(true);
          ESP_LOGI(TAG, "Parsed session key (tentative): %s", key_hex.c_str());
        }
        break;
      }

      default:
        break;
    }
  }

 private:
  // HA sensors
  TextSensor   *session_key_text_ = nullptr;
  TextSensor   *last_notify_text_ = nullptr;
  BinarySensor *key_received_     = nullptr;
  BinarySensor *connected_        = nullptr;
  Sensor       *notify_count_     = nullptr;

  // State
  uint16_t write_handle_  = 0;
  uint16_t notify_handle_ = 0;
  uint16_t cccd_handle_   = 0;
  bool     got_key_       = false;
  bool     delay_hello_after_cccd_ = false;
  uint32_t last_hello_ms_ = 0;
  uint32_t connect_open_ms_ = 0;
  uint32_t notify_counter_ = 0;
  uint8_t  auto_tries_ = 0;
  static constexpr uint32_t kHelloIntervalMs = 8000;
  static constexpr uint8_t  kMaxAutoTries    = 6;
  static constexpr uint32_t kPairingWatchdogMs = 90000;

  // ---- Builders ----
  static void append_crc_xmodem_le_(std::vector<uint8_t> &buf, const std::vector<uint8_t> &payload) {
    uint16_t crc = 0;
    for (auto b : payload) {
      crc ^= (uint16_t) b << 8;
      for (int i = 0; i < 8; i++)
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    buf.push_back(crc & 0xFF);          // LE: low, then high
    buf.push_back((crc >> 8) & 0xFF);
  }
  static void append_crc_xmodem_be_(std::vector<uint8_t> &buf, const std::vector<uint8_t> &payload) {
    uint16_t crc = 0;
    for (auto b : payload) {
      crc ^= (uint16_t) b << 8;
      for (int i = 0; i < 8; i++)
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    buf.push_back((crc >> 8) & 0xFF);   // BE: high, then low
    buf.push_back(crc & 0xFF);
  }

  static void build_payload_(std::vector<uint8_t> &payload) {
    // random8 + 0x01 0x90 + 0x00 0x00  (adjust later if we learn the exact format differs)
    uint64_t rnd = ((uint64_t) esp_random() << 32) | esp_random();
    for (int i = 0; i < 8; i++) payload.push_back((rnd >> (8 * i)) & 0xFF);
    payload.push_back(0x01); payload.push_back(0x90);
    payload.push_back(0x00); payload.push_back(0x00);
  }

  static void build_hello_packet_(HelloVariant v, std::vector<uint8_t> &out) {
    out.clear();
    std::vector<uint8_t> payload;
    build_payload_(payload);

    switch (v) {
      case HELLO_STD_LE: {
        out = {0x55,0xAA,0xAA,0x55};
        out.insert(out.end(), payload.begin(), payload.end());
        append_crc_xmodem_le_(out, payload);
        break;
      }
      case HELLO_STD_NO_RSP: {  // same bytes as STD_LE; only write type differs
        out = {0x55,0xAA,0xAA,0x55};
        out.insert(out.end(), payload.begin(), payload.end());
        append_crc_xmodem_le_(out, payload);
        break;
      }
      case HELLO_STD_BE: {
        out = {0x55,0xAA,0xAA,0x55};
        out.insert(out.end(), payload.begin(), payload.end());
        append_crc_xmodem_be_(out, payload);
        break;
      }
      case HELLO_NO_HEADER_LE: {
        out = payload;
        append_crc_xmodem_le_(out, payload);
        break;
      }
    }
  }

  // ---- Send helpers ----
  void send_hello_variant_(HelloVariant v, const char *tag) {
    if (parent() == nullptr || !parent()->connected()) {
      ESP_LOGW(TAG, "Handshake requested, but BLE not connected");
      return;
    }
    const uint16_t h = (write_handle_ != 0) ? write_handle_ : FALLBACK_WRITE_HANDLE;
    if (h == 0) {
      ESP_LOGW(TAG, "No write handle yet; cannot send HELLO");
      return;
    }
    send_hello_on_(parent()->get_gattc_if(), parent()->get_conn_id(), h, v, tag);
  }

  void send_hello_on_(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle,
                      HelloVariant v, const char *tag) {
    if (handle == 0) return;

    std::vector<uint8_t> packet;
    build_hello_packet_(v, packet);

    const esp_gatt_write_type_t wt = (v == HELLO_STD_NO_RSP) ? ESP_GATT_WRITE_TYPE_NO_RSP : ESP_GATT_WRITE_TYPE_RSP;
    ESP_LOGD(TAG, "[%s] HELLO v%d to handle 0x%04X (%d bytes)", tag, (int) v, handle, (int) packet.size());
    esp_err_t err = esp_ble_gattc_write_char(gattc_if, conn_id, handle,
                                             (uint16_t) packet.size(), packet.data(),
                                             wt, ESP_GATT_AUTH_REQ_NONE);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "[%s] esp_ble_gattc_write_char error=%d", tag, err);
    }
  }

  // ---- Discovery/subscribe ----
  void resolve_handles_and_subscribe_() {
    // Enumerate all chars to find our handles (post-discovery).
    const uint16_t start = 1, end = 0xFFFF;
    esp_gattc_char_elem_t chars[32];
    uint16_t count = 32, offset = 0;

    write_handle_ = notify_handle_ = cccd_handle_ = 0;

    while (true) {
      count = 32;
      esp_err_t st = esp_ble_gattc_get_all_char(
          parent()->get_gattc_if(), parent()->get_conn_id(),
          start, end, chars, &count, offset);
      if (st != ESP_OK || count == 0) break;

      for (int i = 0; i < count; i++) {
        const auto &e = chars[i];
        if (e.uuid.len == ESP_UUID_LEN_16) {
          if (e.uuid.uuid.uuid16 == UUID_WRITE_CHAR)  write_handle_  = e.char_handle;
          if (e.uuid.uuid.uuid16 == UUID_NOTIFY_CHAR) notify_handle_ = e.char_handle;
        }
      }
      offset += count;
    }

    ESP_LOGI(TAG, "Resolved: write_handle=0x%04X, notify_handle=0x%04X", write_handle_, notify_handle_);

    if (notify_handle_ == 0) {
      ESP_LOGW(TAG, "Notify characteristic not found; notifications will not arrive.");
      return;
    }

    // Find CCCD (0x2902) under notify char
    esp_gattc_descr_elem_t descrs[8];
    uint16_t dcount = 8;
    if (esp_ble_gattc_get_all_descr(parent()->get_gattc_if(), parent()->get_conn_id(),
                                    notify_handle_, descrs, &dcount, 0) == ESP_OK) {
      for (int i = 0; i < dcount; i++) {
        if (descrs[i].uuid.len == ESP_UUID_LEN_16 &&
            descrs[i].uuid.uuid.uuid16 == UUID_CCCD_DESCR) {
          cccd_handle_ = descrs[i].handle;
          break;
        }
      }
    }

    if (cccd_handle_ == 0) {
      ESP_LOGW(TAG, "CCCD not found for notify handle 0x%04X", notify_handle_);
      return;
    }

    // Register for notify + enable CCCD (0x0001)
    if (esp_ble_gattc_register_for_notify(parent()->get_gattc_if(), parent()->get_remote_bda(), notify_handle_) != ESP_OK) {
      ESP_LOGW(TAG, "register_for_notify failed");
      return;
    }
    uint8_t enable_notify[2] = {0x01, 0x00};
    if (esp_ble_gattc_write_char_descr(parent()->get_gattc_if(), parent()->get_conn_id(),
                                       cccd_handle_, sizeof(enable_notify), enable_notify,
                                       ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE) != ESP_OK) {
      ESP_LOGW(TAG, "write CCCD failed");
    } else {
      ESP_LOGI(TAG, "CCCD enabled (0x%04X); waiting for key...", cccd_handle_);
    }
  }
};

}  // namespace blackview_lock
}  // namespace esphome
