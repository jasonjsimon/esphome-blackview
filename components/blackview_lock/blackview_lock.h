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

// Blackview service/handles observed on SE60 variants:
//   Service 0x1910
//   Write characteristic  0x2B11 -> handle 0x0009
//   Notify characteristic 0x2B10 -> handle 0x000B
//   CCCD for notify                         0x000C
class BlackviewLock : public Component, public ble_client::BLEClientNode {
 public:
  // ---- Entity wiring (from YAML) ----
  void set_session_key_text_sensor(text_sensor::TextSensor *t) { session_key_text_ = t; }
  void set_last_notify_text_sensor(text_sensor::TextSensor *t) { last_notify_text_ = t; }
  void set_key_received_binary_sensor(binary_sensor::BinarySensor *b) { key_received_ = b; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *b) { connected_ = b; }
  void set_notify_count_sensor(sensor::Sensor *s) { notify_count_ = s; }

  // ---- Back-compat no-ops so existing YAML/main.cpp compiles unchanged ----
  void set_prefer_write_no_rsp(bool) {}
  void set_write_uuid(const char *) {}
  void set_notify_uuid(const char *) {}

  // ---- User actions ----
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

  // ---- GATTC event bridge from ESPHome BLE client ----
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_CONNECT_EVT: {
        connected_flag_ = true;
        std::memcpy(remote_bda_, param->connect.remote_bda, sizeof(remote_bda_));
        have_bda_ = true;
        hello_sent_after_cccd_ = false;
        post_cccd_hello_due_ms_ = 0;
        encryption_requested_ms_ = 0;
        if (connected_) connected_->publish_state(true);
        ESP_LOGI(TAG, "Connected.");

        // Proactively request link encryption; many locks reject writes until encrypted.
        if (have_bda_) {
          esp_err_t e = esp_ble_set_encryption(remote_bda_, ESP_BLE_SEC_ENCRYPT_MITM);
          if (e == ESP_OK) {
            encryption_requested_ms_ = millis();
            ESP_LOGD(TAG, "Requested encryption (MITM). Waiting ~%ums before first HELLO.",
                     encryption_settle_ms_);
          } else {
            ESP_LOGW(TAG, "Failed to request encryption (err=%d). Proceeding without explicit auth.", (int) e);
          }
        }
        break;
      }

      case ESP_GATTC_SEARCH_CMPL_EVT: {
        ESP_LOGI(TAG, "Service discovery complete");
        resolve_handles_();
        // Prefer indications; fall back to notifications if CCCD write fails.
        enable_notify_(gattc_if, param->search_cmpl.conn_id, /*use_indications=*/true);
        break;
      }

      case ESP_GATTC_WRITE_DESCR_EVT: {
        // CCCD write result
        if (param->write.status == ESP_GATT_OK) {
          ESP_LOGD(TAG, "Descriptor write OK (handle 0x%04X)", param->write.handle);
          // Defer HELLO: small gap after CCCD write; also give encryption time to complete.
          pending_conn_id_ = param->write.conn_id;
          pending_gattc_if_ = gattc_if;
          post_cccd_hello_due_ms_ = millis() + post_cccd_delay_ms_;
        } else {
          ESP_LOGW(TAG, "Descriptor write failed (handle 0x%04X, status %d)",
                   param->write.handle, param->write.status);
          // If indications failed, retry with notifications.
          if (last_cccd_used_indications_) {
            last_cccd_used_indications_ = false;
            enable_notify_(gattc_if, param->write.conn_id, /*use_indications=*/false);
          } else {
            // If notifications also failed, try HELLO anyway (some firmwares still accept writes).
            pending_conn_id_ = param->write.conn_id;
            pending_gattc_if_ = gattc_if
