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
#include <vector>

namespace esphome {
namespace blackview_lock {

static const char *const TAG = "blackview_lock";

// CRC-16/XMODEM implementation
uint16_t crc16_xmodem(const uint8_t *data, size_t length) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

class BlackviewLock : public Component, public ble_client::BLEClientNode {
 public:
  enum class LockState {
    IDLE,
    AWAITING_RANDOM_CODE,
    AWAITING_SESSION_KEY,
    READY
  };

  // ----- Setters for YAML configuration (Restored for compatibility) -----
  void set_session_key_text_sensor(text_sensor::TextSensor *t) { this->session_key_text_ = t; }
  void set_last_notify_text_sensor(text_sensor::TextSensor *t) { this->last_notify_text_ = t; }
  void set_key_received_binary_sensor(binary_sensor::BinarySensor *b) { this->key_received_ = b; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *b) { this->connected_bin_ = b; }
  void set_notify_count_sensor(sensor::Sensor *s) { this->notify_count_ = s; }
  // Deprecated setters, kept for compile compatibility
  void set_prefer_write_no_rsp(bool v) {}
  void set_write_uuid(const std::string &u) {}
  void set_notify_uuid(const std::string &u) {}

  // ----- Public actions for YAML buttons (Restored for compatibility) -----
  void request_handshake() {
    ESP_LOGD(TAG, "Manual handshake requested. Starting process...");
    if (this->parent() && this->parent()->connected()) { // Corrected
      this->send_get_random_code_();
    } else {
      ESP_LOGW(TAG, "Cannot start handshake: not connected.");
    }
  }
  void request_handshake_mode(int mode) {
    ESP_LOGD(TAG, "Manual handshake (mode %d) requested. Starting process...", mode);
    this->request_handshake();
  }
  
  void setup() override {
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
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

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_CONNECT_EVT: {
        this->state_ = LockState::IDLE;
        if (this->connected_bin_ != nullptr) this->connected_bin_->publish_state(true);
        ESP_LOGD(TAG, "Connected. Waiting for service discovery.");
        break;
      }
      case ESP_GATTC_DISCONNECT_EVT: {
        this->state_ = LockState::IDLE;
        if (this->connected_bin_ != nullptr) this->connected_bin_->publish_state(false);
        ESP_LOGD(TAG, "Disconnected, reason 0x%X", (unsigned) param->disconnect.reason);
        break;
      }
      case ESP_GATTC_SEARCH_CMPL_EVT: {
        this->resolve_handles_();
        esp_ble_gattc_register_for_notify(gattc_if, this->parent()->get_remote_bda(), this->notify_handle_);
        break;
      }
      case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGD(TAG, "Subscribed to notifications. Sending GetRandomCode command...");
        this->send_get_random_code_();
        break;
      }
      case ESP_GATTC_NOTIFY_EVT: {
        this->handle_notification_(param->notify.value, param->notify.value_len);
        break;
      }
      default:
        break;
    }
  }

 protected:
  void resolve_handles_() {
    this->write_handle_ = 0x0009;
    this->notify_handle_ = 0x000B;
    ESP_LOGI(TAG, "Resolved handles: write=0x%04X, notify=0x%04X", this->write_handle_, this->notify_handle_);
  }
  
  void send_get_random_code_() {
    ESP_LOGI(TAG, "Handshake Step 1: Sending GetRandomCode (32771)...");
    this->state_ = LockState::AWAITING_RANDOM_CODE;
    this->send_command_(32771, 1, {}); 
  }

  void handle_notification_(const uint8_t *data, uint16_t len) {
    ESP_LOGD(TAG, "NOTIFY received: %s", format_hex_pretty(data, len).c_str());
    if (this->last_notify_text_ != nullptr) this->last_notify_text_->publish_state(format_hex_pretty(data, len));

    if (len < 6) {
        ESP_LOGW(TAG, "Received runt packet of length %d", len);
        return;
    }
    
    const uint8_t* payload = data + 2;
    size_t payload_len = len - 4;
    
    if (this->state_ == LockState::AWAITING_RANDOM_CODE) {
        if (payload_len >= 8) {
            uint64_t random_code = 0;
            memcpy(&random_code, payload, 8);
            ESP_LOGI(TAG, "Handshake Step 2: Received random code: 0x%016llX", random_code);
            this->send_get_session_key_(random_code);
        } else {
            ESP_LOGW(TAG, "Expected random code, but payload too short.");
        }
    } else if (this->state_ == LockState::AWAITING_SESSION_KEY) {
        this->state_ = LockState::READY;
        std::string key_hex = format_hex_pretty(payload, payload_len);
        ESP_LOGI(TAG, "Handshake Step 4: SUCCESS! Received Session Key: %s", key_hex.c_str());
        if(this->session_key_text_ != nullptr) this->session_key_text_->publish_state(key_hex);
        if(this->key_received_ != nullptr) this->key_received_->publish_state(true);
    }
  }

  void send_get_session_key_(uint64_t random_code) {
    ESP_LOGI(TAG, "Handshake Step 3: Sending GetSessionKey (36960) with received random code...");
    this->state_ = LockState::AWAITING_SESSION_KEY;
    this->send_command_(36960, random_code, {});
  }

  void send_command_(uint16_t cmd_id, uint64_t random_code, const std::vector<uint8_t>& data) {
    auto *cli = this->parent();
    if (cli == nullptr || !cli->connected()) { // Corrected
      ESP_LOGW(TAG, "Cannot send command: not connected.");
      return;
    }

    std::vector<uint8_t> payload;
    payload.resize(12 + data.size());
    memcpy(payload.data(), &random_code, 8);
    payload[8] = cmd_id & 0xFF;
    payload[9] = (cmd_id >> 8) & 0xFF;
    uint16_t data_len = data.size();
    payload[10] = data_len & 0xFF;
    payload[11] = (data_len >> 8) & 0xFF;
    if (data_len > 0) {
      memcpy(payload.data() + 12, data.data(), data_len);
    }
    
    uint16_t crc = crc16_xmodem(payload.data(), payload.size());

    std::vector<uint8_t> packet;
    packet.push_back(0xA5);
    packet.push_back(0xE5);
    packet.insert(packet.end(), payload.begin(), payload.end());
    packet.push_back(crc & 0xFF);
    packet.push_back((crc >> 8) & 0xFF);

    ESP_LOGD(TAG, "Writing command packet: %s", format_hex_pretty(packet.data(), packet.size()).c_str());
    
    esp_err_t r = esp_ble_gattc_write_char(cli->get_gattc_if(), cli->get_conn_id(), this->write_handle_,
                                           packet.size(), packet.data(),
                                           ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NO_MITM);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "esp_ble_gattc_write_char error, status=%d", r);
    }
  }

  // ----- Members -----
  uint16_t write_handle_{0};
  uint16_t notify_handle_{0};
  LockState state_{LockState::IDLE};
  
  // Member variables for sensors
  text_sensor::TextSensor *session_key_text_{nullptr};
  text_sensor::TextSensor *last_notify_text_{nullptr};
  binary_sensor::BinarySensor *key_received_{nullptr};
  binary_sensor::BinarySensor *connected_bin_{nullptr};
  sensor::Sensor *notify_count_{nullptr};
};

}  // namespace blackview_lock
}  // namespace esphome
