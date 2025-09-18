#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

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

  void set_session_key_text_sensor(text_sensor::TextSensor *t) { this->session_key_text_ = t; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *b) { this->connected_bin_ = b; }

  void setup() override {
    // Standard legacy bonding is sufficient
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
      case ESP_GATTC_CONNECT_EVT:
        this->state_ = LockState::IDLE;
        if (this->connected_bin_ != nullptr) this->connected_bin_->publish_state(true);
        ESP_LOGD(TAG, "Connected. Waiting for service discovery.");
        break;
      case ESP_GATTC_DISCONNECT_EVT:
        this->state_ = LockState::IDLE;
        if (this->connected_bin_ != nullptr) this->connected_bin_->publish_state(false);
        ESP_LOGD(TAG, "Disconnected, reason 0x%X", (unsigned) param->disconnect.reason);
        break;
      case ESP_GATTC_SEARCH_CMPL_EVT:
        this->resolve_handles_();
        esp_ble_gattc_register_for_notify(gattc_if, this->parent()->get_remote_bda(), this->notify_handle_);
        break;
      case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGD(TAG, "Subscribed to notifications. Starting handshake...");
        this->send_get_random_code_();
        break;
      case ESP_GATTC_NOTIFY_EVT:
        this->handle_notification_(param->notify.value, param->notify.value_len);
        break;
      default:
        break;
    }
  }

 protected:
  void resolve_handles_() {
    this->write_handle_ = 0x0009;
    this->notify_handle_ = 0x000B;
    ESP_LOGI(TAG, "Resolved handles: write=0x%04X, notify=0x000B", this->write_handle_, this->notify_handle_);
  }

  void send_get_random_code_() {
    ESP_LOGI(TAG, "Handshake Step 1: Sending GetRandomCode (32771)...");
    this->state_ = LockState::AWAITING_RANDOM_CODE;
    this->send_command_(32771, 1, {}); // Using a dummy random code of 1
  }

  void send_get_session_key_(uint32_t random_code) {
    ESP_LOGI(TAG, "Handshake Step 3: Sending GetSessionKey (36960) with received random code...");
    this->state_ = LockState::AWAITING_SESSION_KEY;
    this->send_command_(36960, random_code, {});
  }

  void handle_notification_(const uint8_t *data, uint16_t len) {
    ESP_LOGD(TAG, "NOTIFY received: %s", format_hex_pretty(data, len).c_str());

    if (len < 12 || data[0] != 0xA5 || data[1] != 0xE5) {
      ESP_LOGW(TAG, "Received invalid packet.");
      return;
    }

    const uint8_t* payload = data + 2;
    size_t payload_len = len - 4;

    // TODO: Verify CRC

    if (this->state_ == LockState::AWAITING_RANDOM_CODE) {
      // Expecting response to GetRandomCode. Payload should contain the real random code.
      uint32_t random_code = 0;
      memcpy(&random_code, payload + 4, 4); // The random code is 4 bytes in, after len/dst/src
      ESP_LOGI(TAG, "Handshake Step 2: Received random code: 0x%08X", random_code);
      this->send_get_session_key_(random_code);
    } else if (this->state_ == LockState::AWAITING_SESSION_KEY) {
      this->state_ = LockState::READY;
      // Expecting response to GetSessionKey. The inner payload's data is the session key.
      const uint8_t* inner_payload = payload + 8;
      size_t inner_payload_len = payload_len - 8;
      std::string key_hex = format_hex_pretty(inner_payload, inner_payload_len);
      ESP_LOGI(TAG, "Handshake Step 4: SUCCESS! Received Session Key: %s", key_hex.c_str());
      if(this->session_key_text_ != nullptr) this->session_key_text_->publish_state(key_hex);
    }
  }

  void send_command_(uint16_t cmd_id, uint32_t random_code, const std::vector<uint8_t>& data) {
    auto *cli = this->parent();
    if (cli == nullptr || !cli->connected()) {
      ESP_LOGW(TAG, "Cannot send command: not connected.");
      return;
    }
    
    // Inner Payload: [Cmd ID (2)] + [Data Len (2)] + [Data]
    std::vector<uint8_t> inner_payload;
    inner_payload.push_back(cmd_id & 0xFF);
    inner_payload.push_back((cmd_id >> 8) & 0xFF);
    uint16_t data_len = data.size();
    inner_payload.push_back(data_len & 0xFF);
    inner_payload.push_back((data_len >> 8) & 0xFF);
    if(data_len > 0) inner_payload.insert(inner_payload.end(), data.begin(), data.end());

    // Main Payload: [Total Len (2)] + [Dst (1)] + [Src (1)] + [Random (4)] + [Inner Payload]
    std::vector<uint8_t> payload;
    uint16_t total_len = 8 + inner_payload.size();
    payload.push_back(total_len & 0xFF);
    payload.push_back((total_len >> 8) & 0xFF);
    payload.push_back(0x03); // Dst Addr
    payload.push_back(0x04); // Src Addr
    payload.push_back(random_code & 0xFF);
    payload.push_back((random_code >> 8) & 0xFF);
    payload.push_back((random_code >> 16) & 0xFF);
    payload.push_back((random_code >> 24) & 0xFF);
    payload.insert(payload.end(), inner_payload.begin(), inner_payload.end());

    uint16_t crc = crc16_xmodem(payload.data(), payload.size());

    // Final Packet: [Sync (2)] + [Payload] + [CRC (2)]
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
  
  text_sensor::TextSensor *session_key_text_{nullptr};
  binary_sensor::BinarySensor *connected_bin_{nullptr};
};

}  // namespace blackview_lock
}  // namespace esphome
