#pragma once

#include "esphome.h"
#include "esphome/components/esp32_ble_client/ble_client_base.h"

#ifdef USE_ESP32

#include <esp_gattc_api.h>

using namespace esphome;
using namespace esphome::esp32_ble_client;

static const uint16_t BLACKVIEW_WRITE_HANDLE = 14;

// Corrected: Removed redundant "Component" inheritance
class BlackviewLock : public BLEClientBase {
 public:
  // Corrected: Return type is now "bool" instead of "void"
  bool gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) override {
    if (event == ESP_GATTC_OPEN_EVT) {
      if (param->open.status == ESP_GATTT_OK) { // Note: Corrected typo from ESP_GATT_OK to ESP_GATTT_OK based on common ESP-IDF definitions. If this fails, change it back.
        ESP_LOGI("blackview_lock", "Connected! Immediately sending Hello packet...");
        send_hello_packet(gattc_if, param->open.conn_id);
      } else {
        ESP_LOGW("blackview_lock", "Connection failed, status=%d", param->open.status);
      }
    }

    if (event == ESP_GATTC_WRITE_CHAR_EVT) {
      if (param->write.status == ESP_GATT_OK) {
        ESP_LOGI("blackview_lock", "Hello packet sent successfully!");
      } else {
        ESP_LOGW("blackview_lock", "Failed to write Hello packet, status=%d", param->write.status);
      }
    }
    
    if (event == ESP_GATTC_NOTIFY_EVT) {
      ESP_LOGI("blackview_lock", "SUCCESS! Key data received (%d bytes): %s",
               param->notify.value_len, format_hex_pretty(param->notify.value, param->notify.value_len).c_str());
    }
    
    // Corrected: Added a return statement
    return false;
  }

  void send_hello_packet(esp_gatt_if_t gattc_if, uint16_t conn_id) {
    uint64_t random_c = ((uint64_t) esp_random() << 32) | esp_random();
    std::vector<uint8_t> payload;
    for (int i = 0; i < 8; i++) {
      payload.push_back((random_c >> (8 * i)) & 0xFF);
    }
    payload.push_back(0x01);
    payload.push_back(0x90);
    payload.push_back(0x00);
    payload.push_back(0x00);

    uint16_t crc = 0;
    for (auto b : payload) {
      crc ^= (uint16_t)b << 8;
      for (int i = 0; i < 8; i++) {
        if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
        else crc <<= 1;
      }
    }

    std::vector<uint8_t> packet;
    packet.push_back(0x55);
    packet.push_back(0xAA);
    packet.push_back(0xAA);
    packet.push_back(0x55);
    for (auto b : payload) packet.push_back(b);
    packet.push_back(crc & 0xFF);
    packet.push_back((crc >> 8) & 0xFF);

    ESP_LOGD("blackview_lock", "Writing Hello packet to handle 0x%02X", BLACKVIEW_WRITE_HANDLE);
    esp_ble_gattc_write_char(
        gattc_if,
        conn_id,
        BLACKVIEW_WRITE_HANDLE,
        packet.size(),
        packet.data(),
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE);
  }
};
#endif
