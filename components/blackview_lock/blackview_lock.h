#pragma once

#include "esphome.h"

#ifdef USE_ESP32

#include <esp_bt.hh>
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>

namespace esphome {
namespace blackview_lock {

static const char *const TAG = "blackview_lock";
static const uint16_t BLACKVIEW_WRITE_HANDLE = 14;
static esp_bd_addr_t blackview_addr = {0xFC, 0x61, 0x79, 0xCF, 0x0A, 0x98};

class BlackviewLock : public PollingComponent {
 public:
  esp_gatt_if_t gattc_if;
  uint16_t conn_id;
  bool connected = false;
  bool scanning = false;

  BlackviewLock() : PollingComponent(60000) {}

  void setup() override;
  void update() override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

 protected:
  void start_scan();
  void send_hello_packet(esp_gatt_if_t gattc_if, uint16_t conn_id);
};

// --- Global instance and callback functions ---
// This section has been re-ordered and corrected

static BlackviewLock *global_blackview_lock = nullptr;

static void global_gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
  if (global_blackview_lock != nullptr)
    global_blackview_lock->gattc_event_handler(event, gattc_if, param);
}

static void global_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (global_blackview_lock != nullptr)
    global_blackview_lock->gap_event_handler(event, param);
}

// --- Method Implementations ---
// These are now outside the class definition for clarity

void BlackviewLock::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Blackview Lock component...");
  global_blackview_lock = this;
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  esp_ble_gattc_register_callback(global_gattc_event_handler);
  esp_ble_gap_register_callback(global_gap_event_handler);
  esp_ble_gattc_app_register(0);
}

void BlackviewLock::update() {
  if (!this->connected && !this->scanning) {
    this->start_scan();
  }
}

void BlackviewLock::start_scan() {
  ESP_LOGI(TAG, "Starting scan for lock...");
  this->scanning = true;
  static esp_ble_scan_params_t scan_params = {
      .scan_type = BLE_SCAN_TYPE_ACTIVE,
      .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
      .scan_interval = 0x50,
      .scan_window = 0x30,
      .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};
  esp_ble_gap_set_scan_params(&scan_params);
  esp_ble_gap_start_scanning(10);
}

void BlackviewLock::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_SCAN_RESULT_EVT) {
    if (memcmp(param->scan_rst.bda, blackview_addr, sizeof(esp_bd_addr_t)) == 0) {
      ESP_LOGI(TAG, "Lock found! Stopping scan and connecting...");
      esp_ble_gap_stop_scanning();
      esp_ble_gattc_open(this->gattc_if, param->scan_rst.bda, BLE_ADDR_TYPE_PUBLIC, true);
    }
  } else if (event == ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT) {
    this->scanning = false;
    ESP_LOGD(TAG, "Scan stopped.");
  }
}

void BlackviewLock::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
  if (event == ESP_GATTC_REG_EVT) {
    this->gattc_if = gattc_if;
  }

  if (gattc_if != this->gattc_if)
    return;
  
  if (event == ESP_GATTC_OPEN_EVT) {
    if (param->open.status == ESP_GATT_OK) {
      this->conn_id = param->open.conn_id;
      this->connected = true;
      this->scanning = false;
      ESP_LOGI(TAG, "Connected! Immediately sending Hello packet...");
      send_hello_packet(this->gattc_if, this->conn_id);
    } else {
      ESP_LOGW(TAG, "Connection failed, status=%d", param->open.status);
    }
  }

  if (event == ESP_GATTC_DISCONNECT_EVT) {
    this->connected = false;
    ESP_LOGI(TAG, "Disconnected from lock.");
  }

  if (event == ESP_GATTC_WRITE_CHAR_EVT) {
    if (param->write.status == ESP_GATT_OK) {
      ESP_LOGI(TAG, "Hello packet sent successfully!");
    } else {
      ESP_LOGW(TAG, "Failed to write Hello packet, status=%d", param->write.status);
    }
  }
  
  if (event == ESP_GATTC_NOTIFY_EVT) {
    ESP_LOGI(TAG, "SUCCESS! Key data received (%d bytes): %s",
             param->notify.value_len, format_hex_pretty(param->notify.value, param->notify.value_len).c_str());
  }
}

void BlackviewLock::send_hello_packet(esp_gatt_if_t gattc_if, uint16_t conn_id) {
  uint64_t random_c = ((uint64_t) esp_random() << 32) | esp_random();
  std::vector<uint8_t> payload;
  for (int i = 0; i < 8; i++) {
    payload.push_back((random_c >> (8 * i)) & 0xFF);
  }
  payload.push_back(0x01); payload.push_back(0x90);
  payload.push_back(0x00); payload.push_back(0x00);

  uint16_t crc = 0;
  for (auto b : payload) {
    crc ^= (uint16_t)b << 8;
    for (int i = 0; i < 8; i++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }

  std::vector<uint8_t> packet;
  packet.push_back(0x55); packet.push_back(0xAA);
  packet.push_back(0xAA); packet.push_back(0x55);
  for (auto b : payload) packet.push_back(b);
  packet.push_back(crc & 0xFF);
  packet.push_back((crc >> 8) & 0xFF);

  ESP_LOGD(TAG, "Writing Hello packet to handle 0x%02X", BLACKVIEW_WRITE_HANDLE);
  esp_ble_gattc_write_char(gattc_if, conn_id, BLACKVIEW_WRITE_HANDLE, packet.size(), packet.data(), ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
}

} // namespace blackview_lock
} // namespace esphome

#endif
