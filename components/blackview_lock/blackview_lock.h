#pragma once
#include "esphome.h"
#include "esphome/components/ble_client/ble_client.h"

#ifdef USE_ESP32
#include <esp_gattc_api.h>
#include <esp_gatt_defs.h>

namespace esphome {
namespace blackview_lock {

using namespace esphome::ble_client;
static const char *const TAG = "blackview_lock";

// --- Known UUIDs from your reverse-engineering ---
static constexpr uint16_t UUID_NOTIFY_CHAR  = 0x2B10; // 00002b10-0000-1000-8000-00805f9b34fb
static constexpr uint16_t UUID_WRITE_CHAR   = 0x2B11; // 00002b11-0000-1000-8000-00805f9b34fb
static constexpr uint16_t UUID_CCCD_DESCR   = 0x2902; // Client Characteristic Configuration

// Optional: keep the early HELLO path using a fallback handle to win the race.
// If unknown, keep at 0 (disabled). Your previous value was 0x000E (14).
static constexpr uint16_t FALLBACK_WRITE_HANDLE = 0x000E;

class BlackviewLock : public Component, public BLEClientNode {
 public:
  void setup() override {
    ESP_LOGI(TAG, "Setup complete");
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "BlackviewLock:");
    ESP_LOGCONFIG(TAG, "  write_handle_ : 0x%04X", write_handle_);
    ESP_LOGCONFIG(TAG, "  notify_handle_: 0x%04X", notify_handle_);
    ESP_LOGCONFIG(TAG, "  cccd_handle_  : 0x%04X", cccd_handle_);
  }

  void gattc_event_handler(esp_gattc_cb_event_t event,
                           esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_OPEN_EVT: {
        if (param->open.status == ESP_GATT_OK) {
          ESP_LOGI(TAG, "Connected! Sending HELLO immediately...");
          // Try early HELLO using fallback handle (wins race vs. discovery)
          if (FALLBACK_WRITE_HANDLE != 0) {
            try_send_hello(gattc_if, param->open.conn_id, FALLBACK_WRITE_HANDLE, /*tag=*/"fallback");
          } else {
            ESP_LOGW(TAG, "FALLBACK_WRITE_HANDLE is 0; will wait for discovery to resolve handles.");
          }
        } else {
          ESP_LOGW(TAG, "OPEN_EVT status=%d", param->open.status);
        }
        break;
      }

      case ESP_GATTC_WRITE_CHAR_EVT: {
        if (param->write.status == ESP_GATT_OK) {
          ESP_LOGI(TAG, "Write OK (handle 0x%04X, len=%d)", param->write.handle, param->write.value_len);
        } else {
          ESP_LOGW(TAG, "Write FAILED (handle 0x%04X) status=%d", param->write.handle, param->write.status);
        }
        break;
      }

      case ESP_GATTC_SEARCH_CMPL_EVT: {
        if (param->search_cmpl.status == ESP_GATT_OK) {
          ESP_LOGI(TAG, "Service discovery complete; resolving handles...");
          resolve_handles_and_subscribe_();
          // If we didn’t get a key yet and we now have the real write handle, resend HELLO.
          if (write_handle_ != 0 && !got_key_) {
            try_send_hello(parent()->get_gattc_if(), parent()->get_conn_id(), write_handle_, /*tag=*/"resolved");
          }
        } else {
          ESP_LOGW(TAG, "SEARCH_CMPL status=%d", param->search_cmpl.status);
        }
        break;
      }

      case ESP_GATTC_NOTIFY_EVT: {
        // Got server notification – this should be the dynamic key after HELLO
        ESP_LOGI(TAG, "NOTIFY (%d bytes) on 0x%04X: %s",
                 param->notify.value_len, param->notify.handle,
                 format_hex_pretty(param->notify.value, param->notify.value_len).c_str());
        got_key_ = true;
        break;
      }

      default:
        // Optional: uncomment to trace other events
        // ESP_LOGD(TAG, "Event %d", event);
        break;
    }
  }

 private:
  // --- State ---
  uint16_t write_handle_  = 0;
  uint16_t notify_handle_ = 0;
  uint16_t cccd_handle_   = 0;
  bool     got_key_       = false;

  // Compose and send the HELLO packet (your same algorithm)
  void try_send_hello(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle, const char *tag) {
    if (handle == 0) return;

    // Build payload: random 8B + 0x01 0x90 + 0x00 0x00 + CRC16/XMODEM over payload
    uint64_t random_c = ((uint64_t) esp_random() << 32) | esp_random();
    std::vector<uint8_t> payload;
    for (int i = 0; i < 8; i++) payload.push_back((random_c >> (8 * i)) & 0xFF);
    payload.push_back(0x01); payload.push_back(0x90);
    payload.push_back(0x00); payload.push_back(0x00);

    uint16_t crc = 0;
    for (auto b : payload) {
      crc ^= (uint16_t) b << 8;
      for (int i = 0; i < 8; i++) {
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
      }
    }

    std::vector<uint8_t> packet;
    packet.reserve(4 + payload.size() + 2);
    packet.push_back(0x55); packet.push_back(0xAA);
    packet.push_back(0xAA); packet.push_back(0x55);
    packet.insert(packet.end(), payload.begin(), payload.end());
    packet.push_back(crc & 0xFF);
    packet.push_back((crc >> 8) & 0xFF);

    ESP_LOGD(TAG, "[%s] Writing HELLO to handle 0x%04X (%d bytes)", tag, handle, (int) packet.size());
    auto err = esp_ble_gattc_write_char(
      gattc_if, conn_id, handle,
      (uint16_t) packet.size(), packet.data(),
      ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE
    );

    if (err != ESP_OK) {
      ESP_LOGW(TAG, "[%s] esp_ble_gattc_write_char returned error=%d", tag, err);
    }
  }

  // After discovery, enumerate ALL characteristics and find ours by UUID.
  void resolve_handles_and_subscribe_() {
    // 1..0xFFFF is valid in the local GATTC cache post-discovery. :contentReference[oaicite:3]{index=3}
    const uint16_t start = 1, end = 0xFFFF;
    esp_gattc_char_elem_t elems[32];
    uint16_t count = 32;
    uint16_t offset = 0;

    // Reset first
    write_handle_ = notify_handle_ = cccd_handle_ = 0;

    // Pull characteristics in batches
    while (true) {
      count = 32;
      auto st = esp_ble_gattc_get_all_char(parent()->get_gattc_if(), parent()->get_conn_id(),
                                           start, end, elems, &count, offset);
      if (st != ESP_OK || count == 0) break;
      for (int i = 0; i < count; i++) {
        const auto &e = elems[i];
        if (e.uuid.len == ESP_UUID_LEN_16) {
          if (e.uuid.uuid.uuid16 == UUID_WRITE_CHAR)  write_handle_  = e.char_handle;
          if (e.uuid.uuid.uuid16 == UUID_NOTIFY_CHAR) notify_handle_ = e.char_handle;
        }
      }
      offset += count;
    }

    ESP_LOGI(TAG, "Resolved: write_handle=0x%04X, notify_handle=0x%04X", write_handle_, notify_handle_);

    if (notify_handle_ != 0) {
      // Try to locate CCCD via ESPHome helper first, then fallback to raw scan.
      if (auto d = parent()->get_config_descriptor(notify_handle_)) {  // CCCD finder helper. :contentReference[oaicite:4]{index=4}
        cccd_handle_ = d->handle;
      }
      if (cccd_handle_ == 0) {
        // Fallback: scan descriptors under notify_handle_ and pick 0x2902
        esp_gattc_descr_elem_t descrs[8];
        uint16_t dcount = 8;
        auto st2 = esp_ble_gattc_get_all_descr(parent()->get_gattc_if(), parent()->get_conn_id(),
                                               notify_handle_, descrs, &dcount, 0);
        if (st2 == ESP_OK) {
          for (int i = 0; i < dcount; i++) {
            if (descrs[i].uuid.len == ESP_UUID_LEN_16 &&
                descrs[i].uuid.uuid.uuid16 == UUID_CCCD_DESCR) {
              cccd_handle_ = descrs[i].handle;
              break;
            }
          }
        }
      }

      if (cccd_handle_ == 0) {
        ESP_LOGW(TAG, "Could not find CCCD for notify handle 0x%04X; notifications won’t arrive.", notify_handle_);
        return;
      }

      // 1) Register for notify; 2) write CCCD=0x0001 (enable notify). :contentReference[oaicite:5]{index=5}
      auto st3 = esp_ble_gattc_register_for_notify(parent()->get_gattc_if(), parent()->get_remote_bda(), notify_handle_);
      if (st3 != ESP_OK) {
        ESP_LOGW(TAG, "register_for_notify failed: %d", st3);
        return;
      }

      uint8_t enable_notify[2] = {0x01, 0x00};
      auto st4 = esp_ble_gattc_write_char_descr(parent()->get_gattc_if(), parent()->get_conn_id(),
                                                cccd_handle_, sizeof(enable_notify), enable_notify,
                                                ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
      if (st4 != ESP_OK) {
        ESP_LOGW(TAG, "write CCCD failed: %d", st4);
      } else {
        ESP_LOGI(TAG, "CCCD enabled (handle 0x%04X). Waiting for key...", cccd_handle_);
      }
    } else {
      ESP_LOGW(TAG, "Notify characteristic 0x%04X not found in cache.", UUID_NOTIFY_CHAR);
    }
  }
};

}  // namespace blackview_lock
}  // namespace esphome
#endif  // USE_ESP32
