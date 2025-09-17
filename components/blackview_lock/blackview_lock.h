// ... same includes ...

namespace esphome {
namespace blackview_lock {

// ... same using/consts as your last good build ...

class BlackviewLock : public Component, public BLEClientNode {
 public:
  // existing sensor setters...
  void set_session_key_text_sensor(TextSensor *t) { session_key_text_ = t; }
  void set_last_notify_text_sensor(TextSensor *t) { last_notify_text_ = t; }
  void set_key_received_binary_sensor(BinarySensor *b) { key_received_ = b; }
  void set_connected_binary_sensor(BinarySensor *b) { connected_ = b; }
  void set_notify_count_sensor(Sensor *s) { notify_count_ = s; }

  // NEW: config setters
  void set_fallback_write_handle(uint16_t h) { fallback_write_handle_ = h; }
  void set_fallback_notify_handle(uint16_t h) { fallback_notify_handle_ = h; }
  void set_fallback_cccd_handle(uint16_t h) { fallback_cccd_handle_ = h; }
  void set_prefer_write_no_rsp(bool v) { prefer_write_no_rsp_ = v; }
  void set_write_uuid(const std::string &s) { write_uuid_ = s; }
  void set_notify_uuid(const std::string &s) { notify_uuid_ = s; }

  // ... setup(), dump_config(), request_handshake(), loop() unchanged ...

  void gattc_event_handler(esp_gattc_cb_event_t event,
                           esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override {
    switch (event) {
      case ESP_GATTC_OPEN_EVT: {
        if (param->open.status == ESP_GATT_OK) {
          connect_open_ms_ = millis();
          auto_tries_ = 0;
          if (connected_) connected_->publish_state(true);
          ESP_LOGI(TAG, "Connected!");

          // EARLY subscribe + HELLO (BEFORE discovery), if we have fallbacks
          early_subscribe_and_hello_(gattc_if, param->open.conn_id);

          // As a fallback, also fire once more when handles are resolved
          last_hello_ms_ = millis();
        } else {
          ESP_LOGW(TAG, "OPEN_EVT status=%d", param->open.status);
        }
        break;
      }

      // ... CLOSE/DISCONNECT/WRITE events unchanged ...

      case ESP_GATTC_SEARCH_CMPL_EVT: {
        if (param->search_cmpl.status == ESP_GATT_OK) {
          ESP_LOGI(TAG, "Service discovery complete; resolving handles...");
          this->resolve_handles_and_subscribe_();
          if (write_handle_ != 0 && !got_key_) {
            // one re-poke after CCCD via resolved handle
            send_hello_on_(parent()->get_gattc_if(), parent()->get_conn_id(),
                           write_handle_, prefer_no_rsp_ /*NO_RSP?*/, "resolved");
            last_hello_ms_ = millis();
          }
        } else {
          ESP_LOGW(TAG, "SEARCH_CMPL status=%d", param->search_cmpl.status);
        }
        break;
      }

      // ... NOTIFY handler unchanged ...

      default:
        break;
    }
  }

 private:
  // sensors...
  TextSensor *session_key_text_ = nullptr, *last_notify_text_ = nullptr;
  BinarySensor *key_received_ = nullptr, *connected_ = nullptr;
  Sensor *notify_count_ = nullptr;

  // resolved handles
  uint16_t write_handle_ = 0, notify_handle_ = 0, cccd_handle_ = 0;

  // NEW: fallback handles + flag
  uint16_t fb_write_ = 0, fb_notify_ = 0, fb_cccd_ = 0;
  bool prefer_no_rsp_ = true;
  uint16_t fallback_write_handle_{0};
  uint16_t fallback_notify_handle_{0};
  uint16_t fallback_cccd_handle_{0};
  bool prefer_write_no_rsp_{false};
  std::string write_uuid_{"00002b11-0000-1000-8000-00805f9b34fb"};
  std::string notify_uuid_{"00002b10-0000-1000-8000-00805f9b34fb"};

  // state...
  bool got_key_ = false;
  uint32_t last_hello_ms_ = 0, connect_open_ms_ = 0, notify_counter_ = 0;
  uint8_t auto_tries_ = 0;
  static constexpr uint32_t kHelloIntervalMs = 8000;
  static constexpr uint8_t  kMaxAutoTries    = 6;
  static constexpr uint32_t kPairingWatchdogMs = 90000;

  // --- early path: beat service discovery ---
  void early_subscribe_and_hello_(esp_gatt_if_t gattc_if, uint16_t conn_id) {
    if (fb_notify_ && fb_cccd_) {
      esp_ble_gattc_register_for_notify(gattc_if, parent()->get_remote_bda(), fb_notify_);
      uint8_t en[2] = {0x01, 0x00};
      esp_ble_gattc_write_char_descr(gattc_if, conn_id, fb_cccd_, sizeof(en), en,
                                     ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
      ESP_LOGI(TAG, "Early-CCCD enable sent (notify=0x%04X, cccd=0x%04X)", fb_notify_, fb_cccd_);
    }
    if (fb_write_) {
      send_hello_on_(gattc_if, conn_id, fb_write_, /*no_rsp?*/true, "fallback");
    }
  }

  // --- send helpers ---
  static void build_payload_(std::vector<uint8_t> &payload) {
    uint64_t rnd = ((uint64_t) esp_random() << 32) | esp_random();
    for (int i = 0; i < 8; i++) payload.push_back((rnd >> (8 * i)) & 0xFF);
    payload.push_back(0x01); payload.push_back(0x90);
    payload.push_back(0x00); payload.push_back(0x00);
  }
  static void append_crc_xmodem_le_(std::vector<uint8_t> &buf,_

