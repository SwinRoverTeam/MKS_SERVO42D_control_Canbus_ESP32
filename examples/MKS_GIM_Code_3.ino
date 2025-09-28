#include <Arduino.h>
#include "driver/twai.h"

bool init_twai(gpio_num_t tx_pin, gpio_num_t rx_pin);
int  send_can_msg(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr);
void can_check_recv();

// ---- CAN pins ----
#define CAN_TX GPIO_NUM_43   // ESP32-S3 TX
#define CAN_RX GPIO_NUM_44   // ESP32-S3 RX

#define MKS_CAN_ID 0x02    // Servo42D node ID

// CRC helper for MKS
uint8_t mks_crc(uint16_t id, const uint8_t *payload, uint8_t len) {
  uint32_t sum = id & 0x7FF;
  for (uint8_t i = 0; i < len; i++) sum += payload[i];
  return (uint8_t)(sum & 0xFF);
}

// Generic CAN send (with CRC for MKS)
int send_can_msg(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr) {
  twai_message_t msg = {};
  msg.identifier = can_id;
  msg.extd       = 0;             // standard 11-bit
  msg.rtr        = rtr ? 1 : 0;   // set for info (RTR) frames
  msg.self       = 0;
  msg.ss         = 0;

  if (len > 8) len = 8;
  msg.data_length_code = len;

  if (!rtr && len > 0 && data != nullptr) {
    memcpy(msg.data, data, len);
    msg.data[len] = mks_crc(can_id, data, len);
    msg.data_length_code = len + 1;
  }

  if (twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
    return 0; // success
  } else {
    return -1; // fail
  }
}

// --------- CAN init ----------
bool init_twai(gpio_num_t tx, gpio_num_t rx) {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 20;
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // canbus set to 500kbit/s
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
      Serial.println("TWAI driver install failed");
      return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("TWAI start failed");
    return false;
  }
  Serial.println("TWAI started @ 500 kbit/s");
  return true;
}

// -------------------------------------------------------------------
// RX pump  (only for MKS feedback sofar)
void can_check_recv() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    if (!msg.rtr && msg.identifier == MKS_CAN_ID && msg.data_length_code > 0) {
      uint8_t cmd = msg.data[0];
      if (cmd == 0x31 && msg.data_length_code >= 4) {
        int32_t encoder = (int32_t)(
          msg.data[1] | (msg.data[2] << 8) | (msg.data[3] << 16)
        );
        long revo = encoder / 16385;
        Serial.printf("[MKS] Encoder = %ld pulses (rev %ld)\n", encoder, revo);
      }
      else if (cmd == 0x32 && msg.data_length_code >= 3) {
        int16_t rpm = msg.data[1] | (msg.data[2] << 8);
        Serial.printf("[MKS] RPM = %d\n", rpm);
      }
      else {
        Serial.print("[MKS] RX unknown | ");
        for (int i = 0; i < msg.data_length_code; i++) {
          Serial.printf("%02X ", msg.data[i]);
        }
        Serial.println();
      }
    }
  }
}

// -------------------------------------------------------------------
// MKS COMMANDS
void mks_enable(bool en) {
  uint8_t p[] = {0xF3, (uint8_t)(en ? 1 : 0)};
  send_can_msg(MKS_CAN_ID, sizeof(p), p, false);
}
void mks_set_mode(uint8_t mode) {
  uint8_t p[] = {0x82, mode};
  send_can_msg(MKS_CAN_ID, sizeof(p), p, false);
}
void mks_move_absolute_axis(int32_t axis, uint16_t speed, uint8_t acc) {
  uint8_t p[] = {
    0xF5,
    (uint8_t)((speed >> 8) & 0xFF),
    (uint8_t)(speed & 0xFF),
    acc,
    (uint8_t)((axis >> 16) & 0xFF),
    (uint8_t)((axis >> 8) & 0xFF),
    (uint8_t)(axis & 0xFF)
  };
  send_can_msg(MKS_CAN_ID, sizeof(p), p, false);
}
void mks_speed_stop_pos(uint8_t acc) {
  uint8_t p[] = {0xF5, 0x00, 0x00, acc, 0x00, 0x00, 0x00};
  send_can_msg(MKS_CAN_ID, sizeof(p), p, false);
}
void mks_read_encoder() {
  uint8_t p[] = {0x31};
  send_can_msg(MKS_CAN_ID, sizeof(p), p, true);
}
void mks_read_rpm() {
  uint8_t p[] = {0x32};
  send_can_msg(MKS_CAN_ID, sizeof(p), p, true);
}
void mks_home() {
  uint8_t p[] = {0x91};
  send_can_msg(MKS_CAN_ID, sizeof(p), p, false);
}
void mks_speed_run(uint8_t dir, uint16_t speed, uint8_t acc) {
  if (speed > 3000) speed = 3000;
  uint8_t b2 = ((dir & 0x01) << 7) | ((speed >> 8) & 0x0F);
  uint8_t b3 = speed & 0xFF;
  uint8_t p[] = {0xF6, b2, b3, acc};
  send_can_msg(MKS_CAN_ID, sizeof(p), p, true);
}
void mks_speed_stop(uint8_t acc) {
  uint8_t p[] = {0xF6, 0x00, 0x00, acc};
  send_can_msg(MKS_CAN_ID, sizeof(p), p, false);
}

// -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 CAN -> MKS SERVO42D");

  if (!init_twai(CAN_TX, CAN_RX)) {
    Serial.println("TWAI init failed");
    while (1) delay(1000);
  }
  Serial.println("CAN up, MKS ready");

  mks_set_mode(5);    // FOC mode
  mks_enable(true);   // enable motor
}

String cmd; // serial command buffer
unsigned long lastRequest = 0;
// -------------------------------------------------------------------
void loop() {
  can_check_recv();

  //request encoder and RPM every 3s
  if (millis() - lastRequest > 3000) {
    mks_read_encoder();
    delay(5);
    //mks_read_RPM();
    lastRequest = millis();
  }

  if (Serial.available()) {
    cmd = Serial.readStringUntil('\n');
    cmd.toLowerCase();
    cmd.trim();

    if (cmd.equalsIgnoreCase("enable")) {
      Serial.println("Motor enabled");
      mks_enable(true);

    } else if (cmd.equalsIgnoreCase("disable")) {
      Serial.println("Motor disabled");
      mks_enable(false);

    } else if (cmd.equalsIgnoreCase("run")) {
      Serial.println("Run motor at 100 RPM (example)");
      mks_speed_run(1, 400, 2); // dir, RPM, acc

    } else if (cmd.equalsIgnoreCase("stop")) {
      Serial.println("Stop motor");
      mks_speed_stop(0); // acc

    } else if (cmd.startsWith("mode ")) {
      int mode = cmd.substring(5).toInt();
      Serial.printf("change mode: %d\n", mode);
      mks_set_mode((uint8_t)mode);

    // position control by revolutions
    } else if (cmd.startsWith("rev ")) {
      float revs = cmd.substring(4).toFloat();
      int32_t axis = (int32_t)(revs * 16385.0f);
      Serial.printf("Move to axis: %ld (revs %.3f)\n", (long)axis, revs);
      // pos 16385 is 1 full revolution
      // pos 511 is max revs we can do -511 is max on other end (be careful)
      mks_move_absolute_axis(axis, 2000, 250);  // F5h
      mks_read_encoder();

    // position control by degrees
    } else if (cmd.startsWith("deg ")) {
      float deg = cmd.substring(4).toFloat();
      int32_t axis = (int32_t)(deg * (16385.0f / 360.0f));
      Serial.printf("Move to axis: %ld (deg %.3f)\n", (long)axis, deg);
      mks_move_absolute_axis(axis, 2000, 250);  // F5h
      mks_read_encoder();

    } else if (cmd.equalsIgnoreCase("pos stop")) {
      Serial.println("Stop motor (pos)");
      mks_speed_stop_pos(0); // acc

    } else if (cmd.equalsIgnoreCase("home")) {
      Serial.println("Go Home");
      mks_home();

    } else {
      Serial.println("Unknown Command: TRY: enable / disable / run / stop / mode <n> / rev <revs> / deg <deg> / pos stop / home");
    }
  // You 
  }
}