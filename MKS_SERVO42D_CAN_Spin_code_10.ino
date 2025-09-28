#include "driver/twai.h"

#define CAN_TX GPIO_NUM_43   // ESP32-S3 CAN TX
#define CAN_RX GPIO_NUM_44   // ESP32-S3 CAN RX
#define MOTOR_CAN_ID 0x01        // CAN ID

// CRC helper
uint8_t mks_crc(uint16_t id,const uint8_t *payload, uint8_t len) {
  uint32_t sum = id & 0x7FF;// frame header
  for (uint8_t i = 0; i < len; i++) sum += payload[i];
  return (uint8_t)(sum & 0xFF);
}

// Send CAN frame
bool mks_send(uint16_t id, const uint8_t *payload, uint8_t len) {
  if (len == 0 || len > 8) return false;
  twai_message_t tx_msg = {};
  tx_msg.identifier = id;
  tx_msg.extd = 0;
  tx_msg.rtr = 0;
  tx_msg.data_length_code = len + 1;

  memcpy(tx_msg.data, payload, len);
  tx_msg.data[len] = mks_crc(id, payload, len);

  if (twai_transmit(&tx_msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
    return true;
  } else {
    return false;
  }
}

// MKS COMMANDS
// Set Mode
void mks_set_mode(uint8_t mode) {
  uint8_t p[] = {0x82, mode};
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}

//Encoder value
void mks_read_encoder() {
  uint8_t p[] = {0x31};
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}
//RPM value (CCW >0, CW <0)
void mks_read_RPM() {
  uint8_t p[] = {0x32};
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}
// Enable/disable
void mks_enable(bool en) {
  uint8_t p[] = {0xF3, (uint8_t)(en ? 1 : 0)};
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}

// Speed run
void mks_speed_run(uint8_t dir, uint16_t speed, uint8_t acc) {
  //if (speed > 3000) speed = 3000;
  uint8_t b2 = ((dir & 0x01) << 7) | ((speed >> 8) & 0x0F);
  uint8_t b3 = speed & 0xFF;
  uint8_t p[] = {0xF6, b2, b3, acc};
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}

// Stop motor
void mks_speed_stop(uint8_t acc) {
  uint8_t p[] = {0xF6, 0x00, 0x00, acc};
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}

// Absolute position move
void mks_move_absolute_axis(int32_t axis, uint16_t speed, uint8_t acc) {
  if (speed > 3000) speed = 3000;

  uint8_t p[] = {
    0xF5,//maybe use FE
    (uint8_t)((speed >> 8)),
    (uint8_t)(speed),        // speed low
    acc,
    (uint8_t)((axis >> 16)),
    (uint8_t)((axis >> 8)),
    (uint8_t)(axis)         // axis LSB
  };
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}
/*
void mks_move_absolute_axis(int32_t axis,uint16_t speed,uint8_t acc) {
  uint8_t p[] = {
  0xF4,uint8_t(speed>>8),uint8_t(speed),acc,uint8_t(axis>>16),uint8_t(axis>>8),uint8_t(axis)
  };
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}
*/
// Absolute position move by pulses (FEh)
void mks_move_absolute_pulses(int32_t pulses, uint16_t speed, uint8_t acc) {
  if (speed > 3000) speed = 3000;

  uint8_t p[] = {
    0xFE,                              // function: absolute pulses mode
    (uint8_t)((speed >> 8) & 0xFF),    // speed high
    (uint8_t)(speed & 0xFF),           // speed low
    acc,
    (uint8_t)((pulses >> 24) & 0xFF),  // pulses MSB
    (uint8_t)((pulses >> 16) & 0xFF),
    (uint8_t)((pulses >> 8) & 0xFF),
    (uint8_t)(pulses & 0xFF)           // pulses LSB
  };
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}

void mks_speed_stop_pos(uint8_t acc) {
  uint8_t p[] = {0xF5, 0x00, 0x00, acc, 0x00,0x00,0x00,0x00};
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}

void mks_speed_stop_pulse(uint8_t acc) {
  uint8_t p[] = {0xFE, 0x00, 0x00, acc, 0x00,0x00,0x00,0x00};
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}
//Go Home (using endstop)
void mks_home() {
  uint8_t p[] = {0x91};
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}

void mks_query_status() {
  uint8_t p[] = {0xF1};
  mks_send(MOTOR_CAN_ID, p, sizeof(p));
}


unsigned long lastRequest = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 CAN + MKS Servo Starting...");

  // Configure CAN
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
      twai_start() == ESP_OK) {
    Serial.println("CAN bus initialized at 500 kbps.");
  } else {
    Serial.println("Failed to init CAN.");
    while (1) delay(100);
  }

  // Init motor
  mks_set_mode(5);     //5 SR_vFOC mode //maybe change to 2 CR_vFOC (must be in CR_vFOV)
  delay(50);
  mks_enable(true);    //Enable motor
  delay(50);
}

String cmd;

void loop() {
/*
  //request encoder and RPM every 3s
  if (millis() - lastRequest > 3000) {
    mks_read_encoder();
    delay(5);
    mks_read_RPM();
    lastRequest = millis();
  }

  // RX Handling
  twai_message_t rx_msg;
  if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
    if (rx_msg.data_length_code >= 2) {
      uint8_t cmd = rx_msg.data[0]; // first byte = command
      // Encoder feedback
      if (cmd == 0x31 && rx_msg.data_length_code >= 5) {
        int32_t encoder = (int32_t)(
          ((uint32_t)rx_msg.data[1]) |
          ((uint32_t)rx_msg.data[2] << 8) |
          ((uint32_t)rx_msg.data[3] << 16) |
          ((uint32_t)rx_msg.data[4] << 24)
        );
        Serial.printf("Encoder Position = %ld pulses\n", encoder);
      }
      // RPM feedback
      else if (cmd == 0x32 && rx_msg.data_length_code >= 3) {
        int16_t rpm = (int16_t)(
          ((uint16_t)rx_msg.data[1]) |
          ((uint16_t)rx_msg.data[2] << 8)
        );
        Serial.printf("Motor Speed = %d RPM\n", rpm);
      }
      else {
        // Unknown/other response
        Serial.print("RX ID: 0x");
        Serial.print(rx_msg.identifier, HEX);
        Serial.print(" | Len: ");
        Serial.print(rx_msg.data_length_code);
        Serial.print(" | Data: ");
        for (int i = 0; i < rx_msg.data_length_code; i++) {
          Serial.printf("%02X ", rx_msg.data[i]);
        }
        Serial.println();
      }
    }
  }
*/
  
  twai_message_t rx_msg;
  if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
    Serial.print("RX ID: 0x");
    Serial.print(rx_msg.identifier, HEX);
    Serial.print(" | Len: ");
    Serial.print(rx_msg.data_length_code);
    Serial.print(" | Data: ");
    for (int i = 0; i < rx_msg.data_length_code; i++) {
      Serial.printf("%02X ", rx_msg.data[i]);
    }
    Serial.println();
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
      Serial.println("Run motor at 100 RPM");
      mks_speed_run(1, 400, 2); //dir, RPM, acc 
      //4000 is max rpm it goes but at 4100 rpm it does a cool thing

    } else if (cmd.equalsIgnoreCase("stop")) {
      Serial.println("Stop motor");
      mks_speed_stop(0); //acc

    } else if (cmd.equalsIgnoreCase("mode")) {
      int mode = cmd.substring(5).toInt();
      Serial.printf("cnahe mode: %d mode\n", mode);
      mks_set_mode(mode);


    // postioton control by revolutions
    } else if (cmd.startsWith("rev ")) {
      float revs = cmd.substring(4).toFloat();
      float axis = revs * (16385); 
      Serial.printf("Move to axis: %ld (revs%ld)\n", axis, revs);
      //pos 16385 is 1 full revolution 
      // pos 511 is max revs we can do -511 is max on other end
      mks_move_absolute_axis(axis, 2000, 250);  // F5h
      mks_read_encoder();
    

    // postioton control by degrees
    } else if (cmd.startsWith("deg ")) {
      float deg = cmd.substring(4).toFloat();
      float axis = deg * (16385/360); 
      Serial.printf("Move to axis: %ld (revs%ld)\n", axis, deg);
      //pos 16385 is 1 full revolution 
      // pos 511 is max revs we can do -511 is max on other end
      mks_move_absolute_axis(axis, 2000, 250);  // F5h
      mks_read_encoder();

    } else if (cmd.equalsIgnoreCase("pos stop")) {
      Serial.println("Stop motor");
      mks_speed_stop_pos(0); //acc

    } else if (cmd.equalsIgnoreCase("home")) {
      Serial.println("Go Home");
      mks_home();

    } else {
      Serial.println("Unkown Command: TRY: enable / disable / run / stop / pos <revs> / home / pos stop");
    }
  }
}
