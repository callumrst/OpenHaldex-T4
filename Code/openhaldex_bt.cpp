#include "openhaldex.h"

bool btSendStatus(void *params) {
  bt_packet packet;
  if (!isScreen) {

    packet.data[0] = APP_MSG_STATUS;
    packet.data[1] = 0;  // was haldexStatus
    packet.data[2] = haldexEngagement;  //haldexEngagement;
    packet.data[3] = lockTarget;  //lockTarget;
    packet.data[4] = vehicleSpeed;  //vehicleSpeed;
    //packet.data[5] = state.mode_override; // app software has a limit on len[6], so this stops the app working...
    packet.data[5] = SERIAL_PACKET_END;
    packet.len = 6;

    for (int i = 0; i < packet.len - 1; i++) {
      if (packet.data[i] == SERIAL_PACKET_END) {
        packet.data[i] = SERIAL_PACKET_END - 1;
      }
    }

    /*for (int i = 0; i < packet.len; i++) {
      Serial.println(packet.data[i]);
    }*/
    Serial2.write(packet.data, packet.len);
  }

  if (isScreen) {
    packet.data[0] = APP_MSG_STATUS;
    packet.data[1] = 0;  // was haldexStatus
    packet.data[2] = haldexEngagement;
    packet.data[3] = int(lockTarget);
    packet.data[4] = vehicleSpeed;
    packet.data[5] = state.mode_override;
    packet.data[6] = state.mode;
    packet.data[7] = int(pedValue);
    packet.data[8] = softwareVersion;
    packet.data[9] = SERIAL_PACKET_END;
    packet.len = 10;

    Serial2.write(packet.data, packet.len);
  }
  return true;
}

void btProcess(bt_packet *rx_packet) {
  byte lockpoint_index;
  bt_packet tx_packet;

  lastTransmission = millis();

  switch (rx_packet->data[0]) {
    case APP_MSG_MODE:
      if (state.mode_override) {
        //return;
      }
      state.mode = rx_packet->data[1] <= MODE_CUSTOM ? (openhaldex_mode_id)rx_packet->data[1] : MODE_STOCK;
      state.ped_threshold = rx_packet->data[2];
      if (state.mode == MODE_FWD) {
        lockTarget = 0;
      }

      if (rx_packet->data[1] == MODE_CUSTOM) {
        isCustom = true;
      } else {
        isCustom = false;
      }

#if stateDebug
      Serial.printf("APP_MSG_MODE: mode=%d ped_threshold=%d%%\n", rx_packet->data[1], rx_packet->data[2]);
#endif

      break;

    case APP_MSG_CUSTOM_DATA:
      lockpoint_index = rx_packet->data[1];

      if (lockpoint_index < NUM_LOCK_POINTS) {
        state.custom_mode.lockpoints[lockpoint_index].speed = rx_packet->data[2];
        state.custom_mode.lockpoints[lockpoint_index].lock = rx_packet->data[3];
        state.custom_mode.lockpoints[lockpoint_index].intensity = rx_packet->data[4];

        if (lockpoint_index > 6) {
          state.custom_mode.lockpoint_rx_h |= (1 << (lockpoint_index - 7));
        } else {
          state.custom_mode.lockpoint_rx_l |= (1 << lockpoint_index);
        }

        state.custom_mode.lockpoint_count++;

#if stateDebug
        Serial.printf("lockpoint[%d] low 0x%x high 0x%x (count %d)\n",
                      lockpoint_index,
                      state.custom_mode.lockpoint_rx_l,
                      state.custom_mode.lockpoint_rx_h,
                      state.custom_mode.lockpoint_count);
#endif
      }
      break;

    case APP_MSG_CUSTOM_CTRL:
      switch (rx_packet->data[1]) {
        case DATA_CTRL_CHECK_LOCKPOINTS:
          tx_packet.data[0] = APP_MSG_CUSTOM_CTRL;
          tx_packet.data[1] = DATA_CTRL_CHECK_LOCKPOINTS;
          tx_packet.data[2] = state.custom_mode.lockpoint_rx_l;
          tx_packet.data[3] = state.custom_mode.lockpoint_rx_h;
          tx_packet.data[4] = SERIAL_PACKET_END;
          tx_packet.len = 5;

          Serial2.write(tx_packet.data, tx_packet.len);
          break;
        case DATA_CTRL_CLEAR:
          state.custom_mode.lockpoint_rx_l = 0;
          state.custom_mode.lockpoint_rx_h = 0;
          state.custom_mode.lockpoint_count = 0;
          memset(state.custom_mode.lockpoints, 0, sizeof(state.custom_mode.lockpoints));
          break;
        case DATA_CTRL_CHECK_MODE:
          tx_packet.data[0] = APP_MSG_CUSTOM_CTRL;
          tx_packet.data[1] = DATA_CTRL_CHECK_MODE;
          tx_packet.data[2] = state.mode;
          tx_packet.data[3] = state.ped_threshold;
          tx_packet.data[4] = SERIAL_PACKET_END;
          tx_packet.len = 5;

          Serial2.write(tx_packet.data, tx_packet.len);
          break;
        case DATA_CTRL_RECONNECT_BT:
          state.mode_override = false;
          Serial.println("App assumed control of override");
          break;
      }
      break;
    case APP_MSG_IS_SCREEN:
      isScreen = true;
      state.mode_override = false;
      break;
  }
}

void btInit() {
#if stateDebug
  uint8_t at_buf[128] = { 0 };  // allocate buffer for Bluetooth module Serial messages
#endif                          /* stateDebug */

  // drive all LEDs low to save power!
  digitalWrite(pinLED_R, LOW);
  digitalWrite(pinLED_G, LOW);
  digitalWrite(pinLED_B, LOW);

#if stateDebug
  Serial.println(F("Entering Bluetooth setup mode..."));  // start Bluetooth setup mode after init reset/conf 'buttons'
#endif                                                    /* stateDebug */

  pinMode(pinBT_Conf, OUTPUT);
  pinMode(pinBT_Reset, OUTPUT);
  digitalWrite(pinBT_Reset, LOW);
  digitalWrite(pinBT_Conf, HIGH);
  //delay(2500);
  blinkLED(625, 4, 10, 0, 0);  // 2500ms total 'high' time for reset, blink LED takes 625ms to complete (625x4=2500ms)
  //digitalWrite(pinBT_Reset, LOW);
  pinMode(pinBT_Reset, INPUT);
  delay(2500);

  Serial2.end();         // end current (if any) Serial2/Bluetooth connections
  Serial2.begin(38400);  // AT mode requires Baud 38400

  Serial2.write("AT\r\n");  // confirm in AT mode
#if stateDebug
  Serial.println(F("AT"));
  while (!Serial2.available()) {}
  Serial2.readBytesUntil('\r', at_buf, ARRAY_SIZE(at_buf));
  Serial.printf("%s\r\n", at_buf);
#endif /* stateDebug */
  blinkLED(312, 1, 10, 0, 0);

  Serial2.write("AT+UART=9600,0,0\r\n");  // set baud at 38400
#if stateDebug
  Serial.println(F("AT+UART=9600,0,0"));
  while (!Serial2.available()) {}
  Serial2.readBytesUntil('\n', at_buf, ARRAY_SIZE(at_buf));
  Serial.printf("%s\r\n", at_buf);
#endif /* stateDebug */
  blinkLED(312, 1, 10, 0, 0);

  Serial2.write("AT+NAME=OpenHaldexT4\r\n");  // set Bluetooth name
#if stateDebug
  Serial.println(F("AT+NAME=OpenHaldexT4"));
  while (!Serial2.available()) {}
  Serial2.readBytesUntil('\n', at_buf, ARRAY_SIZE(at_buf));
  Serial.printf("%s\r\n", at_buf);
#endif /* stateDebug */
  blinkLED(312, 1, 10, 0, 0);

  Serial2.write("AT+ROLE=0\r\n");  // query current role
#if stateDebug
  Serial.println(F("AT+ROLE=0"));
  while (!Serial2.available()) {}
  Serial2.readBytesUntil('\n', at_buf, ARRAY_SIZE(at_buf));
  Serial.printf("%s\r\n", at_buf);
#endif /* stateDebug */
  blinkLED(312, 1, 10, 0, 0);

  Serial2.write("AT+RESET\r\n");  // reset Bluetooth module/connections
#if stateDebug
  Serial.println(F("AT+RESET"));
  while (!Serial2.available()) {}
  Serial2.readBytesUntil('\n', at_buf, ARRAY_SIZE(at_buf));
  Serial.printf("%s\r\n", at_buf);
#endif /* stateDebug */
  blinkLED(312, 1, 10, 0, 0);

  Serial2.end();          // end AT mode
  Serial2.begin(baudBT);  // begin normal mode with the above baud

  pinMode(pinBT_Conf, INPUT);

#if stateDebug
  Serial.println(F("Bluetooth initialised!"));
#endif /* stateDebug */
}
