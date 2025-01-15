#include "openhaldex.h"
/* Bluetooth handling */

// AT commands to send while in AT Mode / HC05 setup.  Capture all in a String array for easier sending
String atCommands[8] = { "AT",
                         "AT+UART=9600,0,0",
                         "AT+NAME=OpenHaldexT4",
                         "AT+ROLE=0",
                         "AT+CMODE=0",
                         "AT+CLASS=0",
                         "AT+RMAAD",
                         "AT+RESET" };

bool btSendStatus(void *params) {
  bt_packet packet;
  if (!isScreen) {  // if not screen, must be the app...
    packet.data[0] = APP_MSG_STATUS;    // '1'
    packet.data[1] = 0;                 // was haldexStatus
    packet.data[2] = haldexEngagement;  //haldexEngagement;
    packet.data[3] = lockTarget;        //lockTarget;
    packet.data[4] = vehicleSpeed;      //vehicleSpeed;
    //packet.data[5] = state.mode_override; // app software has a limit on len[6], so this stops the app working...
    packet.data[5] = serialPacketEnd;
    packet.len = 6;

    for (int i = 0; i < packet.len - 1; i++) {
      if (packet.data[i] == serialPacketEnd) {
        packet.data[i] = serialPacketEnd - 1; // not really sure on the purpose of this?  0xFF-1?
      }
    }
    Serial2.write(packet.data, packet.len);
  }

  if (isScreen) {  // must be the screen...
    packet.data[0] = APP_MSG_STATUS;
    packet.data[1] = 0;  // was haldexStatus
    packet.data[2] = haldexEngagement;
    packet.data[3] = int(lockTarget);
    packet.data[4] = vehicleSpeed;
    packet.data[5] = state.mode_override;
    packet.data[6] = state.mode;
    packet.data[7] = int(pedValue);
    packet.data[8] = softwareVersion;
    packet.data[9] = serialPacketEnd;
    packet.len = 10;

    Serial2.write(packet.data, packet.len);
  }
  return true;
}

void btGetStatus(bt_packet *rx_packet) {
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
          tx_packet.data[4] = serialPacketEnd;
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
          tx_packet.data[4] = serialPacketEnd;
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
  uint8_t at_buf[128] = { 0 };  // allocate buffer for Bluetooth module Serial messages

  // drive all LEDs low to save power - HC05 uses a lot of current on setup
  digitalWrite(pinLED_R, LOW);
  digitalWrite(pinLED_G, LOW);
  digitalWrite(pinLED_B, LOW);

#if stateDebug
  Serial.println(F("Entering Bluetooth setup mode..."));  // start Bluetooth setup (AT) mode after init reset/conf 'buttons'
#endif 

  pinMode(pinBT_Conf, OUTPUT);
  pinMode(pinBT_Reset, OUTPUT);
  digitalWrite(pinBT_Reset, LOW);
  digitalWrite(pinBT_Conf, HIGH);
  blinkLED(625, 4, 10, 0, 0);  // 2500ms total 'high' time for reset, blink LED takes 625ms to complete (625x4=2500ms)
  pinMode(pinBT_Reset, INPUT);
  delay(2500);

  Serial2.end();         // end current (if any) Serial2/Bluetooth connections
  Serial2.begin(38400);  // now in AT mode which requires baudrate 38400

  // write each of the AT commands (above)
  for (int i = 0; i < 8; i++) {
    at_buf[0] = '\0';

#if stateDebug
    Serial.println(atCommands[i]);
#endif
    Serial2.println(atCommands[i]);
    blinkLED(312, 1, 10, 0, 0); // show 'progress' with flashing LED, otherwise it looks like the module is doing nothing...
  }

  while (!Serial2.available()) {}
  Serial2.readBytesUntil('\n', at_buf, arraySize(at_buf));

  pinMode(pinBT_Reset, INPUT);
  pinMode(pinBT_Conf, OUTPUT);
  digitalWrite(pinBT_Conf, LOW);
  delay(btTimeout);

#if stateDebug
  Serial.println(F("Ending pairing..."));
#endif

  Serial2.end();
  Serial2.begin(baudBT);

#if stateDebug
  Serial.println(F("Bluetooth initialised!"));
#endif /* stateDebug */
}
