#include "openhaldex.h"

bool printMode(void *params) {
  Serial.printf("OpenHaldex Mode = %d\n", state.mode);
  return true;
}

void basicInit() {
#if stateDebug
  Serial.begin(baudSerial);
  Serial.println(F("\nOpenHaldex - Teensy 4.0 Initialisation..."));
  Serial.printf("Running at %dMHz\r\n", F_CPU_ACTUAL / (1000 * 1000));
#endif /* stateDebug */

#if stateDebug
  Serial.println(F("CAN initialising..."));
#endif
  canInit();  // begin CAN
#if stateDebug
  Serial.println(F("CAN initialised!"));
#endif
  readEEP();  // read EEPROM for saved preferences
  setupTimers();
  setupPins();  // setup IO

#if stateDebug
  Serial.println(F("Bluetooth initialising..."));
#endif
  Serial2.begin(baudBT);
#if stateDebug
  Serial.println(F("Bluetooth initialised!"));
#endif
}

void setupPins() {
  // Disable unwanted Teensy optionals
  CCM_ANALOG_PLL_AUDIO |= CCM_ANALOG_PLL_AUDIO_POWERDOWN;
  CCM_ANALOG_PLL_VIDEO |= CCM_ANALOG_PLL_VIDEO_POWERDOWN;
  CCM_ANALOG_PLL_ENET |= CCM_ANALOG_PLL_ENET_POWERDOWN;

  // Turn off the power LED for power saving
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup the RGB LED pins for outputs
  pinMode(pinLED_R, OUTPUT);
  pinMode(pinLED_G, OUTPUT);
  pinMode(pinLED_B, OUTPUT);

  // Setup the switches (Switch Mode & Bluetooth) for interrupt / inputs (keeps response quick)
  // can't have BT_Conf as an interrupt and change the pin state...
  pinMode(pinBT_Conf, INPUT);

  // if BT Conf is high on boot, flash 'Haldex Gen' as a number (1 flash = Gen1, 2 flashes = Gen2...)
  if (digitalRead(pinBT_Conf)) {
    blinkLED(2000, haldexGen, 255, 0, 0);

    // since BT_Conf is tied to the HC05 it goes into AT mode during power up, so reset it back at normal AT mode...
    pinMode(pinBT_Conf, OUTPUT);
    pinMode(pinBT_Reset, OUTPUT);
    digitalWrite(pinBT_Reset, LOW);
    digitalWrite(pinBT_Conf, LOW);
    delay(2500);
    pinMode(pinBT_Reset, INPUT);
    pinMode(pinBT_Conf, INPUT);
  }

  attachInterrupt(pinSwitchMode, checkSwitchMode, HIGH);
}

void LED() {
  switch (state.mode) {
    case MODE_STOCK:
      analogWrite(pinLED_R, 10);
      analogWrite(pinLED_G, 0);
      analogWrite(pinLED_B, 0);
      break;

    case MODE_FWD:
      analogWrite(pinLED_R, 0);
      analogWrite(pinLED_G, 10);
      analogWrite(pinLED_B, 0);
      break;

    case MODE_5050:
      analogWrite(pinLED_R, 0);
      analogWrite(pinLED_G, 0);
      analogWrite(pinLED_B, 10);
      break;

    case MODE_CUSTOM:
      analogWrite(pinLED_R, 5);
      analogWrite(pinLED_G, 0);
      analogWrite(pinLED_B, 5);
      break;
  }
}

void blinkLED(int duration, int flashes, int R, int G, int B) {
  for (int i = 0; i < flashes; i++) {
    delay(duration);
    analogWrite(pinLED_R, R);
    analogWrite(pinLED_G, G);
    analogWrite(pinLED_B, B);
    delay(duration);
    analogWrite(pinLED_R, 0);
    analogWrite(pinLED_G, 0);
    analogWrite(pinLED_B, 0);
  }
}