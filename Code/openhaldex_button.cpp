#include "openhaldex.h"

void checkSwitchMode() {
  int i = 0;
#if stateDebug
  Serial.println(F("Switch mode button pressed!"));
#endif /* stateDebug */

  while (digitalRead(pinBT_Conf) && (i < 11)) {
    blinkLED(100, 1, 5, 0, 5);
#if stateDebug
    Serial.println(F("2s delay to flipflop standalone!"));
    Serial.println(i);
#endif /* stateDebug */
    i++;
  }

  if (i >= 11) {
#if stateDebug
    Serial.printf("Is Standalone: %d\n", isStandalone);
#endif /* stateDebug */
    isStandalone = !isStandalone;
    blinkLED(500, 11, 5, 0, 5);
    //delay(1500);
  }

  if (btConnected) {
    blinkLED(100, 11, 5, 5, 5);
  } else {
    buttonToggle++;
    Serial.println(buttonToggle);
    if (buttonToggle > 3) {  // if button bashed, possible for buttonToggle to go over 3; just reset (error catching)
      buttonToggle = 0;
    }

    switch (buttonToggle) {
      case 0:
        if (isStandalone) {  // jump over 'stock' since it's Standalone and can't see Body CAN
          buttonToggle++;
          break;
        }
        state.mode_override = false;  // was disabled & false?
        state.mode = MODE_STOCK;
        break;

      case 1:
        state.mode_override = false;
        state.mode = MODE_FWD;
        break;

      case 2:
        state.mode_override = false;
        state.mode = MODE_5050;
        break;

      case 3:
        if (isStandalone) {
          state.mode_override = false;
          state.mode = MODE_FWD;
          buttonToggle = 1;
          break;
        }

        if (isCustom) {
          state.mode_override = false;
          state.mode = MODE_CUSTOM;
          buttonToggle = -1;
          break;
        } else {
          buttonToggle = 0;
          state.mode_override = false;
          state.mode = MODE_STOCK;
          break;
        }
    }
  }

  delay(300);  // reduce button bashing
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