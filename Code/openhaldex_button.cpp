#include "openhaldex.h"
/*
Button handling - consider changing to a button lib 
*/

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
    lastMode++;
    if (lastMode > 3) {  // if button bashed, possible for buttonToggle to go over 3; just reset (error catching)
      lastMode = 0;
    }

    switch (lastMode) {
      case 0:
        if (isStandalone) {  // jump over 'stock' since it's Standalone and can't see Body CAN
          lastMode++;
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
          lastMode = 1;
          break;
        }

        if (isCustom) {
          state.mode_override = false;
          state.mode = MODE_CUSTOM;
          lastMode = 3;
          break;
        } else {
          lastMode = 0;
          state.mode_override = false;
          state.mode = MODE_STOCK;
          break;
        }
    }
  }

  delay(300);  // reduce button bashing
}