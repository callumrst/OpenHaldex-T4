#include "openhaldex.h"

void readEEP() {
#if stateDebug
  Serial.println(F("EEPROM initialising!"));
#endif /* stateDebug */

  if (EEPROM.read(0) == 255) {
// first run comes with EEP valve of 255, so write actual values and clear 'isCustom' & 'lastMode'
#if stateDebug
    Serial.println(F("First run, set Bluetooth module, write Software Version etc"));
#endif /* stateDebug */

    btInit();
    EEPROM.write(0, softwareVersion);      // EEP Address 0: SW Version
    EEPROM.write(1, 0);                    // EEP Address 1: isCustom
    EEPROM.write(2, 0);                    // EEP Address 2: lastMode
    EEPROM.write(3, 0);                    // EEP Address 2: isStandalone
    EEPROM.write(4, state.ped_threshold);  // EEP Address 3: Pedal Threshold
  } else {
    softwareVersion = EEPROM.read(0);
    isCustom = EEPROM.read(1);

    switch (EEPROM.read(2)) {
      case 0:
        state.mode = MODE_STOCK;
        lastMode = 0;
        break;
      case 1:
        state.mode_override = true;
        state.mode = MODE_FWD;
        lastMode = 1;
        break;
      case 2:
        state.mode_override = true;
        state.mode = MODE_5050;
        lastMode = 2;
        break;
      case 3:
        state.mode_override = true;
        state.mode = MODE_CUSTOM;
        lastMode = 3;
        break;
    }
    isStandalone = EEPROM.read(3);
    state.ped_threshold = EEPROM.read(4);
  }
#if stateDebug
  Serial.println(F("EEPROM initialised!"));
#endif /* stateDebug */
}

bool writeEEP(void *params) {
  // only update if the value has changed
  switch (state.mode) {
    case MODE_STOCK:
      lastMode = 0;
      break;
    case MODE_FWD:
      lastMode = 1;
      break;
    case MODE_5050:
      lastMode = 2;
      break;
    case MODE_CUSTOM:
      lastMode = 3;
      break;
  }

  // update EEP only if changes have been made
  EEPROM.update(0, softwareVersion);
  EEPROM.update(1, isCustom);
  EEPROM.update(2, lastMode);
  EEPROM.update(3, isStandalone);
  EEPROM.update(4, state.ped_threshold);

#if stateDebug
  Serial.printf("Software Version: %d.%d%d%\n", (softwareVersion / 100) % 10, (softwareVersion / 10) % 10, (softwareVersion / 1) % 10);
  Serial.printf("Last Mode: %d\n", lastMode);
  Serial.printf("Is Standalone: %d\n", isStandalone);
  Serial.printf("Minimum Pedal: %d%\n", state.ped_threshold);
  Serial.printf("Haldex Engagement: %d%\n", haldexEngagement);
#endif /* stateDebug */

  return true;
}
