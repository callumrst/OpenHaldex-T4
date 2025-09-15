#ifndef OPENHALDEX_EEPROM_H
#define OPENHALDEX_EEPROM_H

// EEPROM addresses
#define EEPROM_ADDRESS_VERSION_HIGH 0
#define EEPROM_ADDRESS_VERSION_LOW 1
#define EEPROM_ADDRESS_STANDALONE 2
#define EEPROM_ADDRESS_MODE 3
#define EEPROM_ADDRESS_PEDAL_THRESHOLD 4
#define EEPROM_ADDRESS_CUSTOM_AVAILABLE 5

// Library
#include <EEPROM.h>

// Functions

void init_EEPROM() {
  // If the version stored in EEPROM does not match the software version, reset the EEPROM to default values.
  if (EEPROM.read(EEPROM_ADDRESS_VERSION_HIGH) != ((SOFTWARE_VERSION >> 8) & 0xFF) || EEPROM.read(EEPROM_ADDRESS_VERSION_LOW) != (SOFTWARE_VERSION & 0xFF)) {
    DEBUG("Resetting EEPROM");

    // Start Bluetooth pairing.
    config_BT();

    // Reset each address to its default value.
    EEPROM.write(EEPROM_ADDRESS_VERSION_HIGH, ((SOFTWARE_VERSION >> 8) & 0xFF));
    EEPROM.write(EEPROM_ADDRESS_VERSION_LOW, (SOFTWARE_VERSION & 0xFF));
    EEPROM.write(EEPROM_ADDRESS_STANDALONE, false);
    EEPROM.write(EEPROM_ADDRESS_MODE, (uint8_t)MODE_STOCK);
    EEPROM.write(EEPROM_ADDRESS_PEDAL_THRESHOLD, state.pedal_threshold);
    EEPROM.write(EEPROM_ADDRESS_CUSTOM_AVAILABLE, false);
  }
  // If the version matches, read the values stored.
  else {
    // Get the "Standalone" preference.
    in_standalone_mode = EEPROM.read(EEPROM_ADDRESS_STANDALONE);

    // Get the "Mode" preference.
    uint8_t EEPROM_mode = EEPROM.read(EEPROM_ADDRESS_MODE);

    // If the requested mode is valid, apply it.
    if (EEPROM_mode < (uint8_t)openhaldex_mode_t_MAX) {
      state.mode = (openhaldex_mode_t)EEPROM_mode;
    }
    // For invalid values, fall back to STOCK mode.
    else {
      state.mode = MODE_STOCK;
    }

    // Enable mode_override for modes FWD, 5050, CUSTOM. (?)
    if (state.mode != MODE_STOCK) {
      state.mode_override = true;
    }

    // Get the "Pedal threshold" preference.
    state.pedal_threshold = EEPROM.read(EEPROM_ADDRESS_PEDAL_THRESHOLD);

    // Get the "Custom data available" flag.
    custom_mode_available = EEPROM.read(EEPROM_ADDRESS_CUSTOM_AVAILABLE);
  }
}

bool update_EEPROM(void *params) {
  // Update the values that can change at runtime.
  EEPROM.update(EEPROM_ADDRESS_STANDALONE, in_standalone_mode);
  EEPROM.update(EEPROM_ADDRESS_MODE, (uint8_t)state.mode);
  EEPROM.update(EEPROM_ADDRESS_PEDAL_THRESHOLD, state.pedal_threshold);
  EEPROM.update(EEPROM_ADDRESS_CUSTOM_AVAILABLE, custom_mode_available);

  DEBUG("Software: %d.%d, Standalone:%s, Mode:%s, PedalThreshold:%d, CustomModeAvailable:%s,BluetoothConnected:%s",
        ((SOFTWARE_VERSION >> 8) & 0xFF), (SOFTWARE_VERSION & 0xFF),
        in_standalone_mode ? "true" : "false",
        get_openhaldex_mode_string(state.mode),
        state.pedal_threshold,
        custom_mode_available ? "true" : "false",
        bluetooth_connected ? "true" : "false"
  );

  return true;
}

#endif
