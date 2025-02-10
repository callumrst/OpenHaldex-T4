#ifndef OPENHALDEX_BUTTON_H
#define OPENHALDEX_BUTTON_H

// Settings
#define MODE_BUTTON_MAX_RATE_MS 300

// Functions

void mode_button_ISR() {
  // Only process the Mode button press if enough time has passed since the last press.
  static unsigned long last_mode_button_press_ms = 0;
  if (millis() - last_mode_button_press_ms < MODE_BUTTON_MAX_RATE_MS) {
    return;
  }
  last_mode_button_press_ms = millis();

  DEBUG("Mode button pressed");

  // Holding the Mode and Bluetooth Configuration buttons for ~2s switches Standalone mode.
  uint8_t conf_button_200ms_hold_periods = 0;
  while (digitalRead(GPIO_BT_CONF_BUTTON)) {
    if (conf_button_200ms_hold_periods == 0) {
      DEBUG("Hold for 2s to switch Standalone mode");
    }

    // Flashing the LED will cause a delay of 2*100=200ms.
    blinkLED(100, 1, 5, 0, 5);

    // After 2s, stop waiting.
    if (++conf_button_200ms_hold_periods >= 10) {
      break;
    }
  }

  // If the Bluetooth Configuration button was held for 2s, switch Standalone mode.
  if (conf_button_200ms_hold_periods >= 10) {
    in_standalone_mode = !in_standalone_mode;
    DEBUG("Switched; Standalone:%s", in_standalone_mode ? "true" : "false");

    // Confirm the switching of Standalone mode by flashing the LED.
    blinkLED(500, 11, 5, 0, 5);

    // Isn't 2*500*11 = 11s a bit long? Perhaps I don't understand Teensy interrupts,
    // but aren't you supposed to waste as little time in an ISR as possible?

    // Does this not freeze Bluetooth or CAN?
  }

  // If the Bluetooth connection is active, ignore Mode button pressed, just flash the LED.
  if (bluetooth_connected) {
    blinkLED(100, 11, 5, 5, 5);
    // Same question, even more pertinent here regarding the Bluetooth connection.
  }
  // Otherwise, cycle to the next mode.
  else {
    // mode_override is disabled when switching modes with the button.
    state.mode_override = false;
    // What does mode_override do exactly? :)

    // Determine the next mode in the sequence.
    uint8_t next_mode = (uint8_t)state.mode + 1;

    // If the next mode is valid, change the current mode to it.
    if (next_mode < (uint8_t)openhaldex_mode_t_MAX - 1) {
      state.mode = (openhaldex_mode_t)next_mode;
    }
    // On overflow, start over from MODE_STOCK.
    else {
      state.mode = MODE_STOCK;
    }

    // If in Standalone mode, jump over STOCK mode, as there is no chassis CAN data available.
    if (state.mode == MODE_STOCK && in_standalone_mode) {
      state.mode = MODE_FWD;
      // Is this the intended logic? The original code seems strange.
    }

    // Only allow CUSTOM mode in non-Standalone mode and after the Bluetooth app sent custom lockpoint data.
    if (state.mode == MODE_CUSTOM) {
      // If in Standalone mode, jump over CUSTOM mode, as there is no car data available.
      if (in_standalone_mode) {
        state.mode = MODE_FWD;
      }

      // If the Bluetooth app didn't provide custom lockpoint data, jump over CUSTOM mode.
      if (!custom_mode_available) {
        state.mode = MODE_STOCK;
      }
    }

    DEBUG("Goto mode: %s", get_openhaldex_mode_string(state.mode));
  }
}

#endif
