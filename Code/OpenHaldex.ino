// Settings
#define HALDEX_GENERATION 1
#define SOFTWARE_VERSION 0x006C // 0.108
#define BROADCAST_OPENHALDEX // Comment out to disable

// Debug (comment out to disable)
#define ENABLE_DEBUG
#define DEBUG_HALDEXCAN_TRAFFIC
//#define DEBUG_CHASSISCAN_TRAFFIC

// Files
#include "openhaldex_definitions.h"
#include "openhaldex.h"
#include "openhaldex_gpio.h"
#include "openhaldex_bluetooth.h"
#include "openhaldex_can.h"
#include "openhaldex_calculations.h"
#include "openhaldex_eeprom.h"
#include "openhaldex_button.h"

// Timer used for repeating tasks
#include <arduino-timer.h>
auto timer = timer_create_default();

bool print_current_mode(void *params)
{
  DEBUG("Mode: %s", get_openhaldex_mode_string(state.mode));
  return true;
}

void setup()
{
#ifdef ENABLE_DEBUG
  Serial.begin(115200);
  DEBUG("OpenHaldex - Teensy 4.0 @ %dMHz", F_CPU_ACTUAL / (1000 * 1000));
#endif

  DEBUG("Init CAN");
  init_CAN();
  
  DEBUG("Init EEPROM");
  init_EEPROM();

  // Update the Bluetooth connection every 2.5 seconds.
  timer.every(2500, bt_send_status);

  // Update the values stored in EEPROM every 2.5 seconds.
  timer.every(2500, update_EEPROM);
  
#ifdef ENABLE_DEBUG
  // Display the current mode every second.
  timer.every(1000, print_current_mode);
#endif

#ifdef BROADCAST_OPENHALDEX
  // Broadcast OpenHaldex data every 50ms.
  timer.every(50, broadcast_openhaldex);
#endif

  // Emulate CAN frames for Standalone mode (if enabled).
  timer.every(20, send_standalone_CAN);

  DEBUG("Init GPIO");
  init_GPIO();

  DEBUG("Init Bluetooth");
  BT.begin(9600);
}

void loop()
{
  // Update the timer.
  timer.tick();

  // If too much time passed since the last Bluetooth message, a device is not connected.
  if (millis() - last_bt_transmission_ms >= BT_TIMEOUT_MS)
  {
    bluetooth_connected = false;
  }
  // Otherwise, a device is connected.
  else
  {
    bluetooth_connected = true;
  }

  // If there is Bluetooth data available, read and parse it.
  if (BT.available())
  {
    static bt_packet_t rx_packet;
    rx_packet.len = BT.readBytesUntil(BT_PACKET_END_BYTE, rx_packet.data, ARRAYSIZE(rx_packet.data));
    parse_bt_packet(rx_packet);
  }

  // If the Bluetooth Configuration button is pressed, start Bluetooth pairing.
  if (digitalRead(GPIO_BT_CONF_BUTTON))
  {
    config_BT();
  }

  // Light up the LED according to the current mode.
  show_current_mode_LED();
}
