/** Forbes Automotive - OpenHaldex T4 ***
Revised 24/10/2024 @ 14:45

V1.00 - Initial release to public
V1.02 - Adjusted locking code...
V1.05 - Added multi-generation options.  Adjustable in _defs.h
*/

#include "openhaldex.h"

openhaldexState state;
byte haldexState;
byte haldexEngagement;
byte vehicleSpeed;
byte ped_threshold;
byte dummyVehicleSpeed;

int buttonToggle;
int softwareVersion = 102;  // softwareVersion 1.02
bool isCustom;
int lastMode;
bool isScreen = false;
bool isStandalone;

int i = 0;

uint32_t lastTransmission = 0;
bool btConnected = false;

auto timer = timer_create_default();

bool printMode(void *params) {
  Serial.printf("OpenHaldex mode=%d\n", state.mode);
  return true;
}

void setup() {
#if stateDebug
  Serial.begin(baudSerial);
  Serial.println(F("\nOpenHaldex - Teensy 4.0 Initialisation..."));
  Serial.printf("Running at %dMHz\r\n", F_CPU_ACTUAL / (1000 * 1000));
#endif /* stateDebug */

#if stateDebug
  Serial.println(F("Bluetooth initialising..."));
#endif
  Serial2.begin(baudBT);
#if stateDebug
  Serial.println(F("Bluetooth initialised!"));
#endif

  setupPins();  // setup IO
  canInit();    // begin CAN
  readEEP();    // read EEPROM for saved preferences

  timer.every(1000, btSendStatus);  // send Bluetooth Status back to App/Screen if connected every 1000ms
  timer.every(2500, writeEEP);      // write EEP (using 'update' to minimise writes) every 2500ms

#if stateDebug
  timer.every(1000, printMode);  // serial output for current mode
#endif

#if canTestData
  timer.every(50, sendCanTest);  // for printing test CAN messages
  timer.every(1000, printMB_Status);
#endif

  timer.every(20, sendStandaloneCAN);  // send standalone CAN messages every 20ms
}

void loop() {
  timer.tick();  // click the timer over

  // check to see if the Bluetooth is connected to any devices
  if (millis() - lastTransmission >= btTimeout) {
    btConnected = false;
  } else {
    btConnected = true;
  }

  // check to see if any Bluetooth Serial data, process if required
  while (Serial2.available()) {
    bt_packet rx_packet = { 0 };
    rx_packet.len = Serial2.readBytesUntil(serialPacketEnd, rx_packet.data, ARRAY_SIZE(rx_packet.data));
    btProcess(&rx_packet);
  }

  if (digitalRead(pinBT_Conf)) {
    btInit();  // can't have BT_Conf as an interrupt and change the pin state...
  }

  // light up the LED as per the 'state.mode'
  LED();
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
  attachInterrupt(pinSwitchMode, checkSwitchMode, HIGH);
  pinMode(pinBT_Conf, INPUT);
}