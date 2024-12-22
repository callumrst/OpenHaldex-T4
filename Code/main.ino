/** Forbes Automotive - OpenHaldex T4 ***
Revised 24/10/2024 @ 14:45

V1.00 - Initial release to public
V1.02 - Adjusted locking code...
V1.05 - Added multi-generation options.  Adjustable in _defs.h
V1.06 - Gen4 confirmed working, released to public
*/

#include "openhaldex.h"

openhaldexState state;
byte haldexState;
byte haldexEngagement;
byte vehicleSpeed;
byte ped_threshold;
byte dummyVehicleSpeed;

int buttonToggle;
int softwareVersion = 106;
bool isCustom;
int lastMode;
bool isScreen = false;
bool isStandalone;
int i = 0;
uint32_t lastTransmission = 0;
bool btConnected = false;

auto timer = timer_create_default();  // for repeatable tasks

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

  timer.every(2500, btSendStatus);  // send Bluetooth Status back to App/Screen if connected every 1000ms
  timer.every(2500, writeEEP);      // write EEP (using 'update' to minimise writes) every 2500ms

#if stateDebug
  timer.every(1000, printMode);  // serial output for current mode
#endif

#if canTestData
  timer.every(50, sendCanTest);  // for printing test CAN messages
  timer.every(1000, printMB_Status);
#endif

#if broadcastOpenHaldex
  timer.every(50, castOpenHaldex);  // for printing CAN messages to FIS/CAN
#endif

  timer.every(20, sendStandaloneCAN);  // send standalone CAN messages every 20ms.  Don't capture in an 'if' as it may change during execution...
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
    rx_packet.len = Serial2.readBytesUntil(serialPacketEnd, rx_packet.data, arraySize(rx_packet.data));
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