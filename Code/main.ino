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
  Serial.begin(baudBT);

#if stateDebug
  Serial.println(F("\nOpenHaldexTeensy 4.0 Initialisation"));
  Serial.printf("Running at %dMHz\r\n", F_CPU_ACTUAL / (1000 * 1000));
#endif /* stateDebug */

  Serial2.begin(baudBT);
#if stateDebug
  Serial.println(F("\nBluetooth initialised!"));
#endif /* stateDebug */

  // Turn off the power LED for power saving
  pinMode(LED_BUILTIN, OUTPUT);
  canInit();
  readEEP();

  // Setup the RGB LED pins for outputs
  pinMode(pinLED_R, OUTPUT);
  pinMode(pinLED_G, OUTPUT);
  pinMode(pinLED_B, OUTPUT);

  // Setup the switches (Switch Mode & Bluetooth) for interrupt / inputs (keeps response quick)
  // can't have BT_Conf as an interrupt and change the pin state...
  attachInterrupt(pinSwitchMode, checkSwitchMode, HIGH);
  pinMode(pinBT_Conf, INPUT);

  timer.every(3000, btSendStatus);  // send Bluetooth Status back to App/Screen if connected every 100ms
  timer.every(5000, writeEEP);      // write EEP (using 'update' to minimise writes) every 5s

#if stateDebug
  timer.every(1000, printMode);
#endif /* stateDebug */

#if canTestData
  timer.every(50, sendCanTest);
  timer.every(1000, printMB_Status);
#endif /* canTest */

  //timer.every(20, sendStandaloneCAN);
  /* Standalone Haldex Control */

  // Disable unwanted Teensy optionals
  CCM_ANALOG_PLL_AUDIO |= CCM_ANALOG_PLL_AUDIO_POWERDOWN;
  CCM_ANALOG_PLL_VIDEO |= CCM_ANALOG_PLL_VIDEO_POWERDOWN;
  CCM_ANALOG_PLL_ENET |= CCM_ANALOG_PLL_ENET_POWERDOWN;
}

void loop() {
  timer.tick();  // click the timer over

  if (millis() - lastTransmission >= btTimeout) {
    btConnected = false;
  } else {
    btConnected = true;
  }

  // check for Bluetooth Serial data, process if required
  while (Serial2.available()) {
    bt_packet rx_packet = { 0 };
    rx_packet.len = Serial2.readBytesUntil(SERIAL_PACKET_END, rx_packet.data, ARRAY_SIZE(rx_packet.data));
    btProcess(&rx_packet);
  }

  if (digitalRead(pinBT_Conf)) {
    btInit();  // can't have BT_Conf as an interrupt and change the pin state...
  }

  // light up the LED as per the 'state.mode'
  LED();
}
