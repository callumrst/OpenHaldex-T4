/** Forbes Automotive - OpenHaldex T4 ***
Revised 11/01/2025

OpenHaldex allows the parsing of CAN messages from Gen1 & Gen4 platforms and edits before sending to the Haldex Controller.  It can emulate ECU & ABS signals - so can operate entirely in standalone mode.
Gen2, 3 & 5 needs support.

ALL CAN messages are parsed directly from the library, so there's very little to do in 'loop'

V1.00 - Initial release to public
V1.02 - Adjusted locking code...
V1.05 - Added multi-generation options.  Adjustable in _defs.h
V1.06 - Gen4 confirmed working, released to public
V1.07 - Further understanding of frames - now added a '_notes.h' section to document findings
*/

#include "openhaldex.h"
auto timer = timer_create_default();

openhaldexState state;
byte haldexState;
byte haldexEngagement;
byte vehicleSpeed;
byte ped_threshold;
byte dummyVehicleSpeed;

int buttonToggle;
int lastMode;

int i = 0;
int bremes1Counter = 0;
int temp = 0;
int softwareVersion = 108;

uint32_t lastTransmission = 0;

bool isCustom;
bool isScreen = false;
bool isStandalone;
bool btConnected = false;
bool reportClutch1 = false;
bool tempProtection = false;
bool reportClutch2 = false;
bool couplingOpen = false;
bool speedLimit = false;

void setup() {
  // initialise the module, put into a separate function because of all of the debug statements, keeps this cleaner
  basicInit(); // in '_io/
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
    btGetStatus(&rx_packet);
  }

  if (digitalRead(pinBT_Conf)) {
    btInit();  // can't have BT_Conf as an interrupt and change the pin state...
  }

  // light up the LED as per the 'state.mode'
  LED(); // in '_io'
}

void setupTimers(){
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
  timer.every(50, castOpenHaldex);  // for parsing CAN messages to FIS/Ignitron etc
#endif

  timer.every(20, sendStandaloneCAN);  // send standalone CAN messages every 20ms.  Don't capture in an 'if' as it may change during execution...
}