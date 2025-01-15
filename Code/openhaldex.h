#include <Arduino.h>
#include <SPI.h>  // is this needed?
#include "openhaldex_defs.h"
#include <stdio.h>
#include <arduino-timer.h>
#include <FlexCAN_T4.h>
#include <EEPROM.h>

/* Defines */
// Debug statements
#define HaldexCANDebug 1       // if 1, will print CAN 1 (Haldex) messages
#define ChassisCANDebug 0      // if 1, will print CAN 2 (Chassis) messages
#define stateDebug 1           // if 1, will Serial print
#define broadcastOpenHaldex 1  // broadcast OpenHaldex data over CAN

// Serial / Serial2 specifics
#define baudSerial 115200  // Serial communication baud rate
#define baudBT 9600        // Bluetooth Baud Rate (default is 9600).  AT Setup is 38400
#define btTimeout 8000     // Bluetooth timeout before registering 'not connected'

// Pin IO
#define pinBT_Conf 4         // button for BT config
#define pinBT_Reset 5        // 'button' for BT reset
#define pinSwitchMode 17     // button for Switch Mode Change
#define pinLED_R 12          // RGB LED (Pin R)
#define pinLED_G 10          // RGB LED (Pin G)
#define pinLED_B 11          // RGB LED (Pin B)
#define pinBT_LED2 14        // Bluetooth LED2 (on HC-05 board)
#define pinBT_LED1 15        // Bluetooth LED1 (on HC-05 board)
#define pinSwitchModePCB 13  // external button for switching modes (for dash conversions etc)

#define arraySize(array) (sizeof((array)) / sizeof((array)[0]))
#define serialPacketEnd 0xFF

/* Globals */
extern openhaldexState state;
extern byte vehicleSpeed;
extern byte dummyVehicleSpeed;
extern byte haldexEngagement;
extern byte haldexState;
extern float lockTarget;
extern float pedValue;
extern int buttonToggle;
extern byte ped_threshold;
extern uint32_t lastTransmission;
extern bool btConnected;
extern int iCounter;

extern bool reportClutch1;
extern bool tempProtection;
extern bool reportClutch2;
extern bool couplingOpen;
extern bool speedLimit;

extern int bremes1Counter;
extern int temp;

/* EEPROM Data */
extern int softwareVersion;
extern bool isCustom;
extern int lastMode;
extern bool isScreen;
extern bool isStandalone;

/* Functions */
// Setup
extern void basicInit();
extern void setupPins();
extern void setupTimers();
extern bool printMode(void *params);

// EEPROM Related
extern void readEEP();
extern bool writeEEP(void *params);

// Bluetooth Related
extern void btInit(void);
extern bool btSendStatus(void *params);
extern void btGetStatus(bt_packet *rx_packet);

// CAN Related
extern void canInit(void);
extern void onHaldexRX(const CAN_message_t &frame);
extern void onBodyRX(const CAN_message_t &frame);
extern void getLockData(CAN_message_t *frame);

// CAN False Frames
extern float getLockTargetAdjustment(void);
extern uint8_t getLockTargetAdjustedValue(uint8_t value, bool invert);
extern bool sendCanTest(void *params);
extern bool sendStandaloneCAN(void *params);
extern bool castOpenHaldex(void *params);
extern void sendOpenFrame_Gen1();
extern void send5050Frame_Gen1();
extern void sendOpenFrame_Gen2();
extern void send5050Frame_Gen2();
extern void sendOpenFrame_Gen4();
extern void send5050Frame_Gen4();

// CAN Events & Mailbox
extern void serviceCanEvents(void);
extern bool printMB_Status(void *params);

// Buttons & LED
extern void checkSwitchMode();
extern void LED();
extern void blinkLED(int duration, int flashes, int R, int G, int B);