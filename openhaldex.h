#ifndef OPENHALDEX_H
#define OPENHALDEX_H

// Values received from Haldex CAN
uint8_t received_haldex_state;
uint8_t received_haldex_engagement;
bool received_report_clutch1;
bool received_report_clutch2;
bool received_temp_protection;
bool received_coupling_open;
bool received_speed_limit;

// Values received from Chassis CAN
float received_pedal_value;
uint8_t received_vehicle_speed;

// for running through vars to see effects
uint8_t tempCounter;
uint8_t tempCounter1;
uint16_t tempCounter2;

// checksum values (for calculating module checksums in standalone mode)
uint8_t MOTOR5_counter = 0;

uint8_t BRAKES1_counter = 0;
uint8_t BRAKES2_counter = 3;  // starting counter for Brakes2 is 3
uint8_t BRAKES4_counter = 0;

uint8_t BRAKES5_counter = 0;
uint8_t BRAKES5_counter2 = 3;  // starting counter for Brake5 is 3

uint8_t BRAKES9_counter = 11;  // starting counter for Brakes9 is 11
uint8_t BRAKES9_counter2 = 0;

uint8_t BRAKES10_counter = 0;

uint8_t mLW_1_counter = 0;
uint8_t mLW_1_crc = 0;

// Internal variables
openhaldex_state_t state;
bool in_standalone_mode = false;
float lock_target = 0;

#endif
