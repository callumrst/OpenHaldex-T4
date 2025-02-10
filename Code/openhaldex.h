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

// Internal variables
openhaldex_state_t state;
bool in_standalone_mode = false;
float lock_target = 0;

#endif
