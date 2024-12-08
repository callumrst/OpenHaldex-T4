#include "openhaldex.h"
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> HaldexCAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> ChassisCAN;

void canInit(void) {
  // setup Haldex CAN module (using FlexCAN T4)
  HaldexCAN.begin();
  HaldexCAN.setClock(CLK_60MHz);
  HaldexCAN.setBaudRate(500000);
  HaldexCAN.setMaxMB(16);
  HaldexCAN.onReceive(onHaldexRX);
  HaldexCAN.enableFIFO();
  HaldexCAN.enableFIFOInterrupt();
  HaldexCAN.mailboxStatus();

  // setup Chassis CAN module (using FlexCAN T4)
  ChassisCAN.begin();
  ChassisCAN.setClock(CLK_60MHz);
  ChassisCAN.setBaudRate(500000);
  ChassisCAN.setMaxMB(16);
  ChassisCAN.onReceive(onBodyRX);
  ChassisCAN.enableFIFO();
  ChassisCAN.enableFIFOInterrupt();
  // filter specific frames?
  //ChassisCAN.setFIFOFilter(0, 0x123, STD);
  //ChassisCAN.setMBFilter(MB6, 0x123);  //
  ChassisCAN.mailboxStatus();
}

void serviceCanEvents(void) {
  HaldexCAN.events();
  ChassisCAN.events();
}

bool printMB_Status(void *params) {

#if HaldexCANDebug
  HaldexCAN.mailboxStatus();
#endif
#if ChassisCANDebug
  ChassisCAN.mailboxStatus();
#endif

  return true;
}

void onHaldexRX(const CAN_message_t &frame) {
  // got data from the Haldex controller...
#if HaldexCANDebug
  Serial.printf("[FROM HALDEX RX]ID: 0x%x DATA: ", frame.id);
  for (int i = 0; i < frame.len; i++) {
    Serial.printf("%02x ", frame.buf[i]);
  }
  Serial.println();
#endif

  haldexState = frame.buf[0];
  haldexEngagement = frame.buf[1];

#if !canTestData
  CAN_message_t frame_out;
  frame_out.id = frame.id;
  frame_out.flags = frame.flags;
  frame_out.len = frame.len;
  if (frame.len <= arraySize(frame_out.buf)) {
    memcpy(frame_out.buf, frame.buf, frame.len);
  }

  // From Haldex to car, just forward whatever we receive.
  if (!ChassisCAN.write(frame_out)) {
    // flash LED?
    Serial.println(F("Body CAN TX Fail!"));
    ChassisCAN.mailboxStatus();
  }
#endif
}

void onBodyRX(const CAN_message_t &frame) {
  // still having 'some' CAN signals will get through here, so check if it's standalone so they can be ignored - mainly for testing(!)
  if (!isStandalone) {
    CAN_message_t frame_out;        // build a frame for transmitting to Haldex...
    frame_out.id = frame.id;        // use the same ID that's came in
    frame_out.flags = frame.flags;  // use the same flags that's came in
    frame_out.len = frame.len;      // use the same length that's came in

    // if the data coming in is <8 bytes, pass it out - all from Motor/Bremes are 8 bytes...
    if (frame.len <= arraySize(frame_out.buf)) {
      memcpy(frame_out.buf, frame.buf, frame.len);
    }

#if ChassisCANDebug
    Serial.printf("[FROM BODY RX]ID: 0x%x DATA: ", frame.id);
    for (int i = 0; i < frame.len; i++) {
      Serial.println("");
      Serial.printf("%02x ", frame.buf[i]);
    }
#endif

    /* Anything which has come from the chassis.. i.e. everything except Haldex
       check to see if the frames come in one at a time or all at once.  Is there a possiblity that it's reading 'motor1_id' once, and by the time it's got through the other 
       frames another 'motor1_id' frame has got to the Haldex.  For loop to go through all of the frames at once?
    */
    switch (frame.id) {  // get real-time data as it comes in, process for Drivers Pedal % & vehicle speed
      case MOTOR1_ID:
        pedValue = frame.buf[5] * 0.4;  // byte 5 is 'wpedv_w'
        break;
      case MOTOR2_ID:
        {
          int calc_speed = (frame.buf[3] * 100 * 128) / 10000;  // byte 3 is 'vfzg'
          vehicleSpeed = (byte)(calc_speed >= 255 ? 255 : calc_speed);
          break;
        }
      case openHaldex_ID:
        switch (frame.buf[0]) {
          case 0:
            state.mode = MODE_STOCK;
            lastMode = 0;
            break;
          case 1:
            state.mode = MODE_FWD;
            lastMode = 1;
            break;
          case 2:
            state.mode = MODE_5050;
            lastMode = 2;
            break;
            /*add default?
                default:
                break;
        */
        }
    }

    if (state.mode == MODE_STOCK) {
      memcpy(frame_out.buf, frame.buf, frame.len);  // do no work; copy all 'frame' data to 'frame_out'
    }

    if (state.mode == MODE_FWD) {
      // If FWD mode then literally zero out everything going from chassis to haldex
      //sendOpenFrame_Gen1();
      //getLockData(&frame_out);
      if (haldexGen == 1) {
        memset(frame_out.buf, 0xFF, frame_out.len);  //0xFF seems to work best...
      }
      if (haldexGen == 4) {
        getLockData(&frame_out);  // parse data for 5050/custom
      }
    }

    if (state.mode == MODE_5050 || state.mode == MODE_CUSTOM) {
      if (haldexGen == 1 || haldexGen == 4) {
        getLockData(&frame_out);  // parse data for 5050/custom
      } else {
        switch (frame.id) {  // don't have data for Gen 2/3/5 yet, so just copy out frames if req.
          case MOTOR1_ID:
            memcpy(frame_out.buf, frame.buf, frame.len);  // do no work; copy all 'frame' data to 'frame_out'
            break;
          case MOTOR2_ID:
            memcpy(frame_out.buf, frame.buf, frame.len);  // do no work; copy all 'frame' data to 'frame_out'
            break;
          case MOTOR3_ID:
            memcpy(frame_out.buf, frame.buf, frame.len);  // do no work; copy all 'frame' data to 'frame_out'
            break;
          case MOTOR6_ID:
            memcpy(frame_out.buf, frame.buf, frame.len);  // do no work; copy all 'frame' data to 'frame_out'
            break;
          case BRAKES1_ID:
            memcpy(frame_out.buf, frame.buf, frame.len);  // do no work; copy all 'frame' data to 'frame_out'
            break;
          case BRAKES2_ID:
            memcpy(frame_out.buf, frame.buf, frame.len);  // do no work; copy all 'frame' data to 'frame_out'
            break;
          case BRAKES3_ID:
            memcpy(frame_out.buf, frame.buf, frame.len);  // do no work; copy all 'frame' data to 'frame_out'
            break;
        }
      }
    }

#if !canTestData
    if (!HaldexCAN.write(frame_out)) {                 // write CAN frame from the body to the Haldex
      Serial.println(F("Haldex CAN Write TX Fail!"));  // if writing is unsuccessful, there is something wrong with the Haldex(!) Possibly flash red LED?
      HaldexCAN.mailboxStatus();                       // print the mailbox status if there's a failure
    }
#endif
  }
}

bool castOpenHaldex(void *params) {
  CAN_message_t broadcastCAN;  //0x7C0
  broadcastCAN.id = fisCuntrol_ID;
  broadcastCAN.len = 8;
  broadcastCAN.buf[0] = APP_MSG_STATUS;       //
  broadcastCAN.buf[1] = isStandalone;         //
  broadcastCAN.buf[2] = haldexEngagement;     //
  broadcastCAN.buf[3] = int(lockTarget);      //
  broadcastCAN.buf[4] = vehicleSpeed;         //
  broadcastCAN.buf[5] = state.mode_override;  //
  broadcastCAN.buf[6] = state.mode;           //
  broadcastCAN.buf[7] = int(pedValue);        //

  if (!ChassisCAN.write(broadcastCAN)) {              // write CAN frame from the body to the Haldex
    Serial.println(F("Chassis CAN Write TX Fail!"));  // if writing is unsuccessful, there is something wrong with the Haldex(!) Possibly flash red LED?
    ChassisCAN.mailboxStatus();                       // print the mailbox status if there's a failure
    return false;
  }
  return true;
}

bool sendStandaloneCAN(void *params) {
  // if isStandalone, will send FWD or 5050 signal to Haldex to 'force' every 20ms (using Timer)
  if (isStandalone) {
    if (state.mode == MODE_FWD) {
      if (haldexGen == 1) {
        sendOpenFrame_Gen1();
      }
      if (haldexGen == 2) {
        sendOpenFrame_Gen2();
      }
      if (haldexGen == 4) {
        sendOpenFrame_Gen4();
      }
    }

    if (state.mode == MODE_5050) {
      if (haldexGen == 1) {
        send5050Frame_Gen1();
      }
      if (haldexGen == 2) {
        send5050Frame_Gen2();
      }
      if (haldexGen == 4) {
        send5050Frame_Gen4();
      }
    }
  }
  return true;
}

void sendOpenFrame_Gen1() {
  // write an empty frame to every ID, will stop all comms to the Haldex pump.  All data should be '0xFF' to null any work
  CAN_message_t motor1;  //280
  motor1.id = MOTOR1_ID;
  motor1.len = 8;
  motor1.buf[0] = 0xFF;  // inner engine moment (%): 0.39*(0xF0) = 93.6%  (make FE?) (was 0xf0)
  motor1.buf[1] = 0xFF;  // was C3
  motor1.buf[2] = 0xFF;  // motor speed (rpm): 32 >
  motor1.buf[3] = 0xFF;  // motor speed (rpm): 78 > 0.25 * 3278 = 819.5 RPM (was 0x4e)  Leave RPM the same?
  motor1.buf[4] = 0xFF;  // was 22
  motor1.buf[5] = 0xFF;  // driving pedal (%): 0.39*(0xF0) = 93.6%  (make FE?)
  motor1.buf[6] = 0xFF;  // was 22
  motor1.buf[7] = 0xFF;  // was 22
  HaldexCAN.write(motor1);

  CAN_message_t motor3;  //380
  motor3.id = MOTOR3_ID;
  motor3.len = 8;
  motor3.buf[0] = 0xFF;  // inner engine moment (%): 0.39*(0xF0) = 93.6%  (make FE?) (was 0xf0)
  motor3.buf[1] = 0xFF;  // was 5E
  motor3.buf[2] = 0xFF;  // motor speed (rpm): 32 >
  motor3.buf[3] = 0xFF;  // motor speed (rpm): 78 > 0.25 * 3278 = 819.5 RPM (was 0x4e)  Leave RPM the same?
  motor3.buf[4] = 0xFF;  // was A0
  motor3.buf[5] = 0xFF;  // was A0
  motor3.buf[6] = 0xFF;  // torque loss (%): 0.39*(0x20) = 12.48%? (make FE?) slippage?
  motor3.buf[7] = 0xFF;  // was 04
  HaldexCAN.write(motor3);

  CAN_message_t brakes1;
  brakes1.id = BRAKES1_ID;
  brakes1.len = 1;
  brakes1.buf[0] = 0xFF;  // asr req
  HaldexCAN.write(brakes1);

  CAN_message_t brakes3;
  brakes3.id = BRAKES3_ID;
  brakes3.len = 8;
  brakes3.buf[0] = 0xFF;  // low byte, LEFT Front
  brakes3.buf[1] = 0xFF;  // high byte, LEFT Front
  brakes3.buf[2] = 0xFF;  // low byte, RIGHT Front
  brakes3.buf[3] = 0xFF;  // high byte, RIGHT Front
  brakes3.buf[4] = 0xFF;  // low byte, LEFT Rear
  brakes3.buf[5] = 0xFF;  // high byte, LEFT Rear // 254+10? (5050 returns 0xA)
  brakes3.buf[6] = 0xFF;  // low byte, RIGHT Rear
  brakes3.buf[7] = 0xFF;  // low byte, RIGHT Rear  // 254+10?
  HaldexCAN.write(brakes3);
}

void send5050Frame_Gen1() {
  CAN_message_t motor1;
  motor1.id = MOTOR1_ID;
  motor1.len = 8;
  motor1.buf[0] = 0x00;  // various individual bits ('space gas', driving pedal, kick down, clutch, timeout brake, brake intervention, drinks-torque intervention?) was 0x01
  motor1.buf[1] = 0xFA;  // inner engine moment (%): 0.39*(0xF0 / 250) = 97.5% (93.6% <> 0xf0)
  motor1.buf[2] = 0x20;  // motor speed (rpm): 32 > (low byte)
  motor1.buf[3] = 0x4E;  // motor speed (rpm): 78 > (high byte) : 0.25 * (32 78) = 819.5 RPM (was 0x4e)  Leave RPM the same?
  motor1.buf[4] = 0xFE;  // inner moment (%): 0.39*(0xF0) = 93.6%  (make FE?)
  motor1.buf[5] = 0xFE;  // driving pedal (%): 0.39*(0xF0) = 93.6%  (make FE?)
  motor1.buf[6] = 0x20;  // torque loss (%): 0.39*(0x20) = 12.48%? (make FE?) slippage?
  motor1.buf[7] = 0xFE;  // drivers moment (%): 0.39*(0xF0) = 93.6%  (make FE?)
  HaldexCAN.write(motor1);

  CAN_message_t motor3;
  motor3.id = MOTOR3_ID;
  motor3.len = 8;
  motor3.buf[0] = 0x00;  // various individual bits ('motor has been launched, only in diesel')
  motor3.buf[1] = 0x50;  // outdoor temperature
  motor3.buf[2] = 0xFA;  // pedal
  motor3.buf[3] = 0x3E;  // wheel command torque (low byte).  If SY_ASG
  motor3.buf[4] = 0xA0;  // wheel command torque (high byte).  If SY_ASG
  motor3.buf[5] = 0x00;  // wheel command torque (0 = positive, 1 = negative).  If SY_ASG
  motor3.buf[6] = 0x00;  // req. torque.  If SY_ASG
  motor3.buf[7] = 0xFE;  // throttle angle (100%)
  HaldexCAN.write(motor3);

  CAN_message_t brakes1;
  brakes1.id = BRAKES1_ID;
  brakes1.len = 8;
  brakes1.buf[0] = 0x80;  // asr req
  brakes1.buf[1] = 0x41;
  brakes1.buf[2] = 0x00;
  brakes1.buf[3] = 0xF4;
  brakes1.buf[4] = 0xFE;
  brakes1.buf[5] = 0xFE;
  brakes1.buf[6] = 0x00;
  brakes1.buf[7] = 0x1E;
  HaldexCAN.write(brakes1);

  CAN_message_t brakes3;
  brakes3.id = BRAKES3_ID;
  brakes3.len = 8;
  brakes3.buf[0] = 0xFF;  // low byte, LEFT Front
  brakes3.buf[1] = 0xA;   // high byte, LEFT Front
  brakes3.buf[2] = 0xFF;  // low byte, RIGHT Front
  brakes3.buf[3] = 0xA;   // high byte, RIGHT Front
  brakes3.buf[4] = 0x0;   // low byte, LEFT Rear
  brakes3.buf[5] = 0xA;   // high byte, LEFT Rear // 254+10? (5050 returns 0xA)
  brakes3.buf[6] = 0x0;   // low byte, RIGHT Rear
  brakes3.buf[7] = 0xA;   // low byte, RIGHT Rear  // 254+10?
  HaldexCAN.write(brakes3);
}

void sendOpenFrame_Gen2() {
  // write an empty frame to every ID, will stop all comms to the Haldex pump.  All data should be '0xFF' to null any work
  CAN_message_t motor1;  //280
  motor1.id = MOTOR1_ID;
  motor1.len = 8;
  motor1.buf[0] = 0xFF;  // inner engine moment (%): 0.39*(0xF0) = 93.6%  (make FE?) (was 0xf0)
  motor1.buf[1] = 0xFF;  // was C3
  motor1.buf[2] = 0xFF;  // motor speed (rpm): 32 >
  motor1.buf[3] = 0xFF;  // motor speed (rpm): 78 > 0.25 * 3278 = 819.5 RPM (was 0x4e)  Leave RPM the same?
  motor1.buf[4] = 0xFF;  // was 22
  motor1.buf[5] = 0xFF;  // driving pedal (%): 0.39*(0xF0) = 93.6%  (make FE?)
  motor1.buf[6] = 0xFF;  // was 22
  motor1.buf[7] = 0xFF;  // was 22
  HaldexCAN.write(motor1);

  CAN_message_t motor3;  //380
  motor3.id = MOTOR3_ID;
  motor3.len = 8;
  motor3.buf[0] = 0xFF;  // inner engine moment (%): 0.39*(0xF0) = 93.6%  (make FE?) (was 0xf0)
  motor3.buf[1] = 0xFF;  // was 5E
  motor3.buf[2] = 0xFF;  // motor speed (rpm): 32 >
  motor3.buf[3] = 0xFF;  // motor speed (rpm): 78 > 0.25 * 3278 = 819.5 RPM (was 0x4e)  Leave RPM the same?
  motor3.buf[4] = 0xFF;  // was A0
  motor3.buf[5] = 0xFF;  // was A0
  motor3.buf[6] = 0xFF;  // torque loss (%): 0.39*(0x20) = 12.48%? (make FE?) slippage?
  motor3.buf[7] = 0xFF;  // was 04
  HaldexCAN.write(motor3);

  CAN_message_t brakes1;
  brakes1.id = BRAKES1_ID;
  brakes1.len = 1;
  brakes1.buf[0] = 0xFF;  // asr req
  HaldexCAN.write(brakes1);

  CAN_message_t brakes3;
  brakes3.id = BRAKES3_ID;
  brakes3.len = 8;
  brakes3.buf[0] = 0xFF;  // low byte, LEFT Front
  brakes3.buf[1] = 0xFF;  // high byte, LEFT Front
  brakes3.buf[2] = 0xFF;  // low byte, RIGHT Front
  brakes3.buf[3] = 0xFF;  // high byte, RIGHT Front
  brakes3.buf[4] = 0xFF;  // low byte, LEFT Rear
  brakes3.buf[5] = 0xFF;  // high byte, LEFT Rear // 254+10? (5050 returns 0xA)
  brakes3.buf[6] = 0xFF;  // low byte, RIGHT Rear
  brakes3.buf[7] = 0xFF;  // low byte, RIGHT Rear  // 254+10?
  HaldexCAN.write(brakes3);
}

void send5050Frame_Gen2() {
  CAN_message_t motor1;
  motor1.id = MOTOR1_ID;
  motor1.len = 8;
  motor1.buf[0] = 0x00;  // various individual bits ('space gas', driving pedal, kick down, clutch, timeout brake, brake intervention, drinks-torque intervention?) was 0x01
  motor1.buf[1] = 0xFA;  // inner engine moment (%): 0.39*(0xF0 / 250) = 97.5% (93.6% <> 0xf0)
  motor1.buf[2] = 0x20;  // motor speed (rpm): 32 > (low byte)
  motor1.buf[3] = 0x4E;  // motor speed (rpm): 78 > (high byte) : 0.25 * (32 78) = 819.5 RPM (was 0x4e)  Leave RPM the same?
  motor1.buf[4] = 0xFE;  // inner moment (%): 0.39*(0xF0) = 93.6%  (make FE?)
  motor1.buf[5] = 0xFE;  // driving pedal (%): 0.39*(0xF0) = 93.6%  (make FE?)
  motor1.buf[6] = 0x20;  // torque loss (%): 0.39*(0x20) = 12.48%? (make FE?) slippage?
  motor1.buf[7] = 0xFE;  // drivers moment (%): 0.39*(0xF0) = 93.6%  (make FE?)
  HaldexCAN.write(motor1);

  CAN_message_t motor3;
  motor3.id = MOTOR3_ID;
  motor3.len = 8;
  motor3.buf[0] = 0x00;  // various individual bits ('motor has been launched, only in diesel')
  motor3.buf[1] = 0x50;  // outdoor temperature
  motor3.buf[2] = 0xFA;  // pedal
  motor3.buf[3] = 0x3E;  // wheel command torque (low byte).  If SY_ASG
  motor3.buf[4] = 0xA0;  // wheel command torque (high byte).  If SY_ASG
  motor3.buf[5] = 0x00;  // wheel command torque (0 = positive, 1 = negative).  If SY_ASG
  motor3.buf[6] = 0x00;  // req. torque.  If SY_ASG 'nmax_w'
  motor3.buf[7] = 0xFE;  // throttle angle (100%)
  HaldexCAN.write(motor3);

  CAN_message_t brakes1;
  brakes1.id = BRAKES1_ID;
  brakes1.len = 8;
  brakes1.buf[0] = 0x80;  // asr req
  brakes1.buf[1] = 0x41;  // abs lamp etc
  brakes1.buf[2] = 0x00;  // Geschwindigkeit low (bits 1-7).  Bit 0 aktiver BKV
  brakes1.buf[3] = 0xF4;  // Geschwindigk. high
  brakes1.buf[4] = 0xFE;  // asr 0.39*hex 'miasrl_can'
  brakes1.buf[5] = 0xFE;  // 'miasrs_can'
  brakes1.buf[6] = 0x00;  // 'mimsr_can'
  brakes1.buf[7] = 0x1E;  // esp
  HaldexCAN.write(brakes1);

  CAN_message_t brakes3;
  brakes3.id = BRAKES3_ID;
  brakes3.len = 8;
  brakes3.buf[0] = 0xFF;  // low byte, LEFT Front 'Fahrtrichtung VL'.  ***Bits 1-7 low byte.  Bit 0 is Vorwarts:0 OR Ruckwarts:1***
  brakes3.buf[1] = 0xA;   // high byte, LEFT Front 'Radgeschw. VL' ALL OF BYTE 2
  brakes3.buf[2] = 0xFF;  // low byte, RIGHT Front 'Fahrtrichtung VR' ***Bits 1-7 low byte.  Bit 0 is Vorwarts:0 OR Ruckwarts:1***
  brakes3.buf[3] = 0xA;   // high byte, RIGHT Front 'Radgeschw. VL'
  brakes3.buf[4] = 0x0;   // low byte, LEFT Rear 'Fahrtrichtung HL' ***Bits 1-7 low byte.  Bit 0 is Vorwarts:0 OR Ruckwarts:1***
  brakes3.buf[5] = 0xA;   // high byte, LEFT Rear // 254+10? (5050 returns 0xA) 'Radgeschw. HL'
  brakes3.buf[6] = 0x0;   // low byte, RIGHT Rear 'Fahrtrichtung HR' ***Bits 1-7 low byte.  Bit 0 is Vorwarts:0 OR Ruckwarts:1***
  brakes3.buf[7] = 0xA;   // low byte, RIGHT Rear  // 254+10? 'Radgeschw. HR'
  HaldexCAN.write(brakes3);
}

void sendOpenFrame_Gen4() {
  // write an empty frame to every ID, will stop all comms to the Haldex pump.  All data should be '0xFF' to null any work
  CAN_message_t motor1;  //280
  motor1.id = MOTOR1_ID;
  motor1.len = 8;
  motor1.buf[0] = 0xFF;  // inner engine moment (%): 0.39*(0xF0) = 93.6%  (make FE?) (was 0xf0)
  motor1.buf[1] = 0xFF;  // was C3
  motor1.buf[2] = 0xFF;  // motor speed (rpm): 32 >
  motor1.buf[3] = 0xFF;  // motor speed (rpm): 78 > 0.25 * 3278 = 819.5 RPM (was 0x4e)  Leave RPM the same?
  motor1.buf[4] = 0xFF;  // was 22
  motor1.buf[5] = 0xFF;  // driving pedal (%): 0.39*(0xF0) = 93.6%  (make FE?)
  motor1.buf[6] = 0xFF;  // was 22
  motor1.buf[7] = 0xFF;  // was 22
  HaldexCAN.write(motor1);

  CAN_message_t motor3;  //380
  motor3.id = MOTOR3_ID;
  motor3.len = 8;
  motor3.buf[0] = 0xFF;  // inner engine moment (%): 0.39*(0xF0) = 93.6%  (make FE?) (was 0xf0)
  motor3.buf[1] = 0xFF;  // was 5E
  motor3.buf[2] = 0xFF;  // motor speed (rpm): 32 >
  motor3.buf[3] = 0xFF;  // motor speed (rpm): 78 > 0.25 * 3278 = 819.5 RPM (was 0x4e)  Leave RPM the same?
  motor3.buf[4] = 0xFF;  // was A0
  motor3.buf[5] = 0xFF;  // was A0
  motor3.buf[6] = 0xFF;  // torque loss (%): 0.39*(0x20) = 12.48%? (make FE?) slippage?
  motor3.buf[7] = 0xFF;  // was 04
  HaldexCAN.write(motor3);

  CAN_message_t brakes1;
  brakes1.id = BRAKES1_ID;
  brakes1.len = 1;
  brakes1.buf[0] = 0xFF;  // asr req
  HaldexCAN.write(brakes1);

  CAN_message_t brakes3;
  brakes3.id = BRAKES3_ID;
  brakes3.len = 8;
  brakes3.buf[0] = 0xFF;  // low byte, LEFT Front
  brakes3.buf[1] = 0xFF;  // high byte, LEFT Front
  brakes3.buf[2] = 0xFF;  // low byte, RIGHT Front
  brakes3.buf[3] = 0xFF;  // high byte, RIGHT Front
  brakes3.buf[4] = 0xFF;  // low byte, LEFT Rear
  brakes3.buf[5] = 0xFF;  // high byte, LEFT Rear // 254+10? (5050 returns 0xA)
  brakes3.buf[6] = 0xFF;  // low byte, RIGHT Rear
  brakes3.buf[7] = 0xFF;  // low byte, RIGHT Rear  // 254+10?
  HaldexCAN.write(brakes3);
}

void send5050Frame_Gen4() {
  CAN_message_t motor1;
  motor1.id = MOTOR1_ID;
  motor1.len = 8;
  motor1.buf[0] = 0x00;  // various individual bits ('space gas', driving pedal, kick down, clutch, timeout brake, brake intervention, drinks-torque intervention?) was 0x01
  motor1.buf[1] = 0xFA;  // inner engine moment (%): 0.39*(0xF0 / 250) = 97.5% (93.6% <> 0xf0)
  motor1.buf[2] = 0x20;  // motor speed (rpm): 32 > (low byte)
  motor1.buf[3] = 0x4E;  // motor speed (rpm): 78 > (high byte) : 0.25 * (32 78) = 819.5 RPM (was 0x4e)  Leave RPM the same?
  motor1.buf[4] = 0xFE;  // inner moment (%): 0.39*(0xF0) = 93.6%  (make FE?)
  motor1.buf[5] = 0xFE;  // driving pedal (%): 0.39*(0xF0) = 93.6%  (make FE?)
  motor1.buf[6] = 0x20;  // torque loss (%): 0.39*(0x20) = 12.48%? (make FE?) slippage?
  motor1.buf[7] = 0xFE;  // drivers moment (%): 0.39*(0xF0) = 93.6%  (make FE?)
  HaldexCAN.write(motor1);

  CAN_message_t motor3;
  motor3.id = MOTOR3_ID;
  motor3.len = 8;
  motor3.buf[0] = 0x00;  // various individual bits ('motor has been launched, only in diesel')
  motor3.buf[1] = 0x50;  // outdoor temperature
  motor3.buf[2] = 0xFA;  // pedal
  motor3.buf[3] = 0x3E;  // wheel command torque (low byte).  If SY_ASG
  motor3.buf[4] = 0xA0;  // wheel command torque (high byte).  If SY_ASG
  motor3.buf[5] = 0x00;  // wheel command torque (0 = positive, 1 = negative).  If SY_ASG
  motor3.buf[6] = 0x00;  // req. torque.  If SY_ASG
  motor3.buf[7] = 0xFE;  // throttle angle (100%)
  HaldexCAN.write(motor3);

  CAN_message_t brakes1;
  brakes1.id = BRAKES1_ID;
  brakes1.len = 8;
  brakes1.buf[0] = 0x80;  // asr req
  brakes1.buf[1] = 0x41;
  brakes1.buf[2] = 0x00;
  brakes1.buf[3] = 0xF4;
  brakes1.buf[4] = 0xFE;
  brakes1.buf[5] = 0xFE;
  brakes1.buf[6] = 0x00;
  brakes1.buf[7] = 0x1E;
  HaldexCAN.write(brakes1);

  CAN_message_t brakes3;
  brakes3.id = BRAKES3_ID;
  brakes3.len = 8;
  brakes3.buf[0] = 0xFF;  // low byte, LEFT Front
  brakes3.buf[1] = 0xA;   // high byte, LEFT Front
  brakes3.buf[2] = 0xFF;  // low byte, RIGHT Front
  brakes3.buf[3] = 0xA;   // high byte, RIGHT Front
  brakes3.buf[4] = 0x0;   // low byte, LEFT Rear
  brakes3.buf[5] = 0xA;   // high byte, LEFT Rear // 254+10? (5050 returns 0xA)
  brakes3.buf[6] = 0x0;   // low byte, RIGHT Rear
  brakes3.buf[7] = 0xA;   // low byte, RIGHT Rear  // 254+10?
  HaldexCAN.write(brakes3);
}

// if canTestData, will send a 'false' message to the Chassis & Haldex
#if canTestData
//byte dummyVehicleSpeed = 0x00;
static byte dummy_haldexEngagement = 0x00;
bool sendCanTest(void *params) {
  CAN_message_t frame;

  frame.id = MOTOR1_ID;
  frame.len = 8;
  frame.buf[0] = 0;
  frame.buf[1] = 0xf0;
  frame.buf[2] = 0x20;
  frame.buf[3] = 0x4e;
  frame.buf[4] = 0xf0;
  frame.buf[5] = 0xf0;
  frame.buf[6] = 0x20;
  frame.buf[7] = state.mode;
  HaldexCAN.write(frame);

  CAN_message_t motor2;
  motor2.id = MOTOR2_ID;
  motor2.len = 4;
  motor2.buf[3] = dummyVehicleSpeed++;
  HaldexCAN.write(motor2);

  frame.id = MOTOR3_ID;
  frame.len = 1;
  frame.buf[0] = 0xff;
  HaldexCAN.write(frame);

  frame.id = MOTOR5_ID;
  frame.len = 1;
  frame.buf[0] = 0xff;
  HaldexCAN.write(frame);

  frame.id = MOTOR6_ID;
  frame.len = 1;
  frame.buf[0] = 0xff;
  HaldexCAN.write(frame);

  frame.id = MOTOR7_ID;
  frame.len = 1;
  frame.buf[0] = 0xff;
  HaldexCAN.write(frame);

  frame.id = MOTOR_FLEX_ID;
  frame.len = 1;
  frame.buf[0] = 0xff;
  HaldexCAN.write(frame);

  frame.id = BRAKES1_ID;
  frame.len = 1;
  frame.buf[0] = 0xff;
  HaldexCAN.write(frame);

  frame.id = BRAKES2_ID;
  frame.len = 1;
  frame.buf[0] = 0xff;
  HaldexCAN.write(frame);

  frame.id = BRAKES3_ID;
  frame.len = 1;
  frame.buf[0] = 0xff;
  HaldexCAN.write(frame);

  frame.id = BRAKES5_ID;
  frame.len = 1;
  frame.buf[0] = 0xff;
  HaldexCAN.write(frame);

  CAN_message_t haldex;
  haldex.id = HALDEX_ID;
  haldex.len = 2;
  haldex.buf[0] = 0x00;
  haldex.buf[1] = dummy_haldexEngagement++;  // Kupplungssteifigk
  //haldex.buf[2] = PNG Status (not sent?)
  ChassisCAN.write(haldex);

  return true;
}
#endif