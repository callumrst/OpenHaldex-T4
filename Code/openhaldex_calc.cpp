#include "openhaldex.h"

float lockTarget = 0;
float pedValue = 0;

float getLockTargetAdjustment(void) {
  float target = 0;

  if (state.mode == MODE_5050) {
    if (pedValue >= state.ped_threshold || state.ped_threshold == 0 || state.mode_override) {
      return 100;
    } else {
      return 0;
    }
  } else if (state.mode == MODE_FWD) {
    return 0;
  }

  /* Find out which lockpoints we're between in terms of speed..
     * Look for the lockpoint above our current speed (lp_upper) */
  lockpoint lp_lower = state.custom_mode.lockpoints[0];
  lockpoint lp_upper = state.custom_mode.lockpoints[state.custom_mode.lockpoint_count - 1];

  for (int i = 0; i < state.custom_mode.lockpoint_count; i++) {
    if (vehicleSpeed <= state.custom_mode.lockpoints[i].speed) {
      lp_upper = state.custom_mode.lockpoints[i];
      lp_lower = state.custom_mode.lockpoints[(i == 0) ? 0 : i - 1];
      break;
    }
  }

  /* If locking at all... */
  if (pedValue >= state.ped_threshold || state.ped_threshold == 0 || state.mode_override) {
    /* Get the easy cases out the way first... */
    if (vehicleSpeed <= lp_lower.speed) {
      return lp_lower.lock;
    }
    if (vehicleSpeed >= lp_upper.speed) {
      return lp_upper.lock;
    }

    /* Need to interpolate */
    float inter = (float)(lp_upper.speed - lp_lower.speed) / (float)(vehicleSpeed - lp_lower.speed);

    target = lp_lower.lock + ((float)(lp_upper.lock - lp_lower.lock) / inter);
#if stateDebug
    Serial.printf("lp_upper:%d@%d lp_lower:%d@%d speed:%d(0x%x) target=%0.2f\n",
                  lp_upper.lock, lp_upper.speed, lp_lower.lock, lp_lower.speed, vehicleSpeed, dummyVehicleSpeed, target);
#endif
  }
  /* Else leave target at its initial value of 0. */

  return target;
}

uint8_t getLockTargetAdjustedValue(uint8_t value, bool invert) {
  if (state.mode == MODE_5050) {
    if (pedValue >= state.ped_threshold || state.ped_threshold == 0 || pedValue >= 10) {
      if (invert) {
        return (255 - value);
      } else {
        return value;
      }
    } else {
      if (invert) {
        return (255 - 0);
      } else {
        return 0;
      }
    }
  } else {
    // Potentially avoid doing math below..
    if (lockTarget == 0) {
      return 0;
    }

    /* Hackery to get the response closer to the target... we are trying to control the
        Haldex as if it's linear.. but it's not. In future, I'd like to implement some sort
        of feedback loop to trim the calculation being made here but this will do for now.  */
    float target_fudge_factor = lockTarget;
    target_fudge_factor = (target_fudge_factor / 2) + 20;
    if (invert) {
      return 255 - (value * (target_fudge_factor / 100));
    } else {
      return value * (target_fudge_factor / 100);
    }
  }
}

void getLockData(CAN_message_t *frame) {
  lockTarget = getLockTargetAdjustment();  // use to get initial lock target.  Will return 100% if 5050, 0 if FWD or xx if custom
  // from the above, to into adjusted value for each of the frame bytes and ONLY use IF !5050 or !FWD
  // so if 0xFA and 'custom', then get a fraction of 0xFA

  if (haldexGen == 1) {
    switch (frame->id) {
      case MOTOR1_ID:
        //frame->buf[0] = 0;
        frame->buf[1] = getLockTargetAdjustedValue(0xFE, false);  // engine moment
        frame->buf[2] = 0x21;  // low byte rpm
        frame->buf[3] = getLockTargetAdjustedValue(0x4E, false);  // high byte rpm - runs pre-charge pump if >0
        //frame->buf[4] = 0x00;  // inner moment
        //frame->buf[5] = 0x00;  // pedal
        frame->buf[6] = getLockTargetAdjustedValue(0x16, true);   // torque loss/slippage.  0x20 makes 198, 0xFE makes 131, 0xFF makes 198
        //frame->buf[7] = 0x00;  // drivers moment
        break;
      case MOTOR3_ID:
        frame->buf[2] = getLockTargetAdjustedValue(0xFE, false);  // pedal
        frame->buf[7] = getLockTargetAdjustedValue(0xFE, false);  // throttle angle
        break;
      case BRAKES1_ID:
        //frame->buf[1] &= ~0x8;
        //frame->buf[0] = 0xFF;  // asr req
        frame->buf[1] = getLockTargetAdjustedValue(0x00, false);
        frame->buf[2] = 0x0;
        frame->buf[3] = getLockTargetAdjustedValue(0xA, false);
        break;

      case BRAKES3_ID:
        //adjusted_slip = getLockTargetAdjustedValue(0xFE);
        frame->buf[0] = getLockTargetAdjustedValue(0xFE, false);
        frame->buf[1] = 0xA;
        frame->buf[2] = getLockTargetAdjustedValue(0xFE, false);
        frame->buf[3] = 0xA;
        frame->buf[4] = 0x0;
        frame->buf[5] = 0xA;
        frame->buf[6] = 0x0;
        frame->buf[7] = 0xA;

        //frame->data->high = (0xa << 24) + (0xa << 8);
        //frame->data->low = frame->data->high + (adjusted_slip << 16) + adjusted_slip;
        break;
    }
  }

  if (haldexGen == 4) {
    switch (frame->id) {
      case MOTOR1_ID:
        //frame->buf[0] = 0x08;                                     // Leergasinformation, Fahrpedalwert, Kickdownschalter, Kupplungsschalter, B_stattoc
        frame->buf[1] = getLockTargetAdjustedValue(0xFE, false);  // inner moment MDNORM miistc
        frame->buf[2] = 0x20;                                     // engine speed nmot_c low byte
        frame->buf[3] = getLockTargetAdjustedValue(0x4E, false);  // engine speed  nmot_c high byte
        //frame->buf[4] = 0x00;  // external moment? MDNORM mifab_w.  Torque without external intervention
        //frame->buf[5] = 0x00;  // throttle pedal wpedc
        frame->buf[6] = getLockTargetAdjustedValue(0x16, false);   // mdverlc was 20; mdverlc - torque loss
        //frame->buf[7] = getLockTargetAdjustedValue(0xFE, false);  // Fahrerwunschmoment mivbeb_w - driver's request
        break;
      case MOTOR3_ID:
        // [0] - Vorgluhmeldung, Ubertemperatur, MotorWunschdreh, MotorWunschdreh, frei, Fahrpedalwert, DK-Winkel, frei, Fehler Ansaugluft
        // [1] - Außentemperatur (0.75*(HEX)-48)
        frame->buf[2] = getLockTargetAdjustedValue(0xFE, false);  // wpedv_w
        // [3] - Rad-Wunschmoment mdwrab_w
        // [4] - Rad-Wunschmoment mdwrab_w and others
        // [5] - Motordrehzahl fgnsol
        // [6] - nmotemi
        frame->buf[7] = getLockTargetAdjustedValue(0x01, false);  // wdkba throttle cut?  Should be 0?  Was FE
        break;
      case MOTOR6_ID:
        // [0] - checksum
        //frame->buf[1] = getLockTargetAdjustedValue(0xFE);  // miautgsc Target torque without P and D part of the | Idle speed controller, without anti-jerk
        //frame->buf[2] = getLockTargetAdjustedValue(0xFE);  // miautgetc  Actual torque without P and D part of the | Idle speed controller, without anti-jerk
      case BRAKES1_ID:
        //frame->buf[1] &= ~0x8;
        //frame->buf[0] = 0x00;                                     // set ASR/MSR/ABS to 0?
        frame->buf[1] = getLockTargetAdjustedValue(0x00, false);                                     // ASR switching influence;
        frame->buf[2] = getLockTargetAdjustedValue(0xFE, false);  // vamsr_c
        frame->buf[3] = 0xA;                                      // vamsr_c
        break;

      case BRAKES3_ID:
        frame->buf[0] = getLockTargetAdjustedValue(0xFE, false);
        frame->buf[1] = 0xA;
        frame->buf[2] = getLockTargetAdjustedValue(0xFE, false);
        frame->buf[3] = 0xA;
        frame->buf[4] = 0x0;
        frame->buf[5] = 0xA;
        frame->buf[6] = 0x0;
        frame->buf[7] = 0xA;

        //frame->data->high = (0xa << 24) + (0xa << 8);
        //frame->data->low = frame->data->high + (adjusted_slip << 16) + adjusted_slip;
        break;
    }
  }
}