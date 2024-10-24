#include "openhaldex.h"

float lockTarget = 0;
float pedValue = 0;

static float get_lockTarget_adjustment(void) {
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

static uint8_t get_lockTarget_adjusted_value(uint8_t value) {
  if (state.mode == MODE_5050) {
    if (pedValue >= state.ped_threshold || state.ped_threshold == 0) {
      return value;
    }

    return 0;
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

    return value * (target_fudge_factor / 100);
  }
}

void getLockData(CAN_message_t *frame) {
  uint8_t adjusted_slip;

  lockTarget = get_lockTarget_adjustment();

  switch (frame->id) {
    case MOTOR1_ID:
      frame->buf[0] = 0;
      frame->buf[1] = get_lockTarget_adjusted_value(0xFA);
      frame->buf[2] = 0x20;
      frame->buf[3] = get_lockTarget_adjusted_value(0x4E);
      frame->buf[4] = get_lockTarget_adjusted_value(0xFE);
      frame->buf[5] = get_lockTarget_adjusted_value(0xFE);
      frame->buf[6] = get_lockTarget_adjusted_value(0x20);  // was 20;
      frame->buf[7] = get_lockTarget_adjusted_value(0xFE);
      break;
    case MOTOR3_ID:
      frame->buf[2] = get_lockTarget_adjusted_value(0xFA);
      frame->buf[7] = get_lockTarget_adjusted_value(0xFE);
      break;
#if 0
        case MOTOR6_ID:
            frame->buf[1] = get_lockTarget_adjusted_value(0xfe);
            frame->buf[2] = get_lockTarget_adjusted_value(0xfe);
            break;
#endif
    case BRAKES1_ID:
      //frame->buf[1] &= ~0x8;
      frame->buf[2] = 0x0;
      frame->buf[3] = 0xA;
      break;

    case BRAKES3_ID:
      adjusted_slip = get_lockTarget_adjusted_value(0xFF);
      frame->buf[0] = adjusted_slip;
      frame->buf[1] = 0xA;
      frame->buf[2] = adjusted_slip;
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