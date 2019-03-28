#include <Joystick.h>
#include "joystick.h"

#define JOYSTICK_CHANNEL_NUMBER 6
#define JOYSTICK_CHANNEL_MIN 1000
#define JOYSTICK_CHANNEL_MID 1500
#define JOYSTICK_CHANNEL_MAX 2000

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_MULTI_AXIS, 2, 0,
  true, true, true, false, false, true,
  false, false, false, false, false);

void setupJoystick() {

  Joystick.setXAxisRange(-127, 127);
  Joystick.setYAxisRange(-127, 127);
  Joystick.setZAxisRange(-127, 127);
  Joystick.setRzAxisRange(-127, 127);

  Joystick.begin();

}

void processJoystick(int (* rcChannelGetCallback)(uint8_t)) {

    int rc_values[JOYSTICK_CHANNEL_NUMBER];

    for (uint8_t i = 0; i < JOYSTICK_CHANNEL_NUMBER; i++) {
        rc_values[i] = rcChannelGetCallback(i);
    }

    // CH 1
    Joystick.setZAxis(map(rc_values[0], JOYSTICK_CHANNEL_MIN, JOYSTICK_CHANNEL_MAX, -127, 127));
    // CH 2
    Joystick.setRzAxis(map(rc_values[1], JOYSTICK_CHANNEL_MIN, JOYSTICK_CHANNEL_MAX, -127, 127));
    // CH 3
    Joystick.setYAxis(map(rc_values[2], JOYSTICK_CHANNEL_MIN, JOYSTICK_CHANNEL_MAX, -127, 127));
    // CH 4
    Joystick.setXAxis(map(rc_values[3], JOYSTICK_CHANNEL_MIN, JOYSTICK_CHANNEL_MAX, -127, 127));
    // CH 5
    Joystick.setButton(0, rc_values[4] < JOYSTICK_CHANNEL_MID ? 0 : 1);
    // CH 6
    Joystick.setButton(1, rc_values[5] < JOYSTICK_CHANNEL_MID ? 0 : 1);

}
