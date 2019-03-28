#ifndef JOYSTICK_H
#define JOYSTICK_H

void setupJoystick();
void processJoystick(int (* rcChannelGetCallback)(uint8_t));

#endif
