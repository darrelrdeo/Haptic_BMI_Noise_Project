#ifndef Joystick_H
#define Joystick_H

#include "chai3d.h"
#include "shared_Data.h"

void initJoystick(void);
void linkSharedDataToJoystick(shared_data& sharedData);
void updateJoystick(void);
void closeJoystick(void);



#endif  // PHANTOM_H
