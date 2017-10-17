
#ifndef DATA_H
#define DATA_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "chai3d.h"
#include "shared_data.h"
#include "Joystick.h"
#include "js.h"

void linkSharedData(shared_data& sharedData);
void setup(void);
void saveOneTimeStep(void);
void recordTrial(void);

#endif  // DATA_H
