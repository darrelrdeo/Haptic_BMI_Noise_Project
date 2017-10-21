
#ifndef PHANTOM_H
#define PHANTOM_H

#include "../../../external/chai3d-3.0.0/src/chai3d.h"
#include "shared_data.h"

void initPhantom(void);
void linkSharedDataToPhantom(shared_data& sharedData);
void updatePhantom(void);
void updateCursor(void);
void closePhantom(void);



#endif