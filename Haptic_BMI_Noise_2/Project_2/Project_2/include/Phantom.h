
#ifndef PHANTOM_H
#define PHANTOM_H

#include "../../../external/chai3d-3.0.0/src/chai3d.h"
#include "shared_data.h"

void initPhantom(void);
void linkSharedDataToPhantom(shared_data& sharedData);
void updatePhantom(void);
void closePhantom(void);
void updateCursor(void);
double LowPassFilterThirdOrder(
	double a[3],
	double b[4],
	double input,
	double output_buff[3],
	double input_buff[3]);


#endif
