#include "Joystick.h"
#include "js.h"
#include <cmath>
#include <limits>
#include <stdlib.h>

using namespace chai3d;
using namespace std;

static shared_data* p_sharedData;

// thread timestamp vars
static DWORD currTime = 0;
static DWORD lastTime = 0;

// define variable to hold axis number
float *ax[1];

// remove joystick bias at zero
static float joystickBias = 0.08;

void initJoystick(void) {
	jsInit();
	// initialize device loop timer
	p_sharedData->m_joystickLoopTimer.setTimeoutPeriodSeconds(LOOP_TIME);
	p_sharedData->m_joystickLoopTimer.start();


  if ( p_sharedData->p_Joystick->notWorking() )
    printf ( "Joystick 0 not detected\n" ) ;
  else
    printf ( "Joystick 0 is \"%s\"\n", p_sharedData->p_Joystick->getName() ) ;

  if ( p_sharedData->p_Joystick->notWorking () ) exit ( 1 ) ;

  ax[0] = new float [ p_sharedData->p_Joystick->getNumAxes() ] ;

}


void linkSharedDataToJoystick(shared_data& sharedData) {
	 p_sharedData = &sharedData;
}

void updateJoystick(void) {

	// initialize frequency counter
    p_sharedData->joystickFreqCounter.reset();

    while(p_sharedData->simulationRunning) {
		 //Time Stamp
			
        if (p_sharedData->m_joystickLoopTimer.timeoutOccurred()) {
			// Get timestamp and compute the delta
			currTime = timeGetTime();
			DWORD delta = currTime - lastTime;
			p_sharedData->joystickLoopTimeStamp = currTime;
			p_sharedData->joystickLoopDelta = delta;

			p_sharedData->m_joystickLoopTimer.stop();

		   if (p_sharedData->input_device == INPUT_JOYSTICK) {
			   // get joystick position and velocity vectors
			   int b ;
			   p_sharedData->p_Joystick->read( &b, ax[0] ) ;
			   int j;
			   p_sharedData->joystickPosX = ax[0][0];
			   p_sharedData->joystickPosY = (-1*ax[0][1]);

			   // remove bias throw in joystick
			   if( abs(p_sharedData->joystickPosX) < joystickBias) p_sharedData->joystickPosX = 0;
			   if( abs(p_sharedData->joystickPosY) < joystickBias) p_sharedData->joystickPosY = 0;

			   if (b > 0) p_sharedData->joystickSwitch = 1;
			   else p_sharedData->joystickSwitch = 0;
			   // update frequency counter
                p_sharedData->joystickFreqCounter.signal(1);
		   }

		   p_sharedData->m_joystickLoopTimer.start(true);
			Sleep(1);

			// update timestamp var last
			lastTime = currTime;

		}

		 
	}
}




void closeJoystick(void) {

}

