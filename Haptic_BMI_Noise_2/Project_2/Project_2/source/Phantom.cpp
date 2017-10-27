
#include "Phantom.h"
#include <cmath>
#include <limits>
using namespace chai3d;
using namespace std;

#define DEBUG_FLAG

static const double phantomScalar = 2;  // to scale PHANTOM workspace to graphics workspace

cVector3d input_pos;  // temporary variable for input PHANTOM position 
cVector3d input_vel;  // temporary variable for input PHANTOM velocity 

cVector3d output_pos;  // temporary variable for output PHANTOM position 
cVector3d output_vel;  // temporary variable for output PHANTOM velocity 

cVector3d output_force; // temp var for output PHANTOM currently output force

static shared_data* p_sharedData;  // structure for sharing data between threads

// thread timestamp vars
static DWORD currTime = 0;
static DWORD lastTime = 0;


// initialize the PHANTOM device
void initPhantom(void) {

    // open and calibrate the input PHANTOM
    bool inPhantomOpened = p_sharedData->p_input_Phantom->open();
    p_sharedData->p_input_Phantom->calibrate();

	// open and calibrate the output PHANTOM
    bool outPhantomOpened = p_sharedData->p_output_Phantom->open();
    p_sharedData->p_output_Phantom->calibrate();

	p_sharedData->p_input_Phantom->setEnableGripperUserSwitch(true);
	p_sharedData->p_output_Phantom->setEnableGripperUserSwitch(true);

    // initialize device loop timer
	p_sharedData->m_phantomLoopTimer.setTimeoutPeriodSeconds(LOOP_TIME);
	p_sharedData->m_phantomLoopTimer.start();

#ifdef DEBUG_FLAG
    printf("\n Phantoms initialized!\n");
#endif
}

// point p_sharedData to sharedData, which is the data shared between all threads
void linkSharedDataToPhantom(shared_data& sharedData) {
    
    p_sharedData = &sharedData;
    
}

// get PHANTOM state and update shared data
void updatePhantom(void) {

    // initialize frequency counter
    p_sharedData->phantomFreqCounter.reset();

	// check whether the simulation is running
    while(p_sharedData->simulationRunning) {
			
		// run loop only if phantomLoopTimer timeout has occurred
        if (p_sharedData->m_phantomLoopTimer.timeoutOccurred()) {

			// ensure the timer has stopped for this loop
			p_sharedData->m_phantomLoopTimer.stop();

			// Get timestamp and compute the delta for looprate
			currTime = timeGetTime();
			DWORD delta = currTime - lastTime;

			// store time stamps for book-keeping
			p_sharedData->phantomLoopDelta = delta;
			p_sharedData->phantomLoopTimeStamp = currTime;

			// if the input device is a phantom then perform updates for input, otherwise skip
            if (p_sharedData->input_device == INPUT_PHANTOM) {
		
				// add noise to the position?
				//p_sharedData->tool->getDeviceLocalPos();
				//p_sharedData->tool->setDeviceLocalPos();

				// Update Tool Cursor pose
				p_sharedData->tool->updatePose();
				updateCursor();

				// compute interaction forces
				p_sharedData->tool->computeInteractionForces();

				// store locally computed interaction forces
				cVector3d computedLocalForce = p_sharedData->tool->getDeviceLocalForce();

				// store this forces as the output phantom desired (global?) output forces
				p_sharedData->outputPhantomForce_Desired_X = computedLocalForce.x();
				p_sharedData->outputPhantomForce_Desired_Y = computedLocalForce.y();
				p_sharedData->outputPhantomForce_Desired_Z = computedLocalForce.z();

                // get INPUT PHANTOM position and velocity vectors
                p_sharedData->p_input_Phantom->getPosition(input_pos);
                p_sharedData->p_input_Phantom->getLinearVelocity(input_vel);
    
                //store position values into respective variable in sharedData structure
				p_sharedData->inputPhantomPosX = input_pos.x();
				p_sharedData->inputPhantomPosY = input_pos.y();
				p_sharedData->inputPhantomPosZ = input_pos.z();

				// store velocity values into respective vars in sharedData structure
                p_sharedData->inputPhantomVelX = input_vel.x();
				p_sharedData->inputPhantomVelY = input_vel.y();
				p_sharedData->inputPhantomVelZ = input_vel.z();

				// Checking for switch press
				bool stat1 = false;
				bool stat2 = false;
				p_sharedData->p_input_Phantom->getUserSwitch(1,stat1);
				p_sharedData->p_input_Phantom->getUserSwitch(2,stat2);
				if(stat1 || stat2) p_sharedData->inputPhantomSwitch = 1;
				else p_sharedData->inputPhantomSwitch = 0;
   
            }

			// if the output device is a phantom then perform updates for output, otherwise skip
			if (p_sharedData->output_device == OUTPUT_PHANTOM) {

				// get current forces output by OUTPUT PHANTOM device
				p_sharedData->p_output_Phantom->getForce(output_force);
				p_sharedData->outputPhantomForce_X = output_force.x();
				p_sharedData->outputPhantomForce_Y = output_force.y();
				p_sharedData->outputPhantomForce_Z = output_force.z();

				// render the appropriate forces through interaction with virtual environment (these desired forces should be computed in experiment thread)
				cVector3d desiredForce = cVector3d(p_sharedData->outputPhantomForce_Desired_X, p_sharedData->outputPhantomForce_Desired_Y, p_sharedData->outputPhantomForce_Desired_Z);
				p_sharedData->p_output_Phantom->setForce(desiredForce);

				// get OUTPUT PHANTOM position and velocity vectors
                p_sharedData->p_output_Phantom->getPosition(output_pos);
                p_sharedData->p_output_Phantom->getLinearVelocity(output_vel);

				 //store position values into respective variable in sharedData structure
				p_sharedData->outputPhantomPosX = output_pos.x();
				p_sharedData->outputPhantomPosY = output_pos.y();
				p_sharedData->outputPhantomPosZ = output_pos.z();

				// store velocity values into respective vars in sharedData structure
                p_sharedData->outputPhantomVelX = output_vel.x();
				p_sharedData->outputPhantomVelY = output_vel.y();
				p_sharedData->outputPhantomVelZ = output_vel.z();


			}
            // update frequency counter
            p_sharedData->phantomFreqCounter.signal(1);
			p_sharedData->phantomFreq = p_sharedData->phantomFreqCounter.getFrequency();

			// restart loop timer after update completion
            p_sharedData->m_phantomLoopTimer.start(true);

			// update timestamp var last
			lastTime = currTime;
        }
    }

}


void updateCursor(void) {
	// position-position mapping between input phantom and virtual cursor
	
	/*
	//This code segment maps cursor position to the "goal sphere"
	p_sharedData->cursorPosY = p_sharedData->tool->getDeviceLocalPos().y();
	p_sharedData->cursorPosZ = p_sharedData->tool->getDeviceLocalPos().z();
	//remove this update because we want the cursor to always be in the same plane
	//p_sharedData->cursorPosX = p_sharedData->tool->getDeviceLocalPos().x();
	*/

	//This code segment maps cursor position to the "proxy sphere"
	p_sharedData->cursorPosY = p_sharedData->tool->m_hapticPoint->m_sphereProxy->getLocalPos().y();
	p_sharedData->cursorPosZ = p_sharedData->tool->m_hapticPoint->m_sphereProxy->getLocalPos().z();

	// update cursor position
	p_sharedData->vCursor->setLocalPos( cVector3d(p_sharedData->cursorPosX,p_sharedData->cursorPosY,p_sharedData->cursorPosZ) );

}





// safely close the PHANTOM devices
void closePhantom(void) {
    if (p_sharedData->input_device == INPUT_PHANTOM) p_sharedData->p_input_Phantom->close();
	if (p_sharedData->output_device == OUTPUT_PHANTOM) p_sharedData->p_output_Phantom->close();
}


