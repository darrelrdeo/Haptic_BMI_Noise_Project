
#include "Phantom.h"
#include <cmath>
#include <limits>
#include "ziggurat.h"
#include <stdint.h>

using namespace chai3d;
using namespace std;

#define DEBUG_FLAG

static const double phantomScalar = 2;  // to scale PHANTOM workspace to graphics workspace

cVector3d cursor_pos; //temporary variable for haptic tool cursor position

cVector3d input_pos;  // temporary variable for input PHANTOM position 
cVector3d input_vel;  // temporary variable for input PHANTOM velocity 

cVector3d output_pos;  // temporary variable for output PHANTOM position 
cVector3d output_vel;  // temporary variable for output PHANTOM velocity 

cVector3d output_force; // temp var for output PHANTOM currently output force

static shared_data* p_sharedData;  // structure for sharing data between threads

//noise generator variables
uint32_t rand_seed;
uint32_t kn[128];
float wn[128];
float fn[128]; 
float noise_x,noise_y,noise_z;

//filter variables
double filt_noise_x;
double filt_noise_y;
double filt_noise_z;
double output_buff_x[3] = {0,0,0};
double output_buff_y[3] = {0,0,0};
double output_buff_z[3] = {0,0,0};
double input_buff_x[3] = {0,0,0};
double input_buff_y[3] = {0,0,0};
double input_buff_z[3] = {0,0,0};
double a[3] = {A0,A1,A2};
double b[4] = {B0,B1,B2,B3};

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
	p_sharedData->m_noiseLoopTimer.setTimeoutPeriodSeconds(NOISE_TIME);
	p_sharedData->m_phantomLoopTimer.start();
	p_sharedData->m_noiseLoopTimer.start();

	//initialize random seed
	rand_seed = (uint32_t)cpu_time;
	printf("time seed = %ul \n", rand_seed);


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
		
		if(p_sharedData->m_noiseLoopTimer.timeoutOccurred())
		{
			p_sharedData->m_noiseLoopTimer.stop();

			//compute noise
				r4_nor_setup(kn, fn, wn);
				noise_x = SIGMA * r4_nor (rand_seed, kn,fn,wn);
				noise_y = SIGMA * r4_nor (rand_seed, kn,fn,wn);
				noise_z = SIGMA * r4_nor (rand_seed, kn,fn,wn);

				//pass noise through filter
				filt_noise_x = LowPassFilterThirdOrder(a,b,noise_x,output_buff_x,input_buff_x);
				filt_noise_y = LowPassFilterThirdOrder(a,b,noise_y,output_buff_y,input_buff_y);
				filt_noise_z = LowPassFilterThirdOrder(a,b,noise_z,output_buff_z,input_buff_z);

				//update required variables

				input_buff_x[2] = input_buff_x[1]; input_buff_x[1] = input_buff_x[0]; input_buff_x[0] = noise_x;
				input_buff_y[2] = input_buff_y[1]; input_buff_y[1] = input_buff_y[0]; input_buff_y[0] = noise_y;
				input_buff_z[2] = input_buff_z[1]; input_buff_z[1] = input_buff_z[0]; input_buff_z[0] = noise_z;

				output_buff_x[2] = output_buff_x[1]; output_buff_x[1] = output_buff_x[0]; output_buff_x[0] = filt_noise_x;
				output_buff_y[2] = output_buff_y[1]; output_buff_y[1] = output_buff_y[0]; output_buff_y[0] = filt_noise_y;
				output_buff_z[2] = output_buff_z[1]; output_buff_z[1] = output_buff_z[0]; output_buff_z[0] = filt_noise_z;

				p_sharedData->m_noiseLoopTimer.start(1);
		}

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
				
				


				// Inject filtered noise and update tool position
				if (p_sharedData->noise_toggle)
					{p_sharedData->tool->updatePoseNoisy(filt_noise_x,filt_noise_y,filt_noise_z);}
				else
					{p_sharedData->tool->updatePose();}

				// once we know the proxy-goal vector we can check for saturation,
				// then we can force the cursor position back 
				//p_sharedData->tool->desaturate(); 
				// recalulate the forces after we shift the cursor position.
				p_sharedData->tool->computeInteractionForces();
				

				updateCursor(); //updates the virtual cursor position

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

				// render the appropriate forces through interaction with virtual environment (these desired forces should be computed in experiment thread)
				cVector3d desiredForce = cVector3d(p_sharedData->outputPhantomForce_Desired_X, p_sharedData->outputPhantomForce_Desired_Y, p_sharedData->outputPhantomForce_Desired_Z);
				p_sharedData->p_output_Phantom->setForce(desiredForce);

				// get current forces output by OUTPUT PHANTOM device
				p_sharedData->p_output_Phantom->getForce(output_force);
				p_sharedData->outputPhantomForce_X = output_force.x();
				p_sharedData->outputPhantomForce_Y = output_force.y();
				p_sharedData->outputPhantomForce_Z = output_force.z();

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
	
	
	//This code segment maps cursor position to the "goal sphere"
	p_sharedData->cursorPosY = p_sharedData->tool->getDeviceLocalPos().y();
	p_sharedData->cursorPosZ = p_sharedData->tool->getDeviceLocalPos().z();
	p_sharedData->cursorPosX = p_sharedData->tool->getDeviceLocalPos().x();
	p_sharedData->cursorPosY = p_sharedData->tool->getDeviceGlobalPos().y();
	p_sharedData->cursorPosZ = p_sharedData->tool->getDeviceGlobalPos().z();
	p_sharedData->cursorPosX = p_sharedData->tool->getDeviceGlobalPos().x();

	/*
	//This code segment maps cursor position to the "proxy sphere"
	p_sharedData->cursorPosX = p_sharedData->tool->m_hapticPoint->m_sphereProxy->getLocalPos().x();
	p_sharedData->cursorPosY = p_sharedData->tool->m_hapticPoint->m_sphereProxy->getLocalPos().y();
	p_sharedData->cursorPosZ = p_sharedData->tool->m_hapticPoint->m_sphereProxy->getLocalPos().z();
	*/

	// update cursor position
	//p_sharedData->vCursor->setLocalPos( cVector3d(VIRTUAL_CURSOR_VPOS,p_sharedData->cursorPosY,p_sharedData->cursorPosZ) );
	p_sharedData->vCursor->setLocalPos( cVector3d(p_sharedData->cursorPosX,p_sharedData->cursorPosY,p_sharedData->cursorPosZ) );
}

// Filter Function Third Order
/*
PURPOSE----------------------------------------------------------------------------------
Low pass third order filter filters a discrete signal.
INPUTS------------------------------------------------------------------------------------
double a[3]                 : output coefficients for butterworth filter from MATLAB
double b[4]                 : input coefficients for butterworth filter from MATLAB
double input             : input vector at current time t                      
double output_buff[3]	: previous outputs at t-1,t-2,t-3		                  
double input_buff[3]		: previous inputs  at t-1,t-2,t-3                                  

OUTPUTS-----------------------------------------------------------------------------------
cVector3d filteredSignal : the filtered signal                                    (-)
*/
double LowPassFilterThirdOrder(
	double a[3],
	double b[4],
	double input,
	double output_buff[3],
	double input_buff[3])
{
	//init variables
	double  filteredSignal;

	filteredSignal =	(input*b[0] + input_buff[0] * b[1] + input_buff[1] * b[2] + input_buff[2] * b[3]
						- output_buff[0] * a[0] - output_buff[1]*a[1] - output_buff[2]*a[2]);

	//return value 
	return filteredSignal;
}


// safely close the PHANTOM devices
void closePhantom(void) {
    if (p_sharedData->input_device == INPUT_PHANTOM) p_sharedData->p_input_Phantom->close();
	if (p_sharedData->output_device == OUTPUT_PHANTOM) p_sharedData->p_output_Phantom->close();
}


