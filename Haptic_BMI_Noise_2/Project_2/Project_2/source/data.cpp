
#include "data.h"
using namespace std;

static char response;   // Y/N response to set-up questions
static save_data temp;  // for temporarily holding one time step of data (that is to be saved)

static shared_data* p_sharedData;  // structure for sharing data between threads


// point p_sharedData to sharedData, which is the data shared between all threads
void linkSharedData(shared_data& sharedData) {
    
    p_sharedData = &sharedData;
    
}

// set-up for simulation
void setup(void) {
    // initialize all elements of p_sharedData structure


	// Simulation State
	p_sharedData->simulationRunning = false; // selected during setup/initialization
	p_sharedData->simulationFinished = false;// executed by experiment

	// debug toggle
	p_sharedData->noise_toggle =  true;
	p_sharedData->friction_toggle = true;

	// input devices 
	p_sharedData->input_device = 0;

	// output devices
	p_sharedData->output_device = 0;

	// graphics
	p_sharedData->message = " ";

	// state initializations
	p_sharedData->experimentStateName = " ";
	p_sharedData->experimentStateNumber = 0;
	p_sharedData->blockNum = 0;			// number of current block
	p_sharedData->blockName = "init";		// name of the current block (i.e. Haptics_Block, Vision_Block)
	p_sharedData->trialNum = 0;			// current trial number
	
	//filter parameters

	p_sharedData->cutoff_freq = "15hz at 50hz sampling";
	p_sharedData->a[0] = 0.577240524806302;	p_sharedData->a[1] = 0.421787048689562;	p_sharedData->a[2] = 0.0562972364918426;
	p_sharedData->b[0] = 0.256915601248463;	p_sharedData->b[1] = 0.77074680374539;
	p_sharedData->b[2] = p_sharedData->b[1]; p_sharedData->b[3] = p_sharedData->b[0];
	
	p_sharedData->cutoff_freq1 = "5hz at 50hz sampling";
	p_sharedData->a1[0] = -1.76004188034317;	p_sharedData->a1[1] = 1.18289326203783;	p_sharedData->a1[2] = -0.278059917634546;
	p_sharedData->b1[0] = 0.0180989330075144;	p_sharedData->b1[1] = 0.0542967990225433;
	p_sharedData->b1[2] = p_sharedData->b1[1]; p_sharedData->b1[3] = p_sharedData->b[0];

	p_sharedData->current_filter_setting = p_sharedData->cutoff_freq;

	//noise parameter
	p_sharedData->sigma_flag=true;
	p_sharedData->current_sigma = SIGMA1;

	p_sharedData->noiseX = 0;
	p_sharedData->noiseY = 0;
	p_sharedData->noiseZ = 0;

	// cursor parameters
	p_sharedData->cursorPosX = 0;	// current cursor x position NOTE: This should be set to our desired z position in the space
	p_sharedData->cursorPosY = 0;	// current cursor y position
	p_sharedData->cursorPosZ = 0;	// current cursor z position 

	p_sharedData->cursorPosX_OneAgo = 0;
	p_sharedData->cursorPosY_OneAgo = 0;
	p_sharedData->cursorPosZ_OneAgo = 0;

	p_sharedData->cursorVelX = 0;
	p_sharedData->cursorVelY = 0;
	p_sharedData->cursorVelZ = 0;

	// Input Phantom state
	p_sharedData->inputPhantomPosX = 0;
	p_sharedData->inputPhantomPosY = 0;
	p_sharedData->inputPhantomPosZ = 0;

	p_sharedData->inputPhantomVelX = 0;
	p_sharedData->inputPhantomVelY = 0;
	p_sharedData->inputPhantomVelZ = 0;

	p_sharedData->inputPhantomSwitch = 0;

	// Output Phantom state
	p_sharedData->outputPhantomPosX = 0;
	p_sharedData->outputPhantomPosY = 0;
	p_sharedData->outputPhantomPosZ = 0;

	p_sharedData->outputPhantomVelX = 0;
	p_sharedData->outputPhantomVelY = 0;
	p_sharedData->outputPhantomVelZ = 0;

	p_sharedData->outputPhantomSwitch = 0;

	// Output Phantom desired force output
	p_sharedData->outputPhantomForce_Desired_X = 0;
	p_sharedData->outputPhantomForce_Desired_Y = 0;
	p_sharedData->outputPhantomForce_Desired_Z = 0;

	// Output Phantom current force output
	p_sharedData->outputPhantomForce_X = 0;
	p_sharedData->outputPhantomForce_Y = 0;
	p_sharedData->outputPhantomForce_Z = 0;

	// Joystick State
	p_sharedData->joystickPosX = 0;
	p_sharedData->joystickPosY = 0;
	p_sharedData->joystickSwitch = 0;

	// Time Stamps
	p_sharedData->phantomLoopTimeStamp = 0;
	p_sharedData->joystickLoopTimeStamp = 0;
	p_sharedData->experimentLoopTimeStamp = 0;
	p_sharedData->recordTimeStamp = 0;

	// Loop Rate Stamps (delta Time)
	p_sharedData->phantomLoopDelta = 0;
	p_sharedData->joystickLoopDelta = 0;
	p_sharedData->experimentLoopDelta = 0;

	// Frequency Counter reported loop frequency in Hz
	p_sharedData->phantomFreq = 0;
	p_sharedData->joystickFreq = 0;
	p_sharedData->experimentFreq = 0;

	// elapsed time 
	p_sharedData->timeElapsed = 0;

    // create timers, PHANTOM device handler
    // NOTE: only use these constructors once (at beginning of main) to avoid pointer issues
    p_sharedData->timer = new cPrecisionClock();

	// New handler instances for phantom devices
    p_sharedData->p_Phantom_Handler = new cHapticDeviceHandler();
	int numHapticDevices = p_sharedData->p_Phantom_Handler->getNumDevices();

	// Assign haptic devices to handler for easy reference via pointers
	p_sharedData->p_Phantom_Handler->getDevice(p_sharedData->p_input_Phantom, 0);   // 1st available haptic device
	p_sharedData->p_Phantom_Handler->getDevice(p_sharedData->p_output_Phantom, 1); // 2nd available haptic device

	//get device specifications
	p_sharedData->inputPhantom_spec = p_sharedData->p_input_Phantom->getSpecifications();
	p_sharedData->outputPhantom_spec = p_sharedData->p_output_Phantom->getSpecifications();

	// Create instance of joystick
	p_sharedData->p_Joystick = new jsJoystick(0);

	// ask for operating mode (defaults to demo)
    printf("\nIs this going to be an Experiment(E) or Demo(D)?\n");
    cin >> response;
    if (response == 'e' || response == 'E') {

		// Ask for type of input 
		printf("\nWhat is your input device?");
		printf("\n(1) PHANTOM, (2) JOYSTICK\n");
        cin >> response;
		if (response == '1') p_sharedData->input_device = INPUT_PHANTOM;
		else if (response == '2') p_sharedData->input_device = INPUT_JOYSTICK;
	

		// ask for type of output device
		printf("\nWhat is your output device?");
		printf("\n(1) PHANTOM, (2) DELTA\n");
		cin >> response;
		if (response == '1') p_sharedData->output_device = OUTPUT_PHANTOM;
		else if (response == '2') p_sharedData->output_device = OUTPUT_DELTA;

		// set operation mode to experiment
        p_sharedData->opMode = EXPERIMENT;
    }
    // if demo, ask for input device 
    else if (response == 'd' || response == 'D'){
		p_sharedData->opMode = DEMO;
        printf("\nWhat is your input device?");
        printf("\n(1) PHANTOM, (2) JOYSTICK\n");
        cin >> response;
        if (response == '1') p_sharedData->input_device = INPUT_PHANTOM;
        else if (response == '2') p_sharedData->input_device = INPUT_JOYSTICK;
		
		//hard code the output device to be a phantom
		p_sharedData->output_device = OUTPUT_PHANTOM;
	}	
}

// save one time step of data to vector for current trial
void saveOneTimeStep(void) {
    // Time Stamp
	p_sharedData->recordTimeStamp = timeGetTime();
	
	// create temporary save_data structure
	save_data temp;
    
    // record individual parameters
	temp.d_blockNum = p_sharedData->blockNum;			// number of current block
	temp.d_blockName = p_sharedData->blockName;		// name of the current block (i.e. Haptics_Block, Vision_Block)
	temp.d_trialNum = p_sharedData->trialNum;			// current trial number
	
	//noise parameters
	temp.d_noiseX = p_sharedData->noiseX;
	temp.d_noiseY = p_sharedData->noiseY;
	temp.d_noiseZ = p_sharedData->noiseZ; 

	// cursor parameters
	temp.d_cursorPosX = p_sharedData->cursorPosX;	// current cursor x position
	temp.d_cursorPosY = p_sharedData->cursorPosY;	// current cursor y position
	temp.d_cursorPosZ = p_sharedData->cursorPosZ;	// current curosr z position

	temp.d_cursorPosX_OneAgo = p_sharedData->cursorPosX_OneAgo;
	temp.d_cursorPosY_OneAgo = p_sharedData->cursorPosY_OneAgo;
	temp.d_cursorPosZ_OneAgo = p_sharedData->cursorPosZ_OneAgo;

	temp.d_cursorVelX = p_sharedData->cursorVelX;
	temp.d_cursorVelY = p_sharedData->cursorVelY;
	temp.d_cursorVelZ = p_sharedData->cursorVelZ;

	// Input Phantom state
	temp.d_inputPhantomPosX = p_sharedData->inputPhantomPosX;
	temp.d_inputPhantomPosY = p_sharedData->inputPhantomPosY;
	temp.d_inputPhantomPosZ = p_sharedData->inputPhantomPosZ;

	temp.d_inputPhantomVelX = p_sharedData->inputPhantomVelX;
	temp.d_inputPhantomVelY = p_sharedData->inputPhantomVelY;
	temp.d_inputPhantomVelZ = p_sharedData->inputPhantomVelZ;

	temp.d_inputPhantomSwitch = p_sharedData->inputPhantomSwitch;

	// Output Phantom state
	temp.d_outputPhantomPosX = p_sharedData->outputPhantomPosX;
	temp.d_outputPhantomPosY = p_sharedData->outputPhantomPosY;
	temp.d_outputPhantomPosZ = p_sharedData->outputPhantomPosZ;

	temp.d_outputPhantomVelX = p_sharedData->outputPhantomVelX;
	temp.d_outputPhantomVelY = p_sharedData->outputPhantomVelY;
	temp.d_outputPhantomVelZ = p_sharedData->outputPhantomVelZ;

	temp.d_outputPhantomSwitch = p_sharedData->outputPhantomSwitch;

	// Output Phantom current force output
	temp.d_outputPhantomForce_X = p_sharedData->outputPhantomForce_X;
	temp.d_outputPhantomForce_Y = p_sharedData->outputPhantomForce_Y;
	temp.d_outputPhantomForce_Z = p_sharedData->outputPhantomForce_Z;


	// Joystick State
	temp.d_joystickPosX = p_sharedData->joystickPosX;
	temp.d_joystickPosY = p_sharedData->joystickPosY;
	temp.d_joystickSwitch = p_sharedData->joystickSwitch;


	// Time Stamps
	temp.d_phantomLoopTimeStamp = p_sharedData->phantomLoopTimeStamp;
	temp.d_joystickLoopTimeStamp = p_sharedData->joystickLoopTimeStamp;
	temp.d_experimentLoopTimeStamp = p_sharedData->experimentLoopTimeStamp;
	temp.d_recordTimeStamp = p_sharedData->recordTimeStamp;

	// Loop Rate Stamps (delta Time)
	temp.d_phantomLoopDelta = p_sharedData->phantomLoopDelta;
	temp.d_joystickLoopDelta = p_sharedData->joystickLoopDelta;
	temp.d_experimentLoopDelta = p_sharedData->experimentLoopDelta;

	// Frequency Counter reported loop frequency in Hz
	temp.d_phantomFreq = p_sharedData->phantomFreq;
	temp.d_joystickFreq = p_sharedData->joystickFreq;
	temp.d_experimentFreq = p_sharedData->experimentFreq;

	// trial elapsed time
	temp.d_timeElapsed = p_sharedData->timeElapsed;
   
    // push into vector for current trial
	p_sharedData->trialData.push_back(temp);
    
}

// write data to file from current trial
void recordTrial(void) {
    
    // iterate over vector, writing one time step at a time
    for (vector<save_data>::iterator it = p_sharedData->trialData.begin() ; it != p_sharedData->trialData.end(); ++it) {
        fprintf(p_sharedData->outputFile,"%d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %lu %lu %lu %lu %lu %lu %lu %f %f %f %f",
                	it->d_blockNum,
					it->d_trialNum,
					it->d_noiseX,
					it->d_noiseY,
					it->d_noiseZ,
					it->d_cursorPosX,
					it->d_cursorPosY,
					it->d_cursorPosZ,
					it->d_cursorPosX_OneAgo,
					it->d_cursorPosY_OneAgo,
					it->d_cursorPosZ_OneAgo,
					it->d_cursorVelX,
					it->d_cursorVelY,
					it->d_cursorVelZ,
					it->d_inputPhantomPosX,
					it->d_inputPhantomPosY,
					it->d_inputPhantomPosZ,
					it->d_inputPhantomVelX,
					it->d_inputPhantomVelY,
					it->d_inputPhantomVelZ,
					it->d_inputPhantomSwitch,
					it->d_outputPhantomPosX,
					it->d_outputPhantomPosY,
					it->d_outputPhantomPosZ,
					it->d_outputPhantomVelX,
					it->d_outputPhantomVelY,
					it->d_outputPhantomVelZ,
					it->d_outputPhantomSwitch,
					it->d_outputPhantomForce_X,
					it->d_outputPhantomForce_Y,
					it->d_outputPhantomForce_Z,
					it->d_joystickPosX,
					it->d_joystickPosY,
					it->d_joystickSwitch,
					it->d_phantomLoopTimeStamp,
					it->d_joystickLoopTimeStamp,
					it->d_experimentLoopTimeStamp,
					it->d_recordTimeStamp,
					it->d_phantomLoopDelta,
					it->d_joystickLoopDelta,
					it->d_experimentLoopDelta,
					it->d_phantomFreq,
					it->d_joystickFreq,
					it->d_experimentFreq,
					it->d_timeElapsed
				);

		// print to file
		fprintf(p_sharedData->outputFile, "\n");

	}
    
    // clear vector for next segment and signal that recording is done
    p_sharedData->trialData.clear();
    
}
