#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include "chai3d.h"
#include "NIDAQcommands.h"
#include "NIDAQmx.h"
#include "cForceSensor.h"
#include "cATIForceSensor.h"
#include "js.h"
using namespace chai3d;
using namespace std;

// Operation Mode
#define DEMO		0
#define EXPERIMENT	1
#define TRAINING	2

// Input devices
#define INPUT_PHANTOM		1
#define INPUT_JOYSTICK		2

// Output devcies
#define OUTPUT_PHANTOM		1

// Experiment States
#define START_UP	0


////////// SAVE DATA STRUCTURE 
typedef struct {
	int d_blockNum;			// number of current block
	string d_blockName;		// name of the current block (i.e. Haptics_Block, Vision_Block)
	int d_trialNum;			// current trial number
	
	// cursor parameters
	double d_cursorPosX;	// current cursor x position
	double d_cursorPosY;	// current cursor y position
	double d_cursorPosZ;	// current curosr z position

	



} save_data; 







///////////// shared data structure
// data to share between all the threads
typedef struct {
	int blockNum;			// number of current block
	string blockName;		// name of the current block (i.e. Haptics_Block, Vision_Block)
	int trialNum;			// current trial number
	
	// cursor parameters
	double cursorPosX;	// current cursor x position
	double cursorPosY;	// current cursor y position
	double cursorPosZ;	// current curosr z position

	double cursorPosX_OneAgo;
	double cursorPosY_OneAgo;
	double cursorPosZ_OneAgo;

	double cursorVelX;
	double cursorVelY;
	double cursorVelZ;


	// device pointers
	cGenericHapticDevicePtr p_input_Phantom;	// ptr to input phantom device
	cGenericHapticDevicePtr p_output_Phantom;	// ptr to output phantom device
	jsJoystick* p_Joystick;						// ptr to joystick device

	// device frequency counter
	cFrequencyCounter inputPhantomFreqCounter;
	cFrequencyCounter outputPhantomFreqCounter;
	cFrequencyCounter joystickFreqCounter;

	// Input Phantom state
	double inputPhantomPosX;
	double inputPhantomPosY;
	double inputPhantomPosZ;
	double inputPhantomPosX_OneAgo;
	double inputPhantomPosY_OneAgo;
	double inputPhantomPosZ_OneAgo;

	double inputPhantomVelX;
	double inputPhantomVelY;
	double inputPhantomVelZ;
	double inputPhantomSwitch;

	// Output Phantom state
	double outputPhantomPosX;
	double outputPhantomPosY;
	double outputPhantomPosZ;
	double outputPhantomPosX_OneAgo;
	double outputPhantomPosY_OneAgo;
	double outputPhantomPosZ_OneAgo;

	double outputPhantomVelX;
	double outputPhantomVelY;
	double outputPhantomVelZ;
	double outputPhantomSwitch;

	// Joystick State
	double joystickPosX;
	double joystickPosY;
	double joystickSwitch;

	// data storage
	vector<save_data> trialData;  // for one trial of experiment
    FILE* outputFile;              // output file for entire experiment (all blocks/trials)

	// timers to regulate thread loop rates
	cPrecisionClock m_input_phantomLoopTimer;
	cPrecisionClock m_output_phantomLoopTimer;
	cPrecisionClock m_joystickLoopTimer;
	
	// Time Stamps
	DWORD inputPhantomLoopTimeStamp;
	DWORD outputPhantomLoopTimeStamp;
	DWORD joystickLoopTimeStamp;
	DWORD recordTimeStamp;
	

} shared_data;



#endif