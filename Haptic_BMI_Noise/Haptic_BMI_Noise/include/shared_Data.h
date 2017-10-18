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
#define OUTPUT_DELTA		2

// Experiment States
#define START_UP	0

// Loop rate parameters
#define LOOP_TIME 0.001  // for regulating thread loop rates (sec) (1Khz)

// Graphics
#define CURSOR_SIZE 0.008  // cursor radius

////////// SAVE DATA STRUCTURE 
typedef struct {
	int d_blockNum;			// number of current block
	string d_blockName;		// name of the current block (i.e. Haptics_Block, Vision_Block)
	int d_trialNum;			// current trial number
	
	// cursor parameters
	double d_cursorPosX;	// current cursor x position
	double d_cursorPosY;	// current cursor y position
	double d_cursorPosZ;	// current curosr z position

	double d_cursorPosX_OneAgo;
	double d_cursorPosY_OneAgo;
	double d_cursorPosZ_OneAgo;

	double d_cursorVelX;
	double d_cursorVelY;
	double d_cursorVelZ;

	// Input Phantom state
	double d_inputPhantomPosX;
	double d_inputPhantomPosY;
	double d_inputPhantomPosZ;

	double d_inputPhantomVelX;
	double d_inputPhantomVelY;
	double d_inputPhantomVelZ;

	double d_inputPhantomSwitch;

	// Output Phantom state
	double d_outputPhantomPosX;
	double d_outputPhantomPosY;
	double d_outputPhantomPosZ;

	double d_outputPhantomVelX;
	double d_outputPhantomVelY;
	double d_outputPhantomVelZ;

	double d_outputPhantomSwitch;

	// Output Phantom desired force output
	double d_outputPhantomForce_Desired_X;
	double d_outputPhantomForce_Desired_Y;
	double d_outputPhantomForce_Desired_Z;

	// Output Phantom current force output
	double d_outputPhantomForce_X;
	double d_outputPhantomForce_Y;
	double d_outputPhantomForce_Z;


	// Joystick State
	double d_joystickPosX;
	double d_joystickPosY;
	double d_joystickSwitch;

	// Time Stamps
	DWORD d_phantomLoopTimeStamp;
	DWORD d_joystickLoopTimeStamp;
	DWORD d_recordTimeStamp;

	// Loop Rate Stamps (delta Time)
	DWORD d_phantomLoopDelta;
	DWORD d_joystickLoopDelta;

	// Frequency Counter reported loop frequency in Hz
	double d_phantomFreq;
	double d_joystickFreq;

} save_data; 







///////////// shared data structure
// data to share between all the threads
typedef struct {
	// operation mode
	int opMode;

	// Simulation State
	bool simulationRunning; // selected during setup/initialization
	bool simulationFinished;// executed by experiment

	// input devices 
	int input_device;

	// output devices
	int output_device;

	int blockNum;			// number of current block
	string blockName;		// name of the current block (i.e. Haptics_Block, Vision_Block)
	int trialNum;			// current trial number

	// graphics
	string message;

	// state machine states
	int experimentStateNumber;
	string experimentStateName;
	
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

	// Timers for simulation/experiment
	cPrecisionClock* timer;

	// device handlers and pointers
	cHapticDeviceHandler* p_input_Phantom_Handler; // Handler for input Phantom device
	cHapticDeviceHandler* p_output_Phantom_Handler; // Handler for output Phantom device
	cGenericHapticDevicePtr p_input_Phantom;	// ptr to input phantom device
	cGenericHapticDevicePtr p_output_Phantom;	// ptr to output phantom device
	jsJoystick* p_Joystick;						// ptr to joystick device

	// device frequency counter
	cFrequencyCounter phantomFreqCounter;
	cFrequencyCounter joystickFreqCounter;

	// Input Phantom state
	double inputPhantomPosX;
	double inputPhantomPosY;
	double inputPhantomPosZ;

	double inputPhantomVelX;
	double inputPhantomVelY;
	double inputPhantomVelZ;

	double inputPhantomSwitch;

	// Output Phantom state
	double outputPhantomPosX;
	double outputPhantomPosY;
	double outputPhantomPosZ;

	double outputPhantomVelX;
	double outputPhantomVelY;
	double outputPhantomVelZ;

	double outputPhantomSwitch;

	// Output Phantom desired force output
	double outputPhantomForce_Desired_X;
	double outputPhantomForce_Desired_Y;
	double outputPhantomForce_Desired_Z;

	// Output Phantom current force output
	double outputPhantomForce_X;
	double outputPhantomForce_Y;
	double outputPhantomForce_Z;


	// Joystick State
	double joystickPosX;
	double joystickPosY;
	double joystickSwitch;

	// data storage
	vector<save_data> trialData;  // for one trial of experiment
    FILE* outputFile;              // output file for entire experiment (all blocks/trials)

	// timers to regulate thread loop rates
	cPrecisionClock m_phantomLoopTimer;
	cPrecisionClock m_joystickLoopTimer;
	
	// Time Stamps
	DWORD phantomLoopTimeStamp;
	DWORD joystickLoopTimeStamp;
	DWORD recordTimeStamp;

	// Loop Rate Stamps (delta Time)
	DWORD phantomLoopDelta;
	DWORD joystickLoopDelta;

	// Frequency Counter reported loop frequency in Hz
	double phantomFreq;
	double joystickFreq;
	

} shared_data;



#endif