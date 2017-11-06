#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include "../../../external/chai3d-3.0.0/src/chai3d.h"
#include "NIDAQcommands.h"
#include "NIDAQmx.h"
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
#define PRE_BLOCK   5
#define RECORD		2
#define RELAX		3
#define BREAK		4
#define THANKS		6

// Loop rate parameters
#define LOOP_TIME 0.001  // for regulating thread loop rates (sec) (1Khz)

// Graphics
#define CURSOR_SIZE 0.003 // cursor radius
#define OBJECT_SCALE_FACTOR 3
#define WORKSPACE_RADIUS 0.5
#define VIRTUAL_CURSOR_VPOS 0.11
#define MESH_POSX -0.065
#define MESH_POSY 0.0
#define MESH_POSZ 0.0

// Noise Generation
#define SIGMA 0.015
#define C_INTERVAL 0.077274879106467 // this value is half the range that contains 99% of the values of the distribution at sigma 0.03 , please recalculate this if you change sigma
#define B0 0.000095442508423
#define B1 0.000286327525271
#define B2 0.000286327525271
#define B3 0.000095442508423
#define A0 -2.811573677324689
#define A1 2.640483492778340
#define	A2 -0.8281462753862610

// Material Parameters
#define FRICTION_MU 0.3 //Mu value
#define T_STIFF_MULT 0.75 //Tissue stiffness multiplier

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
	DWORD d_experimentLoopTimeStamp;
	DWORD d_recordTimeStamp;

	// Loop Rate Stamps (delta Time)
	DWORD d_phantomLoopDelta;
	DWORD d_joystickLoopDelta;
	DWORD d_experimentLoopDelta;

	// Frequency Counter reported loop frequency in Hz
	double d_phantomFreq;
	double d_joystickFreq;
	double d_experimentFreq;

	double d_timeElapsed;

} save_data; 







///////////// shared data structure
// data to share between all the threads
typedef struct {
	// operation mode
	int opMode;

	// Simulation State
	bool simulationRunning; // selected during setup/initialization
	bool simulationFinished;// executed by experiment

	// debug toggle
	bool noise_toggle;

	// input devices 
	int input_device;

	// output devices
	int output_device;

	int blockNum;			// number of current block
	string blockName;		// name of the current block (i.e. Haptics_Block, Vision_Block)
	int trialNum;			// current trial number

	// graphics
	string message;

	// Mesh Objects
	cMultiMesh* p_vholeCasing;
	cMultiMesh* p_vholeCover;
	cMultiMesh* p_vholeSurface[5];

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
	cHapticDeviceHandler* p_Phantom_Handler; // Handler for input Phantom device
	cGenericHapticDevicePtr p_input_Phantom;	// ptr to input phantom device
	cGenericHapticDevicePtr p_output_Phantom;	// ptr to output phantom device
	jsJoystick* p_Joystick;						// ptr to joystick device

	//haptic device specification data (e.g max linear stiffness and etc.)
	cHapticDeviceInfo inputPhantom_spec;
	cHapticDeviceInfo outputPhantom_spec;

	// haptic tool pointers
	cToolCursor* tool;

	// virtual cursor
	cShapeSphere* vCursor;

	// thread frequency counter
	cFrequencyCounter phantomFreqCounter;
	cFrequencyCounter joystickFreqCounter;
	cFrequencyCounter experimentFreqCounter;

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
	cPrecisionClock m_expLoopTimer;
	
	// Time Stamps
	DWORD phantomLoopTimeStamp;
	DWORD joystickLoopTimeStamp;
	DWORD experimentLoopTimeStamp;
	DWORD recordTimeStamp;

	// Loop Rate Stamps (delta Time)
	DWORD phantomLoopDelta;
	DWORD joystickLoopDelta;
	DWORD experimentLoopDelta;

	// Frequency Counter reported loop frequency in Hz
	double phantomFreq;
	double joystickFreq;
	double experimentFreq;
	
	// experiment trial elapsed time
	double timeElapsed;

} shared_data;



#endif