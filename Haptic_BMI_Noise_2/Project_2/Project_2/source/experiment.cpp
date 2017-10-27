
#include "experiment.h"
#include <math.h>
#include <cmath>
#include <random>
#include "js.h"
#include <ctime>

using namespace chai3d;
using namespace std;

//#define DEBUG 

// Experiment State Machine params
static const int trialsPerExperimentBlock = 4;	  // trials per experiment block, same as number of targets
static const int trialsBeforeBreak = 2;
static const int trialTime = 10;                  // max time per trial or washout [sec]
static const int breakTime = 10;                 // break time [sec] between blocks
static const int preblockTime = 10;               // time to display message [sec]
static const int recordTime = 1;                  // time to record data [sec]
static const int relaxTime = 10;				// time to relax between large strings of trials within same block
static const int relax_to_trial_time = 5;       // Message prompt in seconds before starting trial after relaxation, requires button press to continue

static const int numberOfBlocks = 2;

// File params
static int subjectNum;           // subject number
static int session;              // session number for subject
static char filename[100];       // output filename
static int nextExperimentState;  // so state machine knows where to go next

int blockNumberIndex = 0;

int userReady = 0;

int* p_randTrial;
int randTrial = 0;

static shared_data* p_sharedData;  // structure for sharing data between threads

// thread timestamp vars
static DWORD currTime = 0;
static DWORD lastTime = 0;

// point p_sharedData to sharedData, which is the data shared between all threads
void linkSharedDataToExperiment(shared_data& sharedData) {
    
    p_sharedData = &sharedData;
    
}


// set-up for experiment
void initExperiment(void) {

    // get subject number, control paradigm, and session number
    printf("\nEnter subject number: ");
    cin >> subjectNum;
    printf("\nEnter session number: ");
    cin >> session;
    printf("\n");
    
    // generate filename and open file for writing
    sprintf(filename, "Subj_%d_Session_%d.dat", subjectNum, session);
    p_sharedData->outputFile = fopen(filename,"w");
    fprintf(p_sharedData->outputFile, "blockNum, blockName, trialNum, cursorPosX, cursorPosY, cursorPosZ, cursorPosX_OneAgo, cursorPosY_OneAgo, cursorPosZ_OneAgo, cursorVelX, cursorVelY, cursorVelZ, inputPhantomPosX, inputPhantomPosY, inputPhantomPosZ, inputPhantomVelX, inputPhantomVelY, inputPhantomVelZ, inputPhantomSwitch, outputPhantomPosX, outputPhantomPosY, outputPhantomPosZ, outputPhantomVelX, outputPhantomVelY, outputPhantomVelZ, outputPhantomSwitch, outputPhantomForce_Desired_X, outputPhantomForce_Desired_Y, outputPhantomForce_Desired_Z, joystickPosX, joystickPosY, joystickSwitch, phantomLoopTimeStamp, joystickLoopTimeStamp, experimentLoopTimeStamp,recordTimeStamp, phantomLoopDelta, joystickLoopDelta, experimentLoopDelta, phantomFreq, joystickFreq, experimentFreq\n");
    
    // enter start-up mode, with force feedback off for safety
   	p_sharedData->opMode = EXPERIMENT;
    p_sharedData->experimentStateNumber = START_UP;
    p_sharedData->message = "Welcome. Press any key to proceed with the experiment.";
    
	// initialize experiment loop timer
	p_sharedData->m_expLoopTimer.setTimeoutPeriodSeconds(LOOP_TIME);
	p_sharedData->m_expLoopTimer.start(true);

	//seed random number generator
	srand(time(NULL));

	//initialize random trial
	p_randTrial = &randTrial;
	*p_randTrial = rand()%5;
    
}




// experiment state machine (only entered if in experiment mode)
void updateExperiment(void) {

	// initialize frequency counter for experiment thread
    p_sharedData->experimentFreqCounter.reset();

    while (p_sharedData->simulationRunning) {
		
		// only update experiment if timer has expired
	    if (p_sharedData->m_expLoopTimer.timeoutOccurred()) {
			

			// Get timestamp and compute the delta
			currTime = timeGetTime();
			DWORD delta = currTime - lastTime;
			p_sharedData->experimentLoopTimeStamp = currTime;
			p_sharedData->experimentLoopDelta = delta;
			
			// stop timer for experiment loop
			p_sharedData->m_expLoopTimer.stop();
			
			if (p_sharedData->opMode == EXPERIMENT) {
						
				// Begin State Machine
				switch (p_sharedData->experimentStateNumber) {

/**********************************************************************************/                    
					// START UP STATE 
					case START_UP:
						p_sharedData->experimentStateName = "START_UP";
						// wait for a keypress
						while(true){
							if (_kbhit()) {
                        
								// ready subject for 1st block of training and experimental trials

								// Obtain the 1st block parameters and update variables
								blockNumberIndex = 1;
								p_sharedData->blockNum = blockNumberIndex; // this is arbitrary for now, will make more sense once we configure experiment
								p_sharedData->blockName = "Experiment";
								
								//Immediately send to Preblock and update message to be displayed in PRE_BLOCK
								p_sharedData->experimentStateNumber = PRE_BLOCK;
								p_sharedData->message = "Beginning Experiment Block " + to_string(static_cast<long long>(p_sharedData->blockNum)) + " : " + p_sharedData->blockName + " in " + to_string(static_cast<long long>(preblockTime)) + " seconds.";
                        
								// set/start timer (from zero)
								p_sharedData->timer->setTimeoutPeriodSeconds(preblockTime);
								p_sharedData->timer->start(true);

								break;
							}
						}
						break; // END : START UP STATE
                    

/**********************************************************************************/

						// PRE BLOCK STATE
					case PRE_BLOCK :
						// set experiment state name
						p_sharedData->experimentStateName = "PRE_BLOCK";
						
						// wait for PRE_BLOCK timer to expire
						if (p_sharedData->timer->timeoutOccurred()) {
							
							// prepare for 1st trial
							p_sharedData->trialNum = 1;

							//initialize hole position
							initHolePos();

							// Initialize cursor state
							initializeCursorState();		

							// set/start timer (from zero) and begin block of trials
							p_sharedData->timer->setTimeoutPeriodSeconds(trialTime);
							p_sharedData->timer->start(true);
							p_sharedData->experimentStateNumber = EXPERIMENT;

						}
						break; // END: PRE BLOCK STATE

/**********************************************************************************/

						// START RECORD STATE
					case RECORD:
						// update experiment state name 
						p_sharedData->experimentStateName = "RECORDING";
						
						// wait for time to expire (more than enough to record data)
						if (p_sharedData->timer->timeoutOccurred()) {
							
							// prep for next trial
							(p_sharedData->trialNum)++;

							// If it is time for a relax
							if((p_sharedData->trialNum % trialsBeforeBreak) == 0){
								p_sharedData->experimentStateNumber = RELAX;
								p_sharedData->message =  "Please Take a break";

								// set/start timer (from zero) and return to block
								p_sharedData->timer->setTimeoutPeriodSeconds(relaxTime);
								p_sharedData->timer->start(true);
								
							}else{ // it is time to go to next trial
							initializeCursorState();

							//initialize hole position
							initHolePos();

							// set/start timer (from zero) and return to block
							p_sharedData->timer->setTimeoutPeriodSeconds(trialTime);
							p_sharedData->timer->start(true);

							p_sharedData->experimentStateNumber = EXPERIMENT;
							}
						}
						break; // END RECORD STATE
					
					
/**********************************************************************************/	
					// START RELAX STATE

					case RELAX:

						// update experiment state name
						p_sharedData->experimentStateName = "RELAX";

						// check that timeout has occured for relaxation and return to next trial after relax_to_trial_time
						if (p_sharedData->timer->timeoutOccurred()){
							p_sharedData->message =  "Press button when ready. Trial will start 5 seconds afterwards.";
							if (p_sharedData->inputPhantomSwitch == 1) {
								userReady = 1;
								p_sharedData->inputPhantomSwitch = 0;
								p_sharedData->timer->setTimeoutPeriodSeconds(relax_to_trial_time);
								p_sharedData->timer->start(true);
							}
						}

							if ((p_sharedData->timer->timeoutOccurred()) && (userReady)){
								userReady = 0;
								initializeCursorState();
								
								// set/start timer (from zero) and return to block
								p_sharedData->timer->setTimeoutPeriodSeconds(trialTime);
								p_sharedData->timer->start(true);

								p_sharedData->experimentStateNumber = EXPERIMENT;
							}

					break;

/**********************************************************************************/	
					// START BREAK STATE
					case BREAK:
						// update experiment state name
						p_sharedData->experimentStateName = "BREAK";
						
						// check if break is over
						if (p_sharedData->timer->timeoutOccurred()) {

							// set/start timer (from zero) and send directly to preBlock
							p_sharedData->timer->setTimeoutPeriodSeconds(preblockTime);
							p_sharedData->timer->start(true);
							p_sharedData->experimentStateNumber = PRE_BLOCK;
						}
						break; // END BREAK STATE
					
					
/**********************************************************************************/
						// START EXPERIMENT STATE
						case EXPERIMENT:
							// update experiment name
							p_sharedData->experimentStateName = "EXPERIMENT";


							// save data from time step
							p_sharedData->timeElapsed = p_sharedData->timer->getCurrentTimeSeconds();

							if (p_sharedData->trialNum <= trialsPerExperimentBlock) // if the trial is within the block save data
							{
								saveOneTimeStep();
							}

						// check if the Experiment block is complete (all trials completed)
						if (p_sharedData->trialNum > trialsPerExperimentBlock) {
                       
							// give subject a break before Experiment block
							p_sharedData->message = "Break: " + to_string(static_cast<long long>(breakTime)) + " seconds until next experiment block.";
                        
							// set/start timer (from zero) send to break with break time
							p_sharedData->timer->setTimeoutPeriodSeconds(breakTime);
							p_sharedData->timer->start(true);
							p_sharedData->experimentStateNumber = BREAK;

							// Iterate to next block type and parameters
							blockNumberIndex++;

							if(blockNumberIndex > numberOfBlocks){
								
								// thank subject and terminate experiment
								p_sharedData->message = "Thank you.";
								p_sharedData->experimentStateNumber = THANKS;
								closeExperiment();
								
							}


						} else {
							// if switch is pressed, denote as end of trial
							if((p_sharedData->inputPhantomSwitch == 1) || (p_sharedData->joystickSwitch == 1)) {
							
								// turn off switch
								p_sharedData->inputPhantomSwitch = 0;
								p_sharedData->message = "You pressed a button!";

								// start recording trial data
								saveOneTimeStep();
								recordTrial();

								// set/start timer (from zero) and send to record state
								p_sharedData->timer->setTimeoutPeriodSeconds(recordTime);
								p_sharedData->timer->start(true);
								p_sharedData->experimentStateNumber = RECORD;



							}

							// unsuccessful trial (i.e., time expired)
							if (p_sharedData->timer->timeoutOccurred()) {
							
								p_sharedData->message = "Time expired!";
                        
								// start recording trial data
								saveOneTimeStep();
								recordTrial();
							
								// set/start timer (from zero) and send to record state
								p_sharedData->timer->setTimeoutPeriodSeconds(recordTime);
								p_sharedData->timer->start(true);
								p_sharedData->experimentStateNumber = RECORD;
							}
						}
						break; // END EXPERIMENT STATE


/**********************************************************************************/

						case THANKS :
							
						break;


/**********************************************************************************/
					
				}

				
			} // END EXPERIMENT STATE MACHINE

			// restart experiment loop timer            
			p_sharedData->m_expLoopTimer.start(true);
			lastTime = currTime;
		}
    }
     
}



// terminate the experiment
void closeExperiment(void) {
    
    p_sharedData->experimentStateNumber = THANKS;
    if (p_sharedData->outputFile != NULL) fclose(p_sharedData->outputFile);
    
}

// Set all velocities and positions to zero (for cursor)
void initializeCursorState(void){
	p_sharedData->cursorPosX = 0;
	p_sharedData->cursorPosY = 0;
	p_sharedData->cursorPosZ = 0;
	
	p_sharedData->cursorPosX_OneAgo = 0;
	p_sharedData->cursorPosY_OneAgo = 0;
	p_sharedData->cursorPosZ_OneAgo = 0;

	p_sharedData->cursorVelX = 0;
	p_sharedData->cursorVelY = 0;
	p_sharedData->cursorVelZ = 0;
}


void initHolePos()
{	
	//clear existing hole position indexed by rand_trial
	p_sharedData->p_vholeSurface[*p_randTrial]->setGhostEnabled(true);
	p_sharedData->p_vholeSurface[*p_randTrial]->setTransparencyLevel(0.0,true,true);

	//generate new random trial
	*p_randTrial = rand()%5;

	//show chosen hole config
	p_sharedData->p_vholeSurface[*p_randTrial]->setGhostEnabled(false);
	p_sharedData->p_vholeSurface[*p_randTrial]->setTransparencyLevel(1.0,true,true);

	//occlude surface
	p_sharedData->p_vholeCover->setGhostEnabled(false);
	p_sharedData->p_vholeCover->setTransparencyLevel(1.0,true,true);

	printf("hole number = %i \n",randTrial);

}

