
#include "data.h"
using namespace std;

//#define RECORD_HAPTIC_LOOP_RATE
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
    
	
}

// save one time step of data to vector for current trial
void saveOneTimeStep(void) {
    // Time Stamp
	p_sharedData->recordTimeStamp = timeGetTime();
	
	// create temporary save_data structure
	save_data temp;
    
    // record individual parameters
   
    // push into vector for current trial
	p_sharedData->trialData.push_back(temp);
    
}

// write data to file from current trial
void recordTrial(void) {
    
    // iterate over vector, writing one time step at a time
    for (vector<save_data>::iterator it = p_sharedData->trialData.begin() ; it != p_sharedData->trialData.end(); ++it) {
        fprintf(p_sharedData->outputFile,"%d %d %f %f",
                it->d_blockNum,
                it->d_trialNum,
				it->d_cursorPosX,
				it->d_cursorPosY
				);

		// print to file
		fprintf(p_sharedData->outputFile, "\n");

	}
    
    // clear vector for next segment and signal that recording is done
    p_sharedData->trialData.clear();
    
}
