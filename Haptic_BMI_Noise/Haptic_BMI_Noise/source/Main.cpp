/*
 * File:   Main.cpp
 * Author: Darrel R. Deo
 *
 * 
 * Created on October 15, 2017, 9:53 PM
 */


/*******************************************************************************
 *                                 INCLUDES                                    *
 ******************************************************************************/
#include <Windows.h>
#include <assert.h>
#include <cstdio>
#include <stdio.h>
#include <string.h>
#include "Phantom.h"
#include "cForceSensor.h"
#include "cATIForceSensor.h"
#include "cDaqHardwareInterface.h"
#include "chai3d.h"
#include "Joystick.h"
#include "shared_data.h"
#include "data.h"

using namespace chai3d;
using namespace std;

// Main or Test Harness Selection
#define MAIN

//------------------------
// Variables & Structures
//------------------------
// threads and the data shared between them (NOTE: graphics is threaded separate from the CThread architecture)

cThread* phantomThread;
cThread* experimentThread;
shared_data *sharedData;


//---------------
// Main Function
//---------------
#ifdef MAIN
int main(int argc, char* argv[]) {

	sharedData = new shared_data();

	// link shared data structure
	linkSharedData(*sharedData);


}
#endif


