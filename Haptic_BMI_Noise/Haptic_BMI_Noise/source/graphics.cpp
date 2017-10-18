
#include "graphics.h"
#include <math.h>
#include "shared_data.h"
using namespace chai3d;
using namespace std;

static bool DEBUG_DISPLAYS = true; // if true then all debug labels will be displayed

static const int Tgraphics = 20;  // minimum time between graphic updates [msec] = 50 Hz

static int screenW;              // screen width (for relative window sizing/positioning)
static int screenH;              // screen height
static int windowW;              // graphics window width
static int windowH;              // graphics window height
static int windowPosX;           // graphics window X-coordinate
static int windowPosY;           // graphics window Y-coordinate
static bool fullscreen = false;  // for toggling in/out of fullscreen

static cWorld* world;             // CHAI world
static cCamera* camera;           // camera to render the world
static cDirectionalLight* light;  // light to illuminate the world
static cShapeSphere* cursor;      // create cursor 
cMaterial cursorMat;			  // material for coloring cursor

// Labels for display 
static cLabel* opMode;            // label to display simulation operating mode
static cLabel* input_device;      // label to display input device
static cLabel* output_device;	  // label to display output device

static cLabel* phantomRate;       // label to display PHANTOM rate
static cLabel* trial;             // label to display trial count
static cLabel* message;           // label to display message to user

static cLabel* XForce;			  // label to display the x force
static cLabel* YForce;			  // label to display the y force
static cLabel* ZForce;			  // label to display the z force

static cLabel* cursorPosX;			// label to display cursor x position
static cLabel* cursorPosY;			// label to display cursor y position
static cLabel* cursorPosZ;

static cLabel* cursorVelX;
static cLabel* cursorVelY;
static cLabel* cursorVelZ;

static cLabel* experimentState;
static cLabel* blockType;

// phantom positions
static cLabel* input_phantomPosX;
static cLabel* input_phantomPosY;
static cLabel* input_phantomPosZ;

static cLabel* output_phantomPosX;
static cLabel* output_phantomPosY;
static cLabel* output_phantomPosZ;

// phantom velocities
static cLabel* input_phantomVelX;
static cLabel* input_phantomVelY;
static cLabel* input_phantomVelZ;

static cLabel* output_phantomVelX;
static cLabel* output_phantomVelY;
static cLabel* output_phantomVelZ;



static shared_data* p_sharedData;  // structure for sharing data between threads (NOTE: for graphics,
                                  

// point p_sharedData to sharedData, which is the data shared between all threads
void linkSharedDataToGraphics(shared_data& sharedData) {
    
    p_sharedData = &sharedData;
    
}

// initialize graphics before starting the main graphics rendering loop
void initGraphics(int argc, char* argv[]) {
    
    // initialize GLUT & GLEW
    glutInit(&argc, argv);
    
    // size/position graphics window
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = (int)(0.8 * screenH);
    windowH = (int)(0.5 * screenH);
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY;
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow(argv[0]);
    glutSetWindowTitle("Haptic BCI Noise");
    if (fullscreen) glutFullScreen();
    
    // initialize GLEW library (must happen after window creation)
    glewInit();
    
    // set up GLUT event handlers
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(respToKey);
    glutReshapeFunc(resizeWindow);
    
    // create a black world
    world = new cWorld();
    world->m_backgroundColor.setBlack();
    
    // position/orient the camera in the world
    camera = new cCamera(world);
    world->addChild(camera);
    camera->set(cVector3d (0.0, 0.0, 0.5),   // camera position
                cVector3d (0.0, 0.0, 0.0),   // look at center
                cVector3d (0.0, 1.0, 0.0));  // "up" vector
    camera->setClippingPlanes(0.01, 10.0);
    
    // add the light source
    light = new cDirectionalLight(world);
    world->addChild(light);
    light->setEnabled(true);
    light->setDir(0.0, 0.0, 0.0);
    
    // create colored spheres to represent target and cursor
    cursor = new cShapeSphere(CURSOR_SIZE);
	cursor->m_material->setWhite();			 // white

    world->addChild(cursor);
    
    // create labels
    cFont* font = NEW_CFONTCALIBRI20();
    opMode = new cLabel(font);
    input_device = new cLabel(font);
	output_device = new cLabel(font);
    phantomRate = new cLabel(font);
    trial = new cLabel(font);
    message = new cLabel(font);
	XForce = new cLabel(font);
	YForce = new cLabel(font);
	ZForce = new cLabel(font);
	cursorPosX = new cLabel(font);
	cursorPosY = new cLabel(font);
	cursorPosZ = new cLabel(font);
	experimentState = new cLabel(font);
	blockType = new cLabel(font);
	input_phantomPosX = new cLabel(font);
	input_phantomPosY = new cLabel(font);
	input_phantomPosZ = new cLabel(font);

	input_phantomVelX = new cLabel(font);
	input_phantomVelY = new cLabel(font);
	input_phantomVelZ = new cLabel(font);

	output_phantomPosX = new cLabel(font);
	output_phantomPosY = new cLabel(font);
	output_phantomPosZ = new cLabel(font);

	output_phantomVelX = new cLabel(font);
	output_phantomVelY = new cLabel(font);
	output_phantomVelZ = new cLabel(font);

	cursorVelX = new cLabel(font);
	cursorVelY = new cLabel(font);
	cursorVelZ = new cLabel(font);

	// add labels to frontLayer
    camera->m_frontLayer->addChild(opMode);
    camera->m_frontLayer->addChild(input_device);
	camera->m_frontLayer->addChild(output_device);
    camera->m_frontLayer->addChild(phantomRate);
    camera->m_frontLayer->addChild(trial);
    camera->m_frontLayer->addChild(message);
	camera->m_frontLayer->addChild(XForce);
	camera->m_frontLayer->addChild(YForce);
	camera->m_frontLayer->addChild(ZForce);
	camera->m_frontLayer->addChild(cursorPosX);
	camera->m_frontLayer->addChild(cursorPosY);
	camera->m_frontLayer->addChild(cursorPosZ);
	camera->m_frontLayer->addChild(cursorVelX);
	camera->m_frontLayer->addChild(cursorVelY);
	camera->m_frontLayer->addChild(cursorVelZ);

	camera->m_frontLayer->addChild(experimentState);
	camera->m_frontLayer->addChild(blockType);
	camera->m_frontLayer->addChild(input_phantomPosX);
	camera->m_frontLayer->addChild(input_phantomPosY);
	camera->m_frontLayer->addChild(input_phantomPosZ);

	camera->m_frontLayer->addChild(output_phantomPosX);
	camera->m_frontLayer->addChild(output_phantomPosY);
	camera->m_frontLayer->addChild(output_phantomPosZ);

	camera->m_frontLayer->addChild(input_phantomVelX);
	camera->m_frontLayer->addChild(input_phantomVelY);
	camera->m_frontLayer->addChild(input_phantomVelY);
}

// update and re-render the graphics
void updateGraphics(void) {
    
	
	///////////////////////////////
	cursor->setLocalPos(p_sharedData->cursorPosX, p_sharedData->cursorPosY, p_sharedData->cursorPosZ);
    
    // update labels
	// operation mode
	switch (p_sharedData->opMode){
		case DEMO : 
			opMode->setString("Mode: DEMO");
		break;

		case EXPERIMENT :
			opMode->setString("Mode: EXPERIMENT");
		break;
	}

	// input device
	switch (p_sharedData->input_device){
		case INPUT_PHANTOM :
			input_device->setString("Input: PHANTOM");
		break;

		case INPUT_JOYSTICK : 
			input_device->setString("Input: JOYSTICK");
		break;
	}

	// output device
	switch (p_sharedData->output_device){
		case OUTPUT_PHANTOM :
			output_device->setString("Output: PHANTOM");
		break;

		case OUTPUT_DELTA :
			output_device->setString("Output: DELTA");
		break;

	}

    phantomRate->setString("PHANTOM: " + to_string(static_cast<long long>(p_sharedData->phantomFreqCounter.getFrequency())));

    trial->setString("Trial: " + to_string(static_cast<long long>(p_sharedData->trialNum)));
	XForce->setString("XForce Des: " + to_string(static_cast<long double>(p_sharedData->outputPhantomForce_X)));
	YForce->setString("YForce Des: " + to_string(static_cast<long double>(p_sharedData->outputPhantomForce_Y)));
	ZForce->setString("ZForce Des: " + to_string(static_cast<long double>(p_sharedData->outputPhantomForce_Z)));
    cursorPosX->setString("CursorPos X: " + to_string(static_cast<long double>(p_sharedData->cursorPosX)));
	cursorPosY->setString("CursorPos Y: " + to_string(static_cast<long double>(p_sharedData->cursorPosY)));
	cursorPosZ->setString("CursorPos Z: " + to_string(static_cast<long double>(p_sharedData->cursorPosZ)));
	experimentState->setString("Experiment State: " + p_sharedData->experimentStateName);
	blockType->setString("Block Name: " + p_sharedData->blockName);
	cursorVelX->setString("cursorVelX: " + to_string(static_cast<long double>(p_sharedData->cursorVelX)));
	cursorVelY->setString("cursorVelY: " + to_string(static_cast<long double>(p_sharedData->cursorVelY)));
	cursorVelZ->setString("cursorVelZ: " + to_string(static_cast<long double>(p_sharedData->cursorVelZ)));

	input_phantomPosX->setString("Input Phantom Pos X: " + to_string(static_cast<long double>(p_sharedData->inputPhantomPosX)));
	input_phantomPosY->setString("Input Phantom Pos Y: " + to_string(static_cast<long double>(p_sharedData->inputPhantomPosY)));
	input_phantomPosZ->setString("Input Phantom Pos Z: " + to_string(static_cast<long double>(p_sharedData->inputPhantomPosZ)));

	input_phantomVelX->setString("Input Phantom Vel X: " + to_string(static_cast<long double>(p_sharedData->inputPhantomVelX)));
	input_phantomVelY->setString("Input Phantom Vel Y: " + to_string(static_cast<long double>(p_sharedData->inputPhantomVelY)));
	input_phantomVelZ->setString("Input Phantom Vel Z: " + to_string(static_cast<long double>(p_sharedData->inputPhantomVelZ)));


    opMode->setLocalPos(10, (int) (windowH - 1.0 * opMode->getHeight()), 0);
    input_device->setLocalPos(10, (int) (windowH - 2.0 * input_device->getHeight()), 0);
	output_device->setLocalPos(10, (int) (windowH - 3.0 * output_device->getHeight()), 0);
    phantomRate->setLocalPos(10, (int) (windowH - 4.0 * phantomRate->getHeight()), 0);
   	
	XForce->setLocalPos(10, (int) (windowH - 5.0 * XForce->getHeight()), 0);
	YForce->setLocalPos(10, (int) (windowH - 6.0 * YForce->getHeight()), 0);
	ZForce->setLocalPos(10, (int) (windowH - 7.0 * YForce->getHeight()), 0);

	cursorPosX->setLocalPos(10, (int) (windowH - 8.0 * cursorPosX->getHeight()),0);
	cursorPosY->setLocalPos(10, (int) (windowH - 9.0 * cursorPosY->getHeight()),0);
	cursorPosZ->setLocalPos(10, (int) (windowH - 10.0 * cursorPosZ->getHeight()),0);

	cursorVelX->setLocalPos(10, (int) (windowH - 11.0 * cursorVelX->getHeight()),0);
	cursorVelY->setLocalPos(10, (int) (windowH - 12.0 * cursorVelY->getHeight()),0);
	cursorVelZ->setLocalPos(10, (int) (windowH - 13.0 * cursorVelZ->getHeight()),0);

	input_phantomPosX->setLocalPos(10, (int) (windowH - 14.0 * input_phantomPosX->getHeight()),0);
	input_phantomPosY->setLocalPos(10, (int) (windowH - 15.0 * input_phantomPosY->getHeight()),0);
	input_phantomPosZ->setLocalPos(10, (int) (windowH - 16.0 * input_phantomPosZ->getHeight()),0);

	output_phantomPosX->setLocalPos(10, (int) (windowH - 17.0 * output_phantomPosX->getHeight()),0);
	output_phantomPosY->setLocalPos(10, (int) (windowH - 18.0 * output_phantomPosY->getHeight()),0);
	output_phantomPosZ->setLocalPos(10, (int) (windowH - 19.0 * output_phantomPosZ->getHeight()),0);
	
	input_phantomVelX->setLocalPos(10, (int) (windowH - 20.0 * input_phantomVelX->getHeight()),0);
	input_phantomVelY->setLocalPos(10, (int) (windowH - 21.0 * input_phantomVelY->getHeight()),0);
	input_phantomVelZ->setLocalPos(10, (int) (windowH - 22.0 * input_phantomVelZ->getHeight()),0);

	trial->setLocalPos((int) (windowW - trial->getWidth() - 10), (int) (windowH - 2.0 * opMode->getHeight()), 0);
	blockType->setLocalPos((int) (windowW - blockType->getWidth() - 10), (int) (0), 0);
        
	// update message to user
    message->setString(p_sharedData->message);
    message->setLocalPos((int) (0.5 * (windowW - message->getWidth())), (int) (0.8 * (windowH - message->getHeight())), 0);  // center of window
    

	cursor->setShowEnabled(DEBUG_DISPLAYS);
	opMode->setShowEnabled(DEBUG_DISPLAYS);
    input_device->setShowEnabled(DEBUG_DISPLAYS);
    XForce->setShowEnabled(DEBUG_DISPLAYS);
	YForce->setShowEnabled(DEBUG_DISPLAYS);
	ZForce->setShowEnabled(DEBUG_DISPLAYS);
	cursorPosX->setShowEnabled(DEBUG_DISPLAYS);
	cursorPosY->setShowEnabled(DEBUG_DISPLAYS);
	cursorPosZ->setShowEnabled(DEBUG_DISPLAYS);
	phantomRate->setShowEnabled(DEBUG_DISPLAYS);
	trial->setShowEnabled(DEBUG_DISPLAYS);
	input_phantomPosX->setShowEnabled(DEBUG_DISPLAYS);
	input_phantomPosY->setShowEnabled(DEBUG_DISPLAYS);
	input_phantomPosZ->setShowEnabled(DEBUG_DISPLAYS);
    input_phantomVelX->setShowEnabled(DEBUG_DISPLAYS);
	input_phantomVelY->setShowEnabled(DEBUG_DISPLAYS);
	input_phantomVelZ->setShowEnabled(DEBUG_DISPLAYS);
	cursorVelX->setShowEnabled(DEBUG_DISPLAYS);
	cursorVelY->setShowEnabled(DEBUG_DISPLAYS);
	cursorVelZ->setShowEnabled(DEBUG_DISPLAYS);
	message->setShowEnabled(DEBUG_DISPLAYS);
    

    // render and (smoothly, via buffer swapping) display world
    camera->renderView(windowW, windowH);
    glutSwapBuffers();
    
    // check for OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: %s\n" << gluErrorString(err);
    
}

// dictates update frequency for the graphics
void graphicsTimer(int data) {
    
    if (p_sharedData->simulationRunning) {
        glutPostRedisplay();  // flag to update CHAI world
    }
    glutTimerFunc(Tgraphics, graphicsTimer, 0);
    
}

// dictates response to resizing the graphics window
void resizeWindow(int W, int H) {
    
    windowW = W;
    windowH = H;
    
}

// dictates response to a keypress within the graphics window
void respToKey(unsigned char key, int x, int y) {
    
    switch (key) {
        
        
        // f/F = fullscreen toggle
        case 'f':
        case 'F':
            
            if (fullscreen) {
                windowPosX = glutGet(GLUT_INIT_WINDOW_X);
                windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
                windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
                windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
                glutPositionWindow(windowPosX, windowPosY);
                glutReshapeWindow(windowW, windowH);
                fullscreen = false;
            } else {
                glutFullScreen();
                fullscreen = true;
            }
            break;



        
        // q/Q = quit, 27 = ESC key
        case 'q':
        case 'Q':
        case 27:
            
            close();
            exit(0);
            break;
            
        default:
            break;
    }
    
}

// shut down the simulation in response to quitting/ESCaping from within graphics window
void close(void) {
    
    
    // terminate the experiment
    //if (p_sharedData->opMode == EXPERIMENT) closeExperiment();
    
    // stop the simulation
    p_sharedData->simulationRunning = false;
    
    // wait for NeuroTouch thread to terminate
    while (!p_sharedData->simulationFinished) cSleepMs(100);
    
    // close all devices
    if ((p_sharedData->input_device == INPUT_PHANTOM) || (p_sharedData->output_device == OUTPUT_PHANTOM))	closePhantom();
 
	// clean up memory
    

}
