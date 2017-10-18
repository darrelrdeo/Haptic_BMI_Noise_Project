
#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <stdio.h>
#include <string>
#include "chai3d.h"
#include "../additional_include_directories/freeGlut/glut.h"
#include "Phantom.h"
#include "shared_Data.h"

void initGraphics(int argc, char* argv[]);
void linkSharedDataToGraphics(shared_data& sharedData);
void updateGraphics(void);
void graphicsTimer(int data);
void resizeWindow(int w, int h);
void respToKey(unsigned char key, int x, int y);
void close(void);

#endif  // GRAPHICS_H
