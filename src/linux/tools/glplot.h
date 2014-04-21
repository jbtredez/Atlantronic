#ifndef GLPLOT_H
#define GLPLOT_H

#include "qemu.h"
#include "robot_interface.h"

int glplot_main(const char* atlantronicPath, int simulation, Qemu* qemu, RobotInterface* robotItf);

void glplot_update();

#endif
