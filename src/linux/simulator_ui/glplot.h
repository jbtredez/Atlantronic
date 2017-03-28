#ifndef GLPLOT_H
#define GLPLOT_H

#include "linux/usb_interface_common/Robot.h"

int glplot_main(bool cli, Robot* robot, int RobotCount, int selectRobotId);

void glplot_update();

#endif
