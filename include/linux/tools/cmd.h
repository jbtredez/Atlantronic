#ifndef CMD_H
#define CMD_H

#include "linux/tools/cli.h"
#include "linux/tools/robot_interface.h"

int cmd_init(struct robot_interface* robot, void (*f)(void));

#endif
