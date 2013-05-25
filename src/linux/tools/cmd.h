#ifndef CMD_H
#define CMD_H

#include "linux/tools/cli.h"
#include "linux/tools/robot_interface.h"
#include "linux/tools/qemu.h"

int cmd_init(struct robot_interface* robot, struct qemu* qemu, void (*f)(void));

#endif
