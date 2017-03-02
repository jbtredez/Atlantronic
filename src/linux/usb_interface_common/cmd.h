#ifndef CMD_H
#define CMD_H

#include "linux/usb_interface_common/cli.h"
#include "linux/usb_interface_common/robot_interface.h"
#include "linux/usb_interface_common/qemu.h"

int cmd_init(RobotInterface* robot, Qemu* qemu, void (*f)(void));

#endif
