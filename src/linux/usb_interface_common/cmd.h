#ifndef CMD_H
#define CMD_H

#include "linux/usb_interface_common/cli.h"
#include "linux/usb_interface_common/robot_interface.h"
#include "linux/usb_interface_common/qemu.h"

int cmd_init(void (*exit_cb)(void));

int cmd_add_robot(RobotInterface* robot, Qemu* qemu);

int cmd_get_selected_robot();

int cmd_select_robot(int id);

#endif
