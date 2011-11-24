#include <stdio.h>
#include <stdlib.h>

#include "linux/tools/com.h"
#include "linux/tools/cmd.h"
#include "linux/tools/cli.h"
#include "kernel/driver/usb.h"
#include "foo/control/control.h"

static struct com* cmd_com = NULL;

int cmd_help();
int cmd_quit();
int cmd_goto_near(void* arg);
int cmd_straight(void* arg);
int cmd_straight_to_wall(void* arg);
int cmd_rotate(void* arg);
int cmd_rotate_to(void* arg);
int cmd_free();
int cmd_control_param(void*);
int cmd_control_print_param();

COMMAND usb_commands[] = {
	{ "control_param", cmd_control_param, "comtrol_param(kp_av, ki_av, kd_av, kp_rot, ki_rot, kd_rot, kx, ky, kalpha)" },
	{ "control_print_param", cmd_control_print_param, "cmd_control_print_param()"},
	{ "free", cmd_free, "free()" },
	{ "goto_near", cmd_goto_near, "Goto near(x, y, dist, way)" },
	{ "help", cmd_help, "Display this text" },
	{ "q", cmd_quit, "Quit" },
	{ "quit", cmd_quit, "Quit" },
	{ "rotate", cmd_rotate, "rotate(angle)" },
	{ "rotate_to", cmd_rotate_to, "rotate_to(angle)" },
	{ "straight", cmd_straight, "straight(dist)" },
	{ "straight_to_wall", cmd_straight_to_wall, "straight_to_wall(dist)" },
	{ "?", cmd_help, "Synonym for `help'" },
	{ (char *)NULL, (Function *)NULL, (char *)NULL }
};

int cmd_init(struct com* com)
{
	cmd_com = com;
	return cli_init(usb_commands);
}

int cmd_help()
{
	printf("Aide\n");
	return CMD_SUCESS;
}

int cmd_quit()
{
	printf("Quit\n");
	exit(0); // TODO
	return CMD_QUIT;
}

int cmd_control_param(void* arg)
{
	struct control_cmd_param_arg cmd_arg;
	int count = sscanf(arg, "%f %f %f %f %f %f %f %f %f", &cmd_arg.kp_av, &cmd_arg.ki_av, &cmd_arg.kd_av, &cmd_arg.kp_rot, &cmd_arg.ki_rot, &cmd_arg.kd_rot, &cmd_arg.kx, &cmd_arg.ky, &cmd_arg.kalpha);

	if(count != 9)
	{
		printf("cmd_goto_near kp_av ki_av kd_av kp_rot ki_rot kd_rot kx ky kalpha\n");
		return CMD_SUCESS;
	}

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_CONTROL_PARAM;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));
	if(cmd_com)
	{
		com_write(cmd_com, buffer, sizeof(buffer));
	}

	return CMD_SUCESS;
}

int cmd_control_print_param()
{
	char buffer[1];
	buffer[0] = USB_CMD_CONTROL_PRINT_PARAM;
	if(cmd_com)
	{
		com_write(cmd_com, buffer, sizeof(buffer));
	}

	return CMD_SUCESS;
}

int cmd_straight(void* arg)
{
	struct control_cmd_straight_arg cmd_arg;
	int count = sscanf(arg, "%f", &cmd_arg.dist);

	if(count != 1)
	{
		printf("cmd_straight dist\n");
		return 0;
	}

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_STRAIGHT;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	if(cmd_com)
	{
		com_write(cmd_com, buffer, sizeof(buffer));
	}

	return CMD_SUCESS;
}

int cmd_straight_to_wall(void* arg)
{
	struct control_cmd_straight_to_wall_arg cmd_arg;
	int count = sscanf(arg, "%f", &cmd_arg.dist);

	if(count != 1)
	{
		printf("cmd_straight_to_wall dist\n");
		return CMD_SUCESS;
	}

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_STRAIGHT_TO_WALL;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));
	if(cmd_com)
	{
		com_write(cmd_com, buffer, sizeof(buffer));
	}

	return CMD_SUCESS;
}

int cmd_rotate(void* arg)
{
	struct control_cmd_rotate_arg cmd_arg;
	int count = sscanf(arg, "%f", &cmd_arg.angle);

	if(count != 1)
	{
		printf("cmd_rotate dist\n");
		return 0;
	}

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_ROTATE;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));
	if(cmd_com)
	{
		com_write(cmd_com, buffer, sizeof(buffer));
	}

	return CMD_SUCESS;
}

int cmd_rotate_to(void* arg)
{
	struct control_cmd_rotate_to_arg cmd_arg;
	int count = sscanf(arg, "%f", &cmd_arg.angle);

	if(count != 1)
	{
		printf("cmd_rotate_to dist\n");
		return CMD_SUCESS;
	}

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_ROTATE_TO;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));
	if(cmd_com)
	{
		com_write(cmd_com, buffer, sizeof(buffer));
	}

	return CMD_SUCESS;
}

int cmd_free()
{
	char buffer[1];
	buffer[0] = USB_CMD_FREE;
	if(cmd_com)
	{
		com_write(cmd_com, buffer, sizeof(buffer));
	}

	return CMD_SUCESS;
}

int cmd_goto_near(void* arg)
{
	struct control_cmd_goto_near_arg cmd_arg;
	int count = sscanf(arg, "%f %f %f %u", &cmd_arg.x, &cmd_arg.y, &cmd_arg.dist, &cmd_arg.way);

	if(count != 4)
	{
		printf("cmd_goto_near x y dist way\n");
		return CMD_SUCESS;
	}

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_GOTO_NEAR;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));
	if(cmd_com)
	{
		com_write(cmd_com, buffer, sizeof(buffer));
	}

	return CMD_SUCESS;
}
