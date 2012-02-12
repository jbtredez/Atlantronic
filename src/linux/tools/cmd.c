#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "linux/tools/com.h"
#include "linux/tools/cmd.h"
#include "linux/tools/cli.h"
#include "kernel/driver/usb.h"
#include "foo/control/control.h"
#include "foo/control/trajectory.h"

static void (*cmd_exit_callback)(void) = NULL;
static struct robot_interface* cmd_robot = NULL;

int cmd_help();
int cmd_quit();
int cmd_goto_near(void* arg);
int cmd_max_speed(void* arg);
int cmd_pince_open();
int cmd_pince_close();
int cmd_pince_configure();
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
	{ "max_speed", cmd_max_speed, "vitesse max en % (av, rot)" },
	{ "pince_open", cmd_pince_open, "ouverture des pinces"},
	{ "pince_close", cmd_pince_close, "fermeture des pinces"},
	{ "pince_configure", cmd_pince_configure, "configuration des pinces"},
	{ "q", cmd_quit, "Quit" },
	{ "quit", cmd_quit, "Quit" },
	{ "rotate", cmd_rotate, "rotate(angle)" },
	{ "rotate_to", cmd_rotate_to, "rotate_to(angle)" },
	{ "straight", cmd_straight, "straight(dist)" },
	{ "straight_to_wall", cmd_straight_to_wall, "straight_to_wall(dist)" },
	{ "?", cmd_help, "Synonym for `help'" },
	{ (char *)NULL, (Function *)NULL, (char *)NULL }
};

int cmd_init(struct robot_interface* robot, void (*f)(void))
{
	cmd_robot = robot;
	cmd_exit_callback = f;
	return cli_init(usb_commands);
}

int cmd_help()
{
	log_info("Aide\n");
	return CMD_SUCESS;
}

int cmd_quit()
{
	log_info("Quit\n");
	if(cmd_exit_callback)
	{
		cmd_exit_callback();
	}
	return CMD_QUIT;
}

int cmd_control_param(void* arg)
{
	int kp_av;
	int ki_av;
	int kd_av;
	int kp_rot;
	int ki_rot;
	int kd_rot;
	int kx;
	int ky;
	int kalpha;
	int count = sscanf(arg, "%d %d %d %d %d %d %d %d %d", &kp_av, &ki_av, &kd_av, &kp_rot, &ki_rot, &kd_rot, &kx, &ky, &kalpha);

	if(count != 9)
	{
		log_info("cmd_control_param kp_av ki_av kd_av kp_rot ki_rot kd_rot kx ky kalpha\n");
		return CMD_SUCESS;
	}

	robot_interface_control_set_param(cmd_robot, kp_av, ki_av, kd_av, kp_rot, ki_rot, kd_rot, kx, ky, kalpha);

	return CMD_SUCESS;
}

int cmd_control_print_param()
{
	robot_interface_control_print_param(cmd_robot);

	return CMD_SUCESS;
}

int cmd_straight(void* arg)
{
	float dist;
	int count = sscanf(arg, "%f", &dist);

	if(count != 1)
	{
		log_info("cmd_straight dist\n");
		return CMD_SUCESS;
	}

	robot_interface_straight(cmd_robot, dist);

	return CMD_SUCESS;
}

int cmd_straight_to_wall(void* arg)
{
	float dist;
	int count = sscanf(arg, "%f", &dist);

	if(count != 1)
	{
		log_info("cmd_straight_to_wall dist\n");
		return CMD_SUCESS;
	}

	robot_interface_straight_to_wall(cmd_robot, dist);

	return CMD_SUCESS;
}

int cmd_rotate(void* arg)
{
	float alpha;
	int count = sscanf(arg, "%f", &alpha);

	if(count != 1)
	{
		log_info("cmd_rotate dist\n");
		return CMD_SUCESS;
	}

	robot_interface_rotate(cmd_robot, alpha);

	return CMD_SUCESS;
}

int cmd_rotate_to(void* arg)
{
	float alpha;
	int count = sscanf(arg, "%f", &alpha);

	if(count != 1)
	{
		log_info("cmd_rotate_to dist\n");
		return CMD_SUCESS;
	}

	robot_interface_rotate_to(cmd_robot, alpha);

	return CMD_SUCESS;
}

int cmd_free()
{
	robot_interface_free(cmd_robot);

	return CMD_SUCESS;
}

int cmd_goto_near(void* arg)
{
	float x;
	float y;
	float alpha;
	float dist;
	unsigned int way;
	int count = sscanf(arg, "%f %f %f %f %u", &x, &y, &alpha, &dist, &way);

	if(count != 5)
	{
		log_info("cmd_goto_near x y alpha dist way\n");
		return CMD_SUCESS;
	}

	robot_interface_goto_near(cmd_robot, x, y, alpha, dist, way);

	return CMD_SUCESS;
}

int cmd_max_speed(void* arg)
{
	float v_max_av;
	float v_max_rot;
	int count = sscanf(arg, "%f %f", &v_max_av, &v_max_rot);

	if(count != 2)
	{
		log_info("max_speed v_max_av v_max_rot\n");
		return CMD_SUCESS;
	}

	robot_interface_set_max_speed(cmd_robot, v_max_av, v_max_rot);

	return CMD_SUCESS;
}

int cmd_pince_configure()
{
	robot_interface_pince(cmd_robot, PINCE_CONFIGURE);
	return CMD_SUCESS;
}

int cmd_pince_open()
{
	robot_interface_pince(cmd_robot, PINCE_CLOSE);
	return CMD_SUCESS;
}

int cmd_pince_close()
{
	robot_interface_pince(cmd_robot, PINCE_OPEN);
	return CMD_SUCESS;
}