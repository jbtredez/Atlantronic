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

int cmd_ax12_scan();
int cmd_ax12_set_id(void* arg);
int cmd_ax12_set_goal_position(void* arg);
int cmd_help();
int cmd_quit();
int cmd_goto_graph(void* arg);
int cmd_goto_near(void* arg);
int cmd_goto_near_xy(void* arg);
int cmd_localization_set_position(void* arg);
int cmd_max_speed(void* arg);
int cmd_pince_open();
int cmd_pince_close();
int cmd_pince_configure();
int cmd_straight(void* arg);
int cmd_straight_to_wall();
int cmd_recalage();
int cmd_rotate(void* arg);
int cmd_rotate_to(void* arg);
int cmd_free();
int cmd_control_param(void*);
int cmd_control_print_param();

COMMAND usb_commands[] = {
	{ "ax12_scan", cmd_ax12_scan, "scan ax12 id : ax12_scan id"},
	{ "ax12_set_id", cmd_ax12_set_id, "changement d'id des ax12 : ax12_set_id id newid"},
	{ "ax12_set_goal_position", cmd_ax12_set_goal_position, "position cible de l'ax12 : ax12_set_goal_position id alpha"},
	{ "control_param", cmd_control_param, "control_param kp_av ki_av kd_av kp_rot ki_rot kd_rot kx ky kalpha" },
	{ "control_print_param", cmd_control_print_param, "control_print_param"},
	{ "free", cmd_free, "free()" },
	{ "goto_graph", cmd_goto_graph, "goto_graph" },
	{ "goto_near", cmd_goto_near, "goto_near x y alpha dist way avoidance_type" },
	{ "goto_near_xy", cmd_goto_near_xy, "goto_near_xy x y dist way avoidance_type"},
	{ "help", cmd_help, "Display this text" },
	{ "localization_set_position", cmd_localization_set_position, "set robot position : localization_set_position x y alpha"},
	{ "max_speed", cmd_max_speed, "vitesse max en % (av, rot) : max_speed v_max_av v_max_rot" },
	{ "pince_open", cmd_pince_open, "ouverture des pinces"},
	{ "pince_close", cmd_pince_close, "fermeture des pinces"},
	{ "pince_configure", cmd_pince_configure, "configuration des pinces"},
	{ "q", cmd_quit, "Quit" },
	{ "quit", cmd_quit, "Quit" },
	{ "rotate", cmd_rotate, "rotate angle" },
	{ "rotate_to", cmd_rotate_to, "rotate_to angle" },
	{ "recalage", cmd_recalage, "recalage"},
	{ "straight", cmd_straight, "straight dist" },
	{ "straight_to_wall", cmd_straight_to_wall, "straight_to_wall" },
	{ "?", cmd_help, "Synonym for `help'" },
	{ (char *)NULL, (Function *)NULL, (char *)NULL }
};

int cmd_init(struct robot_interface* robot, void (*f)(void))
{
	cmd_robot = robot;
	cmd_exit_callback = f;
	return cli_init(usb_commands);
}

int cmd_ax12_scan()
{
	robot_interface_ax12_scan(cmd_robot);
	return CMD_SUCESS;
}

int cmd_ax12_set_id(void* arg)
{
	unsigned int id;
	unsigned int new_id;
	int count = sscanf(arg, "%u %u", &id, &new_id);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	robot_interface_ax12_set_id(cmd_robot, id, new_id);
	return CMD_SUCESS;
}

int cmd_ax12_set_goal_position(void* arg)
{
	unsigned int id;
	float alpha;
	int count = sscanf(arg, "%u %f", &id, &alpha);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	robot_interface_ax12_set_goal_position(cmd_robot, id, alpha);
	return CMD_SUCESS;
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
		return CMD_ERROR;
	}

	robot_interface_control_set_param(cmd_robot, kp_av, ki_av, kd_av, kp_rot, ki_rot, kd_rot, kx, ky, kalpha);

	return CMD_SUCESS;
}

int cmd_control_print_param()
{
	robot_interface_control_print_param(cmd_robot);

	return CMD_SUCESS;
}

int cmd_localization_set_position(void* arg)
{
	float x;
	float y;
	float alpha;
	int count = sscanf(arg, "%f %f %f", &x, &y, &alpha);

	if(count != 3)
	{
		return CMD_ERROR;
	}

	robot_interface_set_position(cmd_robot, x, y, alpha);

	return CMD_SUCESS;
}

int cmd_straight(void* arg)
{
	float dist;
	int count = sscanf(arg, "%f", &dist);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	robot_interface_straight(cmd_robot, dist);

	return CMD_SUCESS;
}

int cmd_straight_to_wall()
{
	robot_interface_straight_to_wall(cmd_robot);

	return CMD_SUCESS;
}

int cmd_rotate(void* arg)
{
	float alpha;
	int count = sscanf(arg, "%f", &alpha);

	if(count != 1)
	{
		return CMD_ERROR;
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
		return CMD_ERROR;
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
	unsigned int avoidance_type;
	int count = sscanf(arg, "%f %f %f %f %u %u", &x, &y, &alpha, &dist, &way, &avoidance_type);

	if(count != 6)
	{
		return CMD_ERROR;
	}

	robot_interface_goto_near(cmd_robot, x, y, alpha, dist, way, avoidance_type);

	return CMD_SUCESS;
}

int cmd_goto_near_xy(void* arg)
{
	float x;
	float y;
	float dist;
	unsigned int way;
	unsigned int avoidance_type;
	int count = sscanf(arg, "%f %f %f %u %u", &x, &y, &dist, &way, &avoidance_type);

	if(count != 5)
	{
		return CMD_ERROR;
	}

	robot_interface_goto_near_xy(cmd_robot, x, y, dist, way, avoidance_type);

	return CMD_SUCESS;
}

int cmd_goto_graph(void* arg)
{
	(void) arg;
	robot_interface_goto_graph(cmd_robot);

	return CMD_SUCESS;
}

int cmd_max_speed(void* arg)
{
	float v_max_av;
	float v_max_rot;
	int count = sscanf(arg, "%f %f", &v_max_av, &v_max_rot);

	if(count != 2)
	{
		return CMD_ERROR;
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
	robot_interface_pince(cmd_robot, PINCE_OPEN);
	return CMD_SUCESS;
}

int cmd_pince_close()
{
	robot_interface_pince(cmd_robot, PINCE_CLOSE);
	return CMD_SUCESS;
}

int cmd_recalage()
{
	robot_interface_recalage(cmd_robot);
	return CMD_SUCESS;
}
