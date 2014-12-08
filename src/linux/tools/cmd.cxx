#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "linux/tools/com.h"
#include "linux/tools/cmd.h"
#include "linux/tools/cli.h"
#include "kernel/driver/usb.h"

static void (*cmd_exit_callback)(void) = NULL;
static RobotInterface* cmd_robot = NULL;
static Qemu* cmd_qemu = NULL;

int cmd_arm_xyz(const char* arg);
int cmd_arm_ventouse(const char* arg);
int cmd_arm_abz(const char* arg);
int cmd_arm_cmd(const char* arg);
int cmd_can_lss(const char* arg);
int cmd_can_lss_set_nodeid(const char* arg);
int cmd_can_lss_save(const char* arg);
int cmd_can_set_baudrate(const char* arg);
int cmd_can_write(const char* arg);
int cmd_dynamixel_scan(const char* arg);
int cmd_dynamixel_set_id(const char* arg);
int cmd_dynamixel_set_goal_position(const char* arg);
int cmd_dynamixel_set_op_baudrate(const char* arg);
int cmd_dynamixel_set_manager_baudrate(const char* arg);
int cmd_dynamixel_get_position(const char* arg);
int cmd_dynamixel_set_max_torque(const char* arg);
int cmd_dynamixel_set_target_reached_threshold(const char* arg);
int cmd_dynamixel_enable_endless_turn_mode(const char* arg);
int cmd_dynamixel_disable_endless_turn_mode(const char* arg);
int cmd_dynamixel_set_speed(const char* arg);
int cmd_power_off(const char* arg);
int cmd_pwm_set(const char* arg);
int cmd_pump(const char* arg);
int cmd_qemu_set_clock_factor(const char* arg);
int cmd_qemu_set_io(const char* arg);
int cmd_qemu_manage_canopen_connexion(const char* arg);
int cmd_quit(const char* arg);
int cmd_go(const char* arg);
int cmd_go_enable(const char* arg);
int cmd_goto_graph(const char* arg);
int cmd_goto(const char* arg);
int cmd_goto_xy(const char* arg);
int cmd_gyro_calib_start(const char* arg);
int cmd_gyro_calib_stop(const char* arg);
int cmd_gyro_set_theta(const char* arg);
int cmd_gyro_set_calibration_values(const char* arg);
int cmd_heartbeat_disable(const char* arg);
int cmd_heartbeat_update(const char* arg);
int cmd_help(const char* arg);
int cmd_localization_set_position(const char* arg);
int cmd_max_speed(const char* arg);
int cmd_motion_enable(const char* arg);
int cmd_motion_goto(const char*arg);
int cmd_motion_set_max_driving_current(const char* arg);
int cmd_motion_set_actuator_kinematics(const char* arg);
int cmd_motion_set_speed(const char* arg);
int cmd_pince_set_position(const char* arg);
int cmd_ptask(const char* arg);
int cmd_reboot(const char* arg);
int cmd_recalage(const char* arg);
int cmd_rotate(const char* arg);
int cmd_rotate_to(const char* arg);
int cmd_set_color(const char* arg);
int cmd_set_match_time(const char* arg);
int cmd_straight(const char* arg);
int cmd_straight_to_wall(const char* arg);
int cmd_control_param(const char* arg);
int cmd_control_print_param(const char* arg);
int cmd_xbee_set_op_baudrate(const char* arg);
int cmd_xbee_set_manager_baudrate(const char* arg);

COMMAND usb_commands[] = {
	{ "arm_xyz", cmd_arm_xyz, "deplacement du bras (x, y, z, type)"},
	{ "arm_ventouse", cmd_arm_ventouse, "deplacement de la ventouse perpendiculairement au segment [(x1,y1,z) (x2, y2, z)] : arm_ventouse x1 y1 x2 y2 z"},
	{ "arm_abz", cmd_arm_abz, "deplacement du bras (a, b, z)"},
	{ "arm_cmd", cmd_arm_cmd, "arm_cmd type"},
	{ "can_lss", cmd_can_lss, "can_lss on/off"},
	{ "can_lss_set_nodeid", cmd_can_lss_set_nodeid, "can_lss_set_nodeid id"},
	{ "can_lss_save", cmd_can_lss_save, "can_lss_save"},
	{ "can_set_baudrate", cmd_can_set_baudrate, "can_set_baudrate id debug"},
	{ "can_write", cmd_can_write, "can write id size data[0-7]"},
	{ "control_param", cmd_control_param, "control_param kp_av ki_av kd_av kp_rot ki_rot kd_rot kx ky kalpha" },
	{ "control_print_param", cmd_control_print_param, "control_print_param"},
	{ "dynamixel_get_position", cmd_dynamixel_get_position, "donne la position actuelle du dynamixel : dynamixel_get_position type id"},
	{ "dynamixel_scan", cmd_dynamixel_scan, "scan dynamixel id : dynamixel_scan type id"},
	{ "dynamixel_set_id", cmd_dynamixel_set_id, "changement d'id des dynamixels : dynamixel_set_id type id newid"},
	{ "dynamixel_set_goal_position", cmd_dynamixel_set_goal_position, "position cible du dynamixel : dynamixel_set_goal_position type id alpha"},
	{ "dynamixel_set_op_baudrate", cmd_dynamixel_set_op_baudrate, "mise a jour du baudrate en conf OP"},
	{ "dynamixel_set_manager_baudrate", cmd_dynamixel_set_manager_baudrate, "mise a jour du baudrate du gestionnaire dynamixel"},
	{ "dynamixel_set_max_torque", cmd_dynamixel_set_max_torque, "dynamixel_set_max_torque id type val[0 100]"},
	{ "dynamixel_set_target_reached_threshold", cmd_dynamixel_set_target_reached_threshold, "dynamixel_set_target_reached_threshold id type threshold"},
	{ "dynamixel_enable_endless_turn_mode", cmd_dynamixel_enable_endless_turn_mode, "dynamixel_enable_endless_turn_mode  id type"},
	{ "dynamixel_disable_endless_turn_mode", cmd_dynamixel_disable_endless_turn_mode, "dynamixel_disable_endless_turn_mode  id type"},
	{ "dynamixel_set_speed", cmd_dynamixel_set_speed, "dynamixel_set_speed  id type speed"},
	{ "set_match_time", cmd_set_match_time, "set match time"},
	{ "go", cmd_go, "go" },
	{ "go_enable", cmd_go_enable, "go_enable" },
	{ "goto_graph", cmd_goto_graph, "goto_graph" },
	{ "goto", cmd_goto, "goto x y alpha dist way avoidance_type" },
	{ "goto_xy", cmd_goto_xy, "goto_xy x y dist way avoidance_type"},
	{ "gyro_calib_start", cmd_gyro_calib_start, "cmd_gyro_calib_start"},
	{ "gyro_calib_stop", cmd_gyro_calib_stop, "cmd_gyro_calib_stop"},
	{ "gyro_set_theta", cmd_gyro_set_theta, "cmd_gyro_set_theta theta"},
	{ "gyro_set_calibration_values", cmd_gyro_set_calibration_values, "cmd_gyro_set_calibration_values scale bias dead_zone"},
	{ "help", cmd_help, "Display this text" },
	{ "heartbeat_disable", cmd_heartbeat_disable, "heartbeat_disable" },
	{ "heartbeat_update", cmd_heartbeat_update, "heartbeat_update"},
	{ "localization_set_position", cmd_localization_set_position, "set robot position : localization_set_position x y alpha"},
	{ "max_speed", cmd_max_speed, "vitesse max en % (av, rot) : max_speed v_max_av v_max_rot" },
	{ "motion_enable", cmd_motion_enable, "motion_enable enable" },
	{ "motion_goto", cmd_motion_goto, "motion_goto x y theta way type cpx cpy cptheta" },
	{ "motion_set_max_driving_current", cmd_motion_set_max_driving_current, "motion_set_max_driving_current val"},
	{ "motion_set_actuator_kinematics", cmd_motion_set_actuator_kinematics, "set actuators kinematics mode val (x6)"},
	{ "motion_set_speed", cmd_motion_set_speed, "set speed direction valeur"},
	{ "pince_set_position", cmd_pince_set_position, "gestion des pinces: gauche droite"},
	{ "ptask", cmd_ptask, "print tasks"},
	{ "power_off", cmd_power_off, "power off {0,1}"},
	{ "pwm_set", cmd_pwm_set, "pwm_set id val"},
	{ "pump", cmd_pump, "pump id val[0 100]"},
	{ "q", cmd_quit, "Quit" },
	{ "qemu_set_clock_factor", cmd_qemu_set_clock_factor, "qemu_set_clock_factor system_clock_factor icount" },
	{ "qemu_set_io", cmd_qemu_set_io, "qemu_set_io id val" },
	{ "qemu_manage_canopen_connexion", cmd_qemu_manage_canopen_connexion, "qemu connect canopen node : cmd_qemu_manage_canopen_connexion nodeid connected" },
	{ "quit", cmd_quit, "Quit" },
	{ "rotate", cmd_rotate, "rotate angle" },
	{ "rotate_to", cmd_rotate_to, "rotate_to angle" },
	{ "reboot", cmd_reboot, "reboot" },
	{ "recalage", cmd_recalage, "recalage"},
	{ "set_color", cmd_set_color, "set color"},
	{ "straight", cmd_straight, "straight dist" },
	{ "straight_to_wall", cmd_straight_to_wall, "straight_to_wall" },
	{ "xbee_set_op_baudrate", cmd_xbee_set_op_baudrate, "xbee_set_op_baudrate"},
	{ "xbee_set_manager_baudrate", cmd_xbee_set_manager_baudrate, "xbee_set_manager_baudrate baudrate"},
	{ "?", cmd_help, "Synonym for `help'" },
	{ NULL, NULL, NULL }
};

int cmd_init(RobotInterface* robot, Qemu* qemu, void (*f)(void))
{
	cmd_robot = robot;
	cmd_qemu = qemu;
	cmd_exit_callback = f;
	return cli_init(usb_commands);
}

int cmd_dynamixel_scan(const char* arg)
{
	int type;
	int count = sscanf(arg, "%d", &type);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot->dynamixel_scan(type);
	return CMD_SUCESS;
}

int cmd_dynamixel_set_id(const char* arg)
{
	unsigned int id;
	unsigned int new_id;
	int type;
	int count = sscanf(arg, "%d %u %u", &type, &id, &new_id);

	if(count != 3)
	{
		return CMD_ERROR;
	}

	cmd_robot->dynamixel_set_id(type, id, new_id);
	return CMD_SUCESS;
}

int cmd_dynamixel_set_goal_position(const char* arg)
{
	int type;
	unsigned int id;
	float alpha;
	int count = sscanf(arg, "%d %u %f", &type, &id, &alpha);

	if(count != 3)
	{
		return CMD_ERROR;
	}

	cmd_robot->dynamixel_set_goal_position(type, id, alpha);
	return CMD_SUCESS;
}

int cmd_dynamixel_set_op_baudrate(const char* arg)
{
	int type;
	unsigned int id;
	int count = sscanf(arg, "%d %u", &type, &id);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	cmd_robot->dynamixel_set_op_baudrate(type, id);
	return CMD_SUCESS;
}

int cmd_dynamixel_set_manager_baudrate(const char* arg)
{
	int type;
	unsigned int freq;
	int count = sscanf(arg, "%d %u", &type, &freq);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	cmd_robot->dynamixel_set_manager_baudrate(type, freq);
	return CMD_SUCESS;
}

int cmd_dynamixel_set_max_torque(const char* arg)
{
	int type;
	unsigned int id;
	float torque;
	int count = sscanf(arg, "%d %u %f", &type, &id, &torque);

	if(count != 3)
	{
		return CMD_ERROR;
	}

	cmd_robot->dynamixel_set_max_torque(type, id, torque);
	return CMD_SUCESS;
}

int cmd_dynamixel_set_target_reached_threshold(const char* arg)
{
	int type;
	unsigned int id;
	float threshold;
	int count = sscanf(arg, "%d %u %f", &type, &id, &threshold);

	if(count != 3)
	{
		return CMD_ERROR;
	}

	cmd_robot->dynamixel_set_target_reached_threshold(type, id, threshold);
	return CMD_SUCESS;
}

int cmd_dynamixel_enable_endless_turn_mode(const char* arg)
{
	int type;
	unsigned int id;

	int count = sscanf(arg, "%d %u", &type, &id);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	cmd_robot->dynamixel_enable_endless_turn_mode(type, id);
	return CMD_SUCESS;
}

int cmd_dynamixel_disable_endless_turn_mode(const char* arg)
{
	int type;
	unsigned int id;

	int count = sscanf(arg, "%d %u", &type, &id);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	cmd_robot->dynamixel_disable_endless_turn_mode(type, id);
	return CMD_SUCESS;
}

int cmd_dynamixel_set_speed(const char* arg)
{
	int type;
	unsigned int id;
	float speed;

	int count = sscanf(arg, "%d %u %f", &type, &id, &speed);

	if(count != 3)
	{
		return CMD_ERROR;
	}

	cmd_robot->dynamixel_set_speed(type, id, speed);
	return CMD_SUCESS;
}

int cmd_dynamixel_get_position(const char* arg)
{
	int type;
	unsigned int id;
	int count = sscanf(arg, "%d %u", &type, &id);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	cmd_robot->dynamixel_get_position(type, id);
	return CMD_SUCESS;
}

int cmd_can_set_baudrate(const char* arg)
{
	unsigned int id;
	int debug;
	int count = sscanf(arg, "%u %d", &id, &debug);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	cmd_robot->can_set_baudrate((can_baudrate)id, debug);
	return CMD_SUCESS;
}

int cmd_can_write(const char* arg)
{
	struct can_msg msg;
	int id;
	int size;
	int data[8];
	int i = 0;
	int count = sscanf(arg, "%x %d %x %x %x %x %x %x %x %x", &id, &size, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6], &data[7]);

	if(count < 2 || count != size + 2 || size < 0 || size > 8)
	{
		return CMD_ERROR;
	}

	msg.id = id;
	msg.size = size;
	for(i = 0; i < size; i++)
	{
		msg.data[i] = data[i];
	}
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;

	cmd_robot->can_write(&msg);
	return CMD_SUCESS;
}

int cmd_can_lss(const char* arg)
{
	int on;
	int count = sscanf(arg, "%d", &on);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot->can_lss(on);
	return CMD_SUCESS;
}

int cmd_can_lss_set_nodeid(const char* arg)
{
	int id;
	int count = sscanf(arg, "%x", &id);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot->can_lss_set_nodeid(id);
	return CMD_SUCESS;
}

int cmd_can_lss_save(const char* arg)
{
	(void) arg;
	cmd_robot->can_lss_save();
	return CMD_SUCESS;
}

int cmd_heartbeat_disable(const char* arg)
{
	(void) arg;
	cmd_robot->heartbeat_disable();
	return CMD_SUCESS;
}

int cmd_heartbeat_update(const char* arg)
{
	(void) arg;
	cmd_robot->heartbeat_update();
	return CMD_SUCESS;
}

int cmd_help(const char* arg)
{
	(void) arg;
	log_info("Aide\n");
	for(int i = 0; i < (int)(sizeof(usb_commands) / sizeof(usb_commands[0])) - 1; i++)
	{
		log_info("%s : %s", usb_commands[i].name, usb_commands[i].doc);
	}

	return CMD_SUCESS;
}

int cmd_qemu_set_clock_factor(const char* arg)
{
	unsigned int system_clock_factor;
	unsigned int icount =0;
	int count = sscanf(arg, "%u %u", &system_clock_factor, &icount);

	if(count != 1 && count != 2)
	{
		return CMD_ERROR;
	}

	if( cmd_qemu )
	{
		cmd_qemu->set_clock_factor(system_clock_factor, icount);
	}

	return CMD_SUCESS;
}

int cmd_qemu_set_io(const char* arg)
{
	unsigned int id;
	unsigned int val =0;
	int count = sscanf(arg, "%u %u", &id, &val);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	if( cmd_qemu )
	{
		cmd_qemu->set_io(id, val?1:0);
	}

	return CMD_SUCESS;
}

int cmd_qemu_manage_canopen_connexion(const char* arg)
{
	unsigned int nodeid;
	int connected;
	int count = sscanf(arg, "%u %u", &nodeid, &connected);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	if( cmd_qemu )
	{
		cmd_qemu->manage_canopen_connexion(nodeid, connected);
	}

	return CMD_SUCESS;
}

int cmd_quit(const char* arg)
{
	(void) arg;
	if(cmd_exit_callback)
	{
		cmd_exit_callback();
	}
	// pas de log_info pour ne pas remettre le prompt
	cli_log("Quit\n");
	return CMD_QUIT;
}

int cmd_control_param(const char* arg)
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

	cmd_robot->control_set_param(kp_av, ki_av, kd_av, kp_rot, ki_rot, kd_rot, kx, ky, kalpha);

	return CMD_SUCESS;
}

int cmd_control_print_param(const char* arg)
{
	(void) arg;
	cmd_robot->control_print_param();

	return CMD_SUCESS;
}

int cmd_localization_set_position(const char* arg)
{
	VectPlan pos;
	int count = sscanf(arg, "%f %f %f", &pos.x, &pos.y, &pos.theta);

	if(count != 3)
	{
		return CMD_ERROR;
	}

	cmd_robot->set_position(pos);

	return CMD_SUCESS;
}

int cmd_straight(const char* arg)
{
	float dist;
	int count = sscanf(arg, "%f", &dist);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot->straight(dist);

	return CMD_SUCESS;
}

int cmd_straight_to_wall(const char* arg)
{
	(void) arg;
	cmd_robot->straight_to_wall();

	return CMD_SUCESS;
}

int cmd_rotate(const char* arg)
{
	float alpha;
	int count = sscanf(arg, "%f", &alpha);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot->rotate(alpha);

	return CMD_SUCESS;
}

int cmd_rotate_to(const char* arg)
{
	float alpha;
	int count = sscanf(arg, "%f", &alpha);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot->rotate_to(alpha);

	return CMD_SUCESS;
}

int cmd_motion_enable(const char* arg)
{
	int enable;
	int count = sscanf(arg, "%d", &enable);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot->motion_enable(enable);

	return CMD_SUCESS;
}

int cmd_motion_set_max_driving_current(const char* arg)
{
	float maxCurrent;
	int count = sscanf(arg, "%f", &maxCurrent);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot->motion_set_max_driving_current(maxCurrent);

	return CMD_SUCESS;
}

int cmd_goto(const char* arg)
{
	VectPlan dest;
	float dist = 0;
	unsigned int way = WAY_ANY;
	unsigned int avoidance_type = AVOIDANCE_STOP;
	int count = sscanf(arg, "%f %f %f %f %u %u", &dest.x, &dest.y, &dest.theta, &dist, &way, &avoidance_type);

	if(count < 3 || count  > 6 )
	{
		return CMD_ERROR;
	}

	cmd_robot->goto_near(dest, dist, way, avoidance_type);

	return CMD_SUCESS;
}

int cmd_goto_xy(const char* arg)
{
	float x;
	float y;
	float dist = 0;
	unsigned int way = WAY_ANY;
	unsigned int avoidance_type = AVOIDANCE_STOP;
	int count = sscanf(arg, "%f %f %f %u %u", &x, &y, &dist, &way, &avoidance_type);

	if(count > 5 || count < 2)
	{
		return CMD_ERROR;
	}

	cmd_robot->goto_near_xy(x, y, dist, way, avoidance_type);

	return CMD_SUCESS;
}

int cmd_motion_goto(const char* arg)
{
	VectPlan dest;
	VectPlan cp;
	int way = WAY_ANY;
	int type = MOTION_AXIS_XYA;

	int count = sscanf(arg, "%f %f %f %d %d %f %f %f", &dest.x, &dest.y, &dest.theta, &way, &type, &cp.x, &cp.y, &cp.theta);

	if(count != 8 && count != 5 && count != 4 && count != 3)
	{
		return CMD_ERROR;
	}

	KinematicsParameters linearParam = {1000, 1000, 1000}; // TODO
	KinematicsParameters angularParam = {M_PI, 2*M_PI, 2*M_PI}; // TODO

	cmd_robot->motion_goto(dest, cp, (enum motion_way)way, (enum motion_trajectory_type)type, linearParam, angularParam);
	return CMD_SUCESS;
}

int cmd_goto_graph(const char* arg)
{
	(void) arg;
	cmd_robot->goto_graph();

	return CMD_SUCESS;
}

int cmd_gyro_calib_start(const char* arg)
{
	(void) arg;
	cmd_robot->gyro_calibration(GYRO_CALIBRATION_START);
	return CMD_SUCESS;
}

int cmd_gyro_calib_stop(const char* arg)
{
	(void) arg;
	cmd_robot->gyro_calibration(GYRO_CALIBRATION_STOP);
	return CMD_SUCESS;
}

int cmd_gyro_set_theta(const char* arg)
{
	float theta = 0;
	int count = sscanf(arg, "%f", &theta);

	if(count != 1)
	{
		return CMD_ERROR;
	}
	cmd_robot->gyro_set_position(theta);
	return CMD_SUCESS;
}


int cmd_gyro_set_calibration_values(const char* arg)
{
	float scale;
	float bias;
	float dead_zone;
	int count = sscanf(arg, "%f %f %f", &scale, &bias, &dead_zone);

	if(count != 3)
	{
		return CMD_ERROR;
	}
	cmd_robot->gyro_set_calibration_values(scale, bias, dead_zone);
	return CMD_SUCESS;
}

int cmd_max_speed(const char* arg)
{
	float v_max_av;
	float v_max_rot;
	int count = sscanf(arg, "%f %f", &v_max_av, &v_max_rot);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	cmd_robot->set_max_speed(v_max_av, v_max_rot);

	return CMD_SUCESS;
}

int cmd_pince_set_position (const char* arg)
{
	int pince_left;
	int pince_right;
	int count = sscanf(arg, "%d %d", &pince_left, &pince_right);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	cmd_robot->pince((pince_cmd_type)pince_left, (pince_cmd_type)pince_right);
	return CMD_SUCESS;
}

int cmd_ptask(const char*)
{
	cmd_robot->ptask();
	return CMD_SUCESS;
}

int cmd_power_off(const char* arg)
{
	int val;
	int count = sscanf(arg, "%d", &val);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot->power_off(val != 0);
	return CMD_SUCESS;
}

int cmd_pwm_set(const char* arg)
{
	int id;
	float val;
	int count = sscanf(arg, "%d %f", &id, &val);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	cmd_robot->pwm_set(id, val);
	return CMD_SUCESS;
}

int cmd_pump(const char* arg)
{
	int id;
	int val;
	int count = sscanf(arg, "%d %d", &id, &val);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	cmd_robot->pump(id, val);
	return CMD_SUCESS;
}

int cmd_reboot(const char* )
{
	cmd_robot->reboot();
	return CMD_SUCESS;
}

int cmd_recalage(const char*)
{
	cmd_robot->recalage();
	return CMD_SUCESS;
}

int cmd_go(const char*)
{
	cmd_robot->go();
	return CMD_SUCESS;
}

int cmd_go_enable(const char*)
{
	cmd_robot->go_enable();
	return CMD_SUCESS;
}

int cmd_set_match_time(const char* arg)
{
	int time;

	int count = sscanf(arg, "%d", &time);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot->set_match_time(time);
	return CMD_SUCESS;
}

int cmd_set_color(const char* arg)
{
	int color;

	int count = sscanf(arg, "%d", &color);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot->color((uint8_t) color);
	return CMD_SUCESS;
}

int cmd_motion_set_speed(const char* arg)
{
	VectPlan u;
	float v;

	int count = sscanf(arg, "%f %f %f %f", &u.x, &u.y, &u.theta, &v);

	if(count != 4)
	{
		return CMD_ERROR;
	}

	cmd_robot->motion_set_speed(u, v);
	return CMD_SUCESS;
}

int cmd_motion_set_actuator_kinematics(const char* arg)
{
	struct motion_cmd_set_actuator_kinematics_arg cmd;

	int count = sscanf(arg, "%d %f %d %f",
			&cmd.mode[0], &cmd.val[0],
			&cmd.mode[1], &cmd.val[1]);

	if(count != 4)
	{
		return CMD_ERROR;
	}

	cmd_robot->motion_set_actuator_kinematics(cmd);
	return CMD_SUCESS;
}

int cmd_arm_abz(const char* arg)
{
	float z;
	float a;
	float b;

	int count = sscanf(arg, "%f %f %f", &a, &b, &z);

	if(count != 3)
	{
		return CMD_ERROR;
	}

	cmd_robot->arm_abz(a, b, z);
	return CMD_SUCESS;
}

int cmd_arm_cmd(const char* arg)
{
	int cmdType;

	int count = sscanf(arg, "%d", &cmdType);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot->arm_cmd(cmdType);
	return CMD_SUCESS;
}

int cmd_arm_xyz(const char* arg)
{
	float x;
	float y;
	float z;
	int type;

	int count = sscanf(arg, "%f %f %f %d", &x, &y, &z, &type);

	if(count != 4)
	{
		return CMD_ERROR;
	}

	cmd_robot->arm_xyz(x, y, z, (arm_cmd_type)type);
	return CMD_SUCESS;
}

int cmd_arm_ventouse(const char* arg)
{
	float x1;
	float y1;
	float x2;
	float y2;
	float z;
	int tool_way;

	int count = sscanf(arg, "%f %f %f %f %f %d", &x1, &y1, &x2, &y2, &z, &tool_way);

	if(count != 6)
	{
		return CMD_ERROR;
	}

	cmd_robot->arm_ventouse(x1, y1, x2, y2, z, tool_way);
	return CMD_SUCESS;
}

int cmd_xbee_set_op_baudrate(const char* /*arg*/)
{
	cmd_robot->xbee_set_op_baudrate();
	return CMD_SUCESS;
}

int cmd_xbee_set_manager_baudrate(const char* arg)
{
	int baudrate;

	int count = sscanf(arg, "%d", &baudrate);

	if(count != 1)
	{
		return CMD_ERROR;
	}
	cmd_robot->xbee_set_manager_baudrate(baudrate);
	return CMD_SUCESS;
}
