#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "linux/usb_interface_common/com/com.h"
#include "linux/usb_interface_common/cmd.h"
#include "linux/usb_interface_common/cli.h"
#include "kernel/driver/usb.h"

#define MAX_ROBOTS        4

static void (*cmd_exit_callback)(void) = NULL;
static RobotInterface* cmd_robot[MAX_ROBOTS];
static Qemu* cmd_qemu[MAX_ROBOTS];
static int cmd_robots_count = 0;
static int cmd_robots_current_id = 0;

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
int cmd_esc_set(const char* arg);
int cmd_free(const char* arg);
int cmd_odo_wheel_radius(const char* arg);
int cmd_odo_voie(const char* arg);
int cmd_power_off(const char* arg);
int cmd_pwm_set(const char* arg);
int cmd_pump(const char* arg);
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
int cmd_help(const char* arg);
int cmd_localization_set_position(const char* arg);
int cmd_max_speed(const char* arg);
int cmd_motion_enable(const char* arg);
int cmd_motion_set_param(const char* arg);
int cmd_motion_print_param(const char* arg);
int cmd_motion_set_max_driving_current(const char* arg);
int cmd_motion_set_actuator_kinematics(const char* arg);
int cmd_motion_set_speed(const char* arg);
int cmd_ptask(const char* arg);
int cmd_reboot(const char* arg);
int cmd_recalage(const char* arg);
int cmd_select_robot(const char* arg);
int cmd_rotate(const char* arg);
int cmd_rotate_to(const char* arg);
int cmd_rotate_cylinder_to(const char* arg);
int cmd_set_color(const char* arg);
int cmd_set_match_time(const char* arg);
int cmd_set_motors_pid(const char* arg);
int cmd_straight(const char* arg);
int cmd_wing_set_position(const char* arg);
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
	{ "esc_set", cmd_esc_set, "esc_set val"},
	{ "free", cmd_free, "free"},
	{ "set_match_time", cmd_set_match_time, "set match time"},
	{ "go", cmd_go, "go" },
	{ "go_enable", cmd_go_enable, "go_enable" },
	{ "goto_graph", cmd_goto_graph, "goto_graph node" },
	{ "goto", cmd_goto, "goto x y alpha dist way avoidance_type" },
	{ "goto_xy", cmd_goto_xy, "goto_xy x y dist way avoidance_type"},
	{ "gyro_calib_start", cmd_gyro_calib_start, "cmd_gyro_calib_start"},
	{ "gyro_calib_stop", cmd_gyro_calib_stop, "cmd_gyro_calib_stop"},
	{ "gyro_set_theta", cmd_gyro_set_theta, "cmd_gyro_set_theta theta"},
	{ "gyro_set_calibration_values", cmd_gyro_set_calibration_values, "cmd_gyro_set_calibration_values scale bias dead_zone"},
	{ "help", cmd_help, "Display this text" },
	{ "localization_set_position", cmd_localization_set_position, "set robot position : localization_set_position x y alpha"},
	{ "max_speed", cmd_max_speed, "vitesse max en % (av, rot) : max_speed v_max_av v_max_rot" },
	{ "motion_enable", cmd_motion_enable, "motion_enable enable" },
	{ "motion_set_param", cmd_motion_set_param, "motion_set_param kp_x ki_x kd_x kp_y ki_y kd_y kp_theta ki_theta kd_theta" },
	{ "motion_print_param", cmd_motion_print_param, "control_print_param"},
	{ "motion_set_max_driving_current", cmd_motion_set_max_driving_current, "motion_set_max_driving_current val"},
	{ "motion_set_actuator_kinematics", cmd_motion_set_actuator_kinematics, "set actuators kinematics mode val (x6)"},
	{ "motion_set_speed", cmd_motion_set_speed, "set speed direction valeur"},
	{ "odo_wheel_radius", cmd_odo_wheel_radius, "radius1 radius2"},
	{ "odo_voie", cmd_odo_voie, "voie"},
	{ "ptask", cmd_ptask, "print tasks"},
	{ "power_off", cmd_power_off, "power off {0,1}"},
	{ "pwm_set", cmd_pwm_set, "pwm_set id val"},
	{ "pump", cmd_pump, "pump id val[0 100]"},
	{ "q", cmd_quit, "Quit" },
	{ "qemu_set_io", cmd_qemu_set_io, "qemu_set_io id val" },
	{ "qemu_manage_canopen_connexion", cmd_qemu_manage_canopen_connexion, "qemu connect canopen node : cmd_qemu_manage_canopen_connexion nodeid connected" },
	{ "quit", cmd_quit, "Quit" },
	{ "rotate", cmd_rotate, "rotate angle" },
	{ "rotate_to", cmd_rotate_to, "rotate_to angle" },
	{ "rotate_cylinder_to", cmd_rotate_cylinder_to, "rotate_cylinder_to angle" },
	{ "reboot", cmd_reboot, "reboot" },
	{ "recalage", cmd_recalage, "recalage"},
	{ "select_robot", cmd_select_robot, "select_robot id"},
	{ "set_color", cmd_set_color, "set color"},
	{ "set_motors_pid", cmd_set_motors_pid, "set motors pid kp ki kd"},
	{ "straight", cmd_straight, "straight dist" },
	{ "xbee_set_op_baudrate", cmd_xbee_set_op_baudrate, "xbee_set_op_baudrate"},
	{ "xbee_set_manager_baudrate", cmd_xbee_set_manager_baudrate, "xbee_set_manager_baudrate baudrate"},
	{ "?", cmd_help, "Synonym for `help'" },
	{ NULL, NULL, NULL }
};

int cmd_init(void (*exit_cb)(void))
{
	cmd_exit_callback = exit_cb;
	return 0;
}

int cmd_add_robot(RobotInterface* robot, Qemu* qemu)
{
	if( cmd_robots_count >= MAX_ROBOTS )
	{
		log_error("cmd_add_robot failed : too many robots");
		return -1;
	}

	log_info("new robot added id %d : %s", cmd_robots_count, robot->getName());
	cmd_robot[cmd_robots_count] = robot;
	cmd_qemu[cmd_robots_count] = qemu;
	cmd_robots_count++;
	if( cmd_robots_count == 1 )
	{
		log_info("new robot selected id %d : %s. Use select_robot [id] to change robot control", cmd_robots_count, robot->getName());
		cli_set_prompt(robot->getName(), 0);
		return cli_init(usb_commands);
	}
	return 0;
}

int cmd_get_selected_robot()
{
	return cmd_robots_current_id;
}

int cmd_select_robot(int id)
{
	if( id < 0)
	{
		log_error("%s: robot %d not found (max %d)", __FUNCTION__, id, cmd_robots_count-1);
		return CMD_ERROR;
	}

	if( id >= cmd_robots_count )
	{
		log_error("%s: robot %d not found (max %d)", __FUNCTION__, id, cmd_robots_count-1);
		return CMD_ERROR;
	}

	cmd_robots_current_id = id;
	log_info("new robot selected id %d : %s", cmd_robots_count, cmd_robot[cmd_robots_current_id]->getName());
	cli_set_prompt(cmd_robot[cmd_robots_current_id]->getName(), cmd_robots_current_id);

	return 0;
}

int cmd_select_robot(const char* arg)
{
	int id = 0;
	int count = sscanf(arg, "%d", &id);

	if(count != 1 )
	{
		return CMD_ERROR;
	}
	return cmd_select_robot(id);
}

int cmd_dynamixel_scan(const char* arg)
{
	int type;
	int count = sscanf(arg, "%d", &type);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->dynamixel_scan(type);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->dynamixel_set_id(type, id, new_id);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->dynamixel_set_goal_position(type, id, alpha);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->dynamixel_set_op_baudrate(type, id);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->dynamixel_set_manager_baudrate(type, freq);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->dynamixel_set_max_torque(type, id, torque);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->dynamixel_set_target_reached_threshold(type, id, threshold);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->dynamixel_enable_endless_turn_mode(type, id);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->dynamixel_disable_endless_turn_mode(type, id);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->dynamixel_set_speed(type, id, speed);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->dynamixel_get_position(type, id);
	return CMD_SUCCESS;
}

int cmd_esc_set(const char* arg)
{
	float val;
	int count = sscanf(arg, "%f", &val);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->esc_set(val);

	return CMD_SUCCESS;
}

int cmd_free(const char* /*arg*/)
{
	cmd_robot[cmd_robots_current_id]->free();
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->can_set_baudrate((can_baudrate)id, debug);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->can_write(&msg);
	return CMD_SUCCESS;
}

int cmd_can_lss(const char* arg)
{
	int on;
	int count = sscanf(arg, "%d", &on);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->can_lss(on);
	return CMD_SUCCESS;
}

int cmd_can_lss_set_nodeid(const char* arg)
{
	int id;
	int count = sscanf(arg, "%x", &id);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->can_lss_set_nodeid(id);
	return CMD_SUCCESS;
}

int cmd_can_lss_save(const char* arg)
{
	(void) arg;
	cmd_robot[cmd_robots_current_id]->can_lss_save();
	return CMD_SUCCESS;
}

int cmd_help(const char* arg)
{
	(void) arg;
	log_info("Aide\n");
	for(int i = 0; i < (int)(sizeof(usb_commands) / sizeof(usb_commands[0])) - 1; i++)
	{
		log_info("%s : %s", usb_commands[i].name, usb_commands[i].doc);
	}

	return CMD_SUCCESS;
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

	if( cmd_qemu[cmd_robots_current_id] )
	{
		cmd_qemu[cmd_robots_current_id]->setIo(id, val?1:0);
	}

	return CMD_SUCCESS;
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

	if( cmd_qemu[cmd_robots_current_id] )
	{
		cmd_qemu[cmd_robots_current_id]->manage_canopen_connexion(nodeid, connected);
	}

	return CMD_SUCCESS;
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

int cmd_motion_set_param(const char* arg)
{
	float kp_x;
	float ki_x;
	float kd_x;
	float kp_y;
	float ki_y;
	float kd_y;
	float kp_theta;
	float ki_theta;
	float kd_theta;

	int count = sscanf(arg, "%f %f %f %f %f %f %f %f %f", &kp_x, &ki_x, &kd_x, &kp_y, &ki_y, &kd_y, &kp_theta, &ki_theta, &kd_theta);

	if(count != 9)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->motion_set_param(kp_x, ki_x, kd_x, kp_y, ki_y, kd_y, kp_theta, ki_theta, kd_theta);

	return CMD_SUCCESS;
}

int cmd_motion_print_param(const char* arg)
{
	(void) arg;
	cmd_robot[cmd_robots_current_id]->motion_print_param();

	return CMD_SUCCESS;
}

int cmd_localization_set_position(const char* arg)
{
	VectPlan pos;
	int count = sscanf(arg, "%f %f %f", &pos.x, &pos.y, &pos.theta);

	if(count != 3)
	{
		return CMD_ERROR;
	}

	if( cmd_qemu[cmd_robots_current_id] )
	{
		cmd_qemu[cmd_robots_current_id]->setPosition(pos);
	}

	cmd_robot[cmd_robots_current_id]->set_position(pos);

	return CMD_SUCCESS;
}

int cmd_straight(const char* arg)
{
	float dist;
	int count = sscanf(arg, "%f", &dist);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->straight(dist);

	return CMD_SUCCESS;
}

int cmd_rotate(const char* arg)
{
	float alpha;
	int count = sscanf(arg, "%f", &alpha);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->rotate(alpha);

	return CMD_SUCCESS;
}

int cmd_rotate_to(const char* arg)
{
	float alpha;
	int count = sscanf(arg, "%f", &alpha);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->rotate_to(alpha);

	return CMD_SUCCESS;
}

int cmd_rotate_cylinder_to(const char* arg)
{
	float alpha;
	int count = sscanf(arg, "%f", &alpha);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->rotate_cylinder_to(alpha);

	return CMD_SUCCESS;
}

int cmd_motion_enable(const char* arg)
{
	int enable;
	int count = sscanf(arg, "%d", &enable);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->motion_enable(enable);

	return CMD_SUCCESS;
}

int cmd_motion_set_max_driving_current(const char* arg)
{
	float maxCurrent;
	int count = sscanf(arg, "%f", &maxCurrent);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->motion_set_max_driving_current(maxCurrent);

	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->goto_near(dest, dist, way, avoidance_type);

	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->goto_near_xy(x, y, dist, way, avoidance_type);

	return CMD_SUCCESS;
}

int cmd_goto_graph(const char* arg)
{
	(void) arg;
	int node = -1;

	int count = sscanf(arg, "%d", &node);
	if(count != 1)
	{
		node = -1;
	}

	if( node < 0 )
	{
		cmd_robot[cmd_robots_current_id]->goto_graph();
	}
	else
	{
		cmd_robot[cmd_robots_current_id]->goto_graph_node(node);
	}

	return CMD_SUCCESS;
}

int cmd_gyro_calib_start(const char* arg)
{
	(void) arg;
	cmd_robot[cmd_robots_current_id]->gyro_calibration(GYRO_CALIBRATION_START);
	return CMD_SUCCESS;
}

int cmd_gyro_calib_stop(const char* arg)
{
	(void) arg;
	cmd_robot[cmd_robots_current_id]->gyro_calibration(GYRO_CALIBRATION_STOP);
	return CMD_SUCCESS;
}

int cmd_gyro_set_theta(const char* arg)
{
	float theta = 0;
	int count = sscanf(arg, "%f", &theta);

	if(count != 1)
	{
		return CMD_ERROR;
	}
	cmd_robot[cmd_robots_current_id]->gyro_set_position(theta);
	return CMD_SUCCESS;
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
	cmd_robot[cmd_robots_current_id]->gyro_set_calibration_values(scale, bias, dead_zone);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->set_max_speed(v_max_av, v_max_rot);

	return CMD_SUCCESS;
}


int cmd_ptask(const char*)
{
	cmd_robot[cmd_robots_current_id]->ptask();
	return CMD_SUCCESS;
}

int cmd_odo_wheel_radius(const char* arg)
{
	float r1, r2;
	int count = sscanf(arg, "%f %f", &r1, &r2);

	if(count != 2)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->set_odo_wheel_radius(r1, r2);
	return CMD_SUCCESS;
}

int cmd_odo_voie(const char* arg)
{
	float val1, val2;
	int count = sscanf(arg, "%f %f", &val1,&val2);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->set_odo_voie(val1,val2);
	return CMD_SUCCESS;
}

int cmd_power_off(const char* arg)
{
	int val;
	int count = sscanf(arg, "%d", &val);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->power_off(val != 0);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->pwm_set(id, val);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->pump(id, val);
	return CMD_SUCCESS;
}

int cmd_reboot(const char* )
{
	cmd_robot[cmd_robots_current_id]->reboot();
	return CMD_SUCCESS;
}

int cmd_recalage(const char*)
{
	cmd_robot[cmd_robots_current_id]->recalage();
	return CMD_SUCCESS;
}

int cmd_go(const char*)
{
	cmd_robot[cmd_robots_current_id]->go();
	return CMD_SUCCESS;
}

int cmd_go_enable(const char*)
{
	cmd_robot[cmd_robots_current_id]->go_enable();
	return CMD_SUCCESS;
}

int cmd_set_match_time(const char* arg)
{
	int time;

	int count = sscanf(arg, "%d", &time);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->set_match_time(time);
	return CMD_SUCCESS;
}

int cmd_set_color(const char* arg)
{
	int color;

	int count = sscanf(arg, "%d", &color);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->color((uint8_t) color);
	return CMD_SUCCESS;
}

int cmd_set_motors_pid(const char* arg)
{
	float kp;
	float ki;
	float kd;

	int count = sscanf(arg, "%f %f %f", &kp, &ki, &kd);

	if(count != 3)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->set_motors_pid(kp, ki, kd, kp, ki, kd);

	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->motion_set_speed(u, v);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->motion_set_actuator_kinematics(cmd);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->arm_abz(a, b, z);
	return CMD_SUCCESS;
}

int cmd_arm_cmd(const char* arg)
{
	int cmdType;

	int count = sscanf(arg, "%d", &cmdType);

	if(count != 1)
	{
		return CMD_ERROR;
	}

	cmd_robot[cmd_robots_current_id]->arm_cmd(cmdType);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->arm_xyz(x, y, z, (arm_cmd_type)type);
	return CMD_SUCCESS;
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

	cmd_robot[cmd_robots_current_id]->arm_ventouse(x1, y1, x2, y2, z, tool_way);
	return CMD_SUCCESS;
}

int cmd_xbee_set_op_baudrate(const char* /*arg*/)
{
	cmd_robot[cmd_robots_current_id]->xbee_set_op_baudrate();
	return CMD_SUCCESS;
}

int cmd_xbee_set_manager_baudrate(const char* arg)
{
	int baudrate;

	int count = sscanf(arg, "%d", &baudrate);

	if(count != 1)
	{
		return CMD_ERROR;
	}
	cmd_robot[cmd_robots_current_id]->xbee_set_manager_baudrate(baudrate);
	return CMD_SUCCESS;
}
