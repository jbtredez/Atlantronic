#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <pthread.h>
#include "linux/tools/com.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/driver/can.h"
#include "kernel/error_codes.h"
#include "kernel/fault.h"
#include "kernel/math/polyline.h"
#include "discovery/control.h"
#include "foo/pince.h"
#include "foo/arm.h"

#define CONTROL_USB_DATA_MAX        120000 //!< 600s (10 mn) de données avec l'asservissement à 200Hz

enum
{
	COM_FOO,
	COM_BAR,
	COM_MAX
};

struct robot_interface
{
	pthread_t tid;
	volatile int stop_task;
	struct com com[COM_MAX]; //!< communication
	pthread_mutex_t mutex; //!< mutex de protection des donnees ci-dessous
	void (*callback)(void*);
	void* callback_arg;

	// données brutes
	struct hokuyo_scan hokuyo_scan[HOKUYO_MAX];
	struct control_usb_data control_usb_data[CONTROL_USB_DATA_MAX];
	int control_usb_data_count;
	struct fault_status fault_status[COM_MAX][FAULT_MAX];

	double current_time;
	double start_time;

	// tmp (en cours de mise à jour)
	int detection_dynamic_object_id;
	int detection_dynamic_object_pt_tmp_size;
	int16_t detection_dynamic_object_size_tmp;
	struct vect2 detection_dynamic_object_pt_tmp[HOKUYO_NUM_POINTS];
	struct polyline detection_dynamic_obj_tmp[HOKUYO_NUM_POINTS];

	int16_t detection_dynamic_object_size;
	struct vect2 detection_dynamic_object_pt[HOKUYO_NUM_POINTS];
	struct polyline detection_dynamic_obj[HOKUYO_NUM_POINTS];

	// calculs
	struct vect2 detection_hokuyo_pos[HOKUYO_NUM_POINTS*HOKUYO_MAX];
	struct vect2 detection_hokuyo_reg[HOKUYO_NUM_POINTS*HOKUYO_MAX]; // TODO à virer
	int detection_reg_num[HOKUYO_MAX];
};

int robot_interface_init(struct robot_interface* data, const char* file_foo_read, const char* file_foo_write, const char* file_bar_read, const char* file_bar_write, void (*callback)(void*), void* callback_arg);

void robot_interface_destroy(struct robot_interface* data);

// ---------- gestion des ax12 -------------------------------------------------

int robot_interface_ax12_scan(struct robot_interface* data);

int robot_interface_ax12_set_id(struct robot_interface* data, uint8_t id, uint8_t new_id);

int robot_interface_ax12_set_goal_position(struct robot_interface* data, uint8_t id, float alpha);

int robot_interface_ax12_get_position(struct robot_interface* data, uint8_t id);

// ---------- gestion des pinces -----------------------------------------------

int robot_interface_pince(struct robot_interface* data, enum pince_cmd_type cmd_type_left, enum pince_cmd_type cmd_type_right);

// ---------- gestion du bras --------------------------------------------------

int robot_interface_arm_xyz(struct robot_interface* data, float x, float y, float z, enum arm_cmd_type type);

int robot_interface_arm_abz(struct robot_interface* data, float a, float b, float z);

int robot_interface_arm_ventouse(struct robot_interface* data, float x1, float y1, float x2, float y2, float z, int8_t tool_way);

int robot_interface_arm_hook(struct robot_interface* data, float x1, float y1, float x2, float y2, float z, int8_t tool_way);

int robot_interface_arm_bridge(struct robot_interface* data, uint8_t on);

// ---------- gestion recalage, go, couleur... ---------------------------------

int robot_interface_recalage(struct robot_interface* data);

int robot_interface_go(struct robot_interface* data);

int robot_interface_color(struct robot_interface* data, uint8_t color);

//!< time : temps en ms
int robot_interface_set_match_time(struct robot_interface* data, uint32_t time);

// ---------- localisation -----------------------------------------------------

int robot_interface_set_position(struct robot_interface* data, VectPlan pos);

// ---------- gestion control --------------------------------------------------

int robot_interface_control_print_param(struct robot_interface* data);

int robot_interface_control_set_param(struct robot_interface* data, int kp_av, int ki_av, int kd_av, int kp_rot, int ki_rot, int kd_rot, int kx, int ky, int kalpha);

int robot_interface_control_goto(struct robot_interface* data, VectPlan dest, VectPlan cp, KinematicsParameters linearParam, KinematicsParameters angularParam);

// ---------- gestion trajectoire ----------------------------------------------

int robot_interface_straight(struct robot_interface* data, float dist);

int robot_interface_straight_to_wall(struct robot_interface* data);

int robot_interface_rotate(struct robot_interface* data, float alpha);

int robot_interface_rotate_to(struct robot_interface* data, float alpha);

int robot_interface_free(struct robot_interface* data);

int robot_interface_goto_graph(struct robot_interface* data);

int robot_interface_goto_near_xy(struct robot_interface* data, float x, float y, float dist, unsigned int way, unsigned int avoidance_type);

int robot_interface_goto_near(struct robot_interface* data, float x, float y, float alpha, float dist, unsigned int way, unsigned int avoidance_type);

//!< vitesse en % de vmax de configuration du robot
int robot_interface_set_max_speed(struct robot_interface* data, float vmax_av, float vmax_rot);

int robot_interface_straight_speed(struct robot_interface* data, float v);

int robot_interface_rotate_speed(struct robot_interface* data, float v);

// ---------- gestion CAN ------------------------------------------------------

int robot_interface_can_set_baudrate(struct robot_interface* data, enum can_baudrate baudrate, int debug);

int robot_interface_can_write(struct robot_interface* data, struct can_msg* msg);

#endif
