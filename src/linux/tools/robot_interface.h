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
	COM_MAX
};

class RobotInterface
{
	public:
		int init(const char* name, const char* file_read, const char* file_write, void (*callback)(void*), void* callback_arg);
		void destroy();

		// ---------- gestion des ax12 -------------------------------------------------
		int dynamixel_scan(int dynamixel_type);
		int dynamixel_set_id(int dynamixel_type, uint8_t id, uint8_t new_id);
		int dynamixel_set_goal_position(int dynamixel_type, uint8_t id, float alpha);
		int dynamixel_get_position(int dynamixel_type, uint8_t id);

		// ---------- gestion CAN ------------------------------------------------------
		int can_set_baudrate(enum can_baudrate baudrate, int debug);
		int can_write(struct can_msg* msg);

		// ---------- gestion recalage, go, couleur... ---------------------------------
		int recalage();
		int go();
		int color(uint8_t color);
		//!< time : temps en ms
		int set_match_time(uint32_t time);

		Com com; //!< communication
		pthread_mutex_t mutex; //!< mutex de protection des donnees ci-dessous

		// données brutes
		struct hokuyo_scan hokuyo_scan[HOKUYO_MAX];
		struct control_usb_data control_usb_data[CONTROL_USB_DATA_MAX];
		int control_usb_data_count;
		struct fault_status fault_status[FAULT_MAX];

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

	protected:
		char name[32];
		pthread_t tid;
		volatile int stop_task;
		void (*callback)(void*);
		void* callback_arg;

		static void* task_wrapper(void* arg);
		void* task();
		int process_log(char* msg, uint16_t size);
		int process_control(char* msg, uint16_t size);
		int process_go(char* msg, uint16_t size);
		int process_hokuyo(int id, char* msg, uint16_t size);
		int process_hokuyo_seg(int id, char* msg, uint16_t size);
		int process_fault(char* msg, uint16_t size);
		int process_detect_dyn_obj_size(char* msg, uint16_t size);
		int process_detect_dyn_obj(char* msg, uint16_t size);
		int can_trace(char* msg, uint16_t size);
};

// ---------- gestion des pinces -----------------------------------------------

int robot_interface_pince(RobotInterface* data, enum pince_cmd_type cmd_type_left, enum pince_cmd_type cmd_type_right);

// ---------- gestion du bras --------------------------------------------------

int robot_interface_arm_xyz(RobotInterface* data, float x, float y, float z, enum arm_cmd_type type);

int robot_interface_arm_abz(RobotInterface* data, float a, float b, float z);

int robot_interface_arm_ventouse(RobotInterface* data, float x1, float y1, float x2, float y2, float z, int8_t tool_way);

int robot_interface_arm_hook(RobotInterface* data, float x1, float y1, float x2, float y2, float z, int8_t tool_way);

int robot_interface_arm_bridge(RobotInterface* data, uint8_t on);

// ---------- localisation -----------------------------------------------------

int robot_interface_set_position(RobotInterface* data, VectPlan pos);

// ---------- gestion control --------------------------------------------------

int robot_interface_control_print_param(RobotInterface* data);

int robot_interface_control_set_param(RobotInterface* data, int kp_av, int ki_av, int kd_av, int kp_rot, int ki_rot, int kd_rot, int kx, int ky, int kalpha);

int robot_interface_control_goto(RobotInterface* data, VectPlan dest, VectPlan cp, KinematicsParameters linearParam, KinematicsParameters angularParam);

// ---------- gestion trajectoire ----------------------------------------------

int robot_interface_straight(RobotInterface* data, float dist);

int robot_interface_straight_to_wall(RobotInterface* data);

int robot_interface_rotate(RobotInterface* data, float alpha);

int robot_interface_rotate_to(RobotInterface* data, float alpha);

int robot_interface_free(RobotInterface* data);

int robot_interface_goto_graph(RobotInterface* data);

int robot_interface_goto_near_xy(RobotInterface* data, float x, float y, float dist, unsigned int way, unsigned int avoidance_type);

int robot_interface_goto_near(RobotInterface* data, float x, float y, float alpha, float dist, unsigned int way, unsigned int avoidance_type);

//!< vitesse en % de vmax de configuration du robot
int robot_interface_set_max_speed(RobotInterface* data, float vmax_av, float vmax_rot);

int robot_interface_straight_speed(RobotInterface* data, float v);

int robot_interface_rotate_speed(RobotInterface* data, float v);

#endif
