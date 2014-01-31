#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <pthread.h>
#include "linux/tools/com.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/driver/can.h"
#include "kernel/driver/spi.h"
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

		// ---------- gestion control --------------------------------------------------
		int control_print_param();
		int control_set_param(int kp_av, int ki_av, int kd_av, int kp_rot, int ki_rot, int kd_rot, int kx, int ky, int kalpha); // TODO a mettre a jour
		int control_goto(VectPlan dest, VectPlan cp, KinematicsParameters linearParam, KinematicsParameters angularParam);
		int control_set_speed(VectPlan cp, VectPlan u, float v);
		int control_free();

		// ---------- gestion control --------------------------------------------------
		int gyro_calibration(enum spi_calibration_cmd cmd);

/////////////////// TODO a mettre a jour
		// ---------- gestion des pinces -----------------------------------------------
		int pince(enum pince_cmd_type cmd_type_left, enum pince_cmd_type cmd_type_right);

		// ---------- gestion du bras --------------------------------------------------
		int arm_xyz(float x, float y, float z, enum arm_cmd_type type);
		int arm_abz(float a, float b, float z);
		int arm_ventouse(float x1, float y1, float x2, float y2, float z, int8_t tool_way);
		int arm_hook(float x1, float y1, float x2, float y2, float z, int8_t tool_way);
		int arm_bridge(uint8_t on);

		// ---------- localisation -----------------------------------------------------
		int set_position(VectPlan pos);

		// ---------- gestion trajectoire ----------------------------------------------
		int straight(float dist);
		int straight_to_wall();
		int rotate(float theta);
		int rotate_to(float theta);
		int goto_graph();
		int goto_near_xy(float x, float y, float dist, unsigned int way, unsigned int avoidance_type);
		int goto_near(VectPlan dest, float dist, unsigned int way, unsigned int avoidance_type);

		//!< vitesse en % de vmax de configuration du robot
		int set_max_speed(float vmax_av, float vmax_rot);
		int straight_speed(float v);
		int rotate_speed(float v);
//////////////////// FIN TODO

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
		Com com; //!< communication

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

#endif
