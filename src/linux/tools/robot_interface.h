#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#ifndef LINUX
#define LINUX
#endif

#include <pthread.h>
#include "linux/tools/com/com.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/driver/can.h"
#include "kernel/driver/gyro/gyro.h"
#include "kernel/driver/pwm.h"
#include "kernel/error_codes.h"
#include "kernel/fault.h"
#include "kernel/math/polyline.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/Dynamixel.h"
#include "kernel/driver/power.h"
#include "kernel/pump.h"
#include "kernel/arm.h"
#include "kernel/control.h"
#include "middleware/detection.h"
#include "kernel/driver/io.h"
#include "kernel/match.h"
#include "middleware/trajectory/Trajectory.h"
#include "disco/star/wing.h"
#include "disco/star/finger.h"
#include "disco/star/carpet.h"
#include "server_tcp.h"
#include "disco/star/star.h"

#define CONTROL_USB_DATA_MAX        120000 //!< 600s (10 mn) de données avec l'asservissement à 200Hz

enum RobotVersion
{
	ROBOT_VERSION_UNKNOWN,
	ROBOT_VERSION_OK,
	ROBOT_VERSION_KO,
};

struct dynamixel_data
{
	int id;
	float pos;           //!< position
	uint16_t flags;      //!< flags
	struct DynamixelError error; //!< erreurs
};

class RobotInterface
{
	public:
		int init(const char* name, Com* com, bool server_tcp, void (*callback)(void*), void* callback_arg);
		void destroy();

		inline bool isConnected()
		{
			return connected;
		}

		int ptask();

		//! reboot soft du stm
		//! ATTENTION : pour les tests uniquement : n'est pas equivalent a un reboot HW
		int reboot();

		int power_off(bool power_off);

		// ---------- gestion des pwm (bas lv) -----------------------------------------
		int pwm_set(int id, float val);

		// ---------- gestion des dynamixel --------------------------------------------
		int dynamixel_cmd(uint8_t cmd, int dynamixel_type, uint8_t id, float param);
		int dynamixel_scan(int dynamixel_type);
		int dynamixel_set_id(int dynamixel_type, uint8_t id, uint8_t new_id);
		int dynamixel_set_op_baudrate(int dynamixel_type, uint8_t id);
		int dynamixel_set_manager_baudrate(int dynamixel_type, int freq);
		int dynamixel_set_goal_position(int dynamixel_type, uint8_t id, float alpha);
		int dynamixel_set_speed(int dynamixel_type, uint8_t id, float speed);
		int dynamixel_set_max_torque(int dynamixel_type, uint8_t id, float val);
		int dynamixel_set_target_reached_threshold(int dynamixel_type, uint8_t id, float val);
		int dynamixel_get_position(int dynamixel_type, uint8_t id);
		int dynamixel_enable_endless_turn_mode(int dynamixel_type, uint8_t id);
		int dynamixel_disable_endless_turn_mode(int dynamixel_type, uint8_t id);

		// ---------- gestion pompes ---------------------------------------------------
		int pump(uint8_t id, uint8_t val);
		inline bool pump_is_blocked(uint8_t id);

		// ---------- lecture gpio ------------------------------------------------------
		inline bool get_gpio(uint32_t mask);

		// ---------- gestion CAN ------------------------------------------------------
		int can_set_baudrate(enum can_baudrate baudrate, int debug);
		int can_write(struct can_msg* msg);
		int can_lss(bool on);
		int can_lss_set_nodeid(uint8_t nodeid);
		int can_lss_save();

		// ---------- gestion xbee -----------------------------------------------------
		int xbee_cmd(uint8_t cmd, float param);
		int xbee_set_op_baudrate();
		int xbee_set_manager_baudrate(uint32_t baudrate);

		// ---------- gestion recalage, go, couleur... ---------------------------------
		int recalage();
		int go();
		int go_enable();
		int color(uint8_t color);
		//!< time : temps en ms
		int set_match_time(uint32_t time);

		// ---------- gestion motion --------------------------------------------------
		int motion_enable(bool enable);
		int motion_set_max_driving_current(float maxCurrent);
		int motion_set_actuator_kinematics(struct motion_cmd_set_actuator_kinematics_arg cmd);
		int motion_set_speed(VectPlan u, float v);

		// ---------- gestion trajectoire ----------------------------------------------
		int straight(float dist);
		int rotate(float theta);
		int rotate_to(float theta);
		int goto_graph();
		int goto_graph_node(int id);
		int goto_near_xy(float x, float y, float dist, unsigned int way, unsigned int avoidance_type);
		int goto_near(VectPlan dest, float dist, unsigned int way, unsigned int avoidance_type);
		int free();

		// ---------- gestion motion --------------------------------------------------
		int motion_print_param();
		int motion_set_param(float kp_av, float ki_av, float kd_av, float kp_rot, float ki_rot, float kd_rot);

		// ---------- gestion gyro -----------------------------------------------------
		int gyro_calibration(enum GyroCalibrationCmd cmd);
		int gyro_set_calibration_values(float scale, float bias, float dead_zone);
		int gyro_set_position(float theta);
		inline int gyro_start_calibration()
		{
			return gyro_calibration(GYRO_CALIBRATION_START);
		}
		inline int gyro_stop_calibration()
		{
			return gyro_calibration(GYRO_CALIBRATION_STOP);
		}
		int gyro_stop_calibration(float theta);

		// ---------- localisation ------------------------------------------------------
		int set_position(VectPlan pos);

		// ---------- gestion du bras --------------------------------------------------
		int arm_cmd(uint32_t cmdType);

		// ---------- gestion robot 2015 -----------------------------------------------
		int wing(enum wing_cmd_type cmd_type_left, enum wing_cmd_type cmd_type_right);
		int elevator_set_position(float pos);
		int finger_set_position(enum finger_type low, enum finger_type high, enum finger_bottom_type right, enum finger_bottom_type left);
		int carpet_set_position(enum carpet_type right, enum carpet_type left);

/////////////////// TODO a mettre a jour
		// ---------- gestion du bras --------------------------------------------------
		int arm_xyz(float x, float y, float z, enum arm_cmd_type type);
		int arm_abz(float a, float b, float z);
		int arm_ventouse(float x1, float y1, float x2, float y2, float z, int8_t tool_way);

		//!< vitesse en % de vmax de configuration du robot
		int set_max_speed(float vmax_av, float vmax_rot);
//////////////////// FIN TODO
		//! ecriture sur l'usb
		//! @return 0 si ok, -1 sinon
		int usb_write(unsigned char cmd, void* data, int size);

		pthread_mutex_t mutex; //!< mutex de protection des donnees ci-dessous

		// données brutes
		struct hokuyo_scan hokuyo_scan[HOKUYO_MAX];
		struct control_usb_data control_usb_data[CONTROL_USB_DATA_MAX];
		struct control_usb_data last_control_usb_data;
		int control_usb_data_count;
		struct fault_status fault_status[FAULT_MAX];

		double current_time;
		double start_time;

		// TODO remettre d'equerre avec 2 hokuyos...
#if 1
		int detection_dynamic_object_id;
		int detection_dynamic_object_pt_tmp_size;
		int16_t detection_dynamic_object_size_tmp;
		struct Vect2 detection_dynamic_object_pt_tmp[HOKUYO_NUM_POINTS];
		struct polyline detection_dynamic_obj_tmp[HOKUYO_NUM_POINTS];

		int16_t detection_dynamic_object_size;
		struct Vect2 detection_dynamic_object_pt[HOKUYO_NUM_POINTS];
		struct polyline detection_dynamic_obj[HOKUYO_NUM_POINTS];
#endif
		// detection adverse
		struct detection_object detection_obj1[DETECTION_NUM_OBJECT_USB];
		int16_t detection_dynamic_object_count1;
		struct detection_object detection_obj2[DETECTION_NUM_OBJECT_USB];
		int16_t detection_dynamic_object_count2;

		struct dynamixel_data ax12[DYNAMIXEL_MAX_ON_BUS];
		//struct dynamixel_data rx24[DYNAMIXEL_MAX_ON_BUS];

		// calculs
		struct Vect2 detection_hokuyo_pos[HOKUYO_NUM_POINTS*HOKUYO_MAX];
		struct Vect2 detection_hokuyo_reg[HOKUYO_NUM_POINTS*HOKUYO_MAX]; // TODO à virer
		int detection_reg_num[HOKUYO_MAX];

		RobotVersion versionCompatible;
		char stm_code_version[41];
		static const char expected_version[41];
		const char* getName(){return name;};

	protected:
		char msg[65537]; // taille max = 65536 (header.size) + 1
		char name[32];
		pthread_t tid;
		volatile int stop_task;
		void (*callback)(void*);
		void* callback_arg;
		Com* com; //!< communication
		bool connected;
		ServerTcp serverTcp;

		void fault_reset();
		static void* task_wrapper(void* arg);
		void* task();
		int process_log(char* msg, uint16_t size);
		int process_control(char* msg, uint16_t size);
		int process_control_light(char* msg, uint16_t size);
		int process_go(char* msg, uint16_t size);
		int process_hokuyo(char* msg, uint16_t size);
		int process_hokuyo_seg(char* msg, uint16_t size);
		int process_fault(char* msg, uint16_t size);
		int process_detect_dyn_obj_size1(char* msg, uint16_t size);
		int process_detect_dyn_obj_size2(char* msg, uint16_t size);
		int process_detect_dyn_obj(char* msg, uint16_t size);
		int process_detect_obj1(char* msg, uint16_t size);
		int process_detect_obj2(char* msg, uint16_t size);
		int process_code_version(char* msg, uint16_t size);
		int can_trace(char* msg, uint16_t size);
		int get_stm_code_version();

		int add_usb_data_callback(uint8_t cmd, int (RobotInterface::*process_func)(char* msg, uint16_t size));
		int (RobotInterface::*process_func[USB_DATA_MAX])(char* msg, uint16_t size);
};


inline bool RobotInterface::get_gpio(uint32_t mask)
{
	return (last_control_usb_data.gpio & mask) ? 1 : 0;
}

inline bool RobotInterface::pump_is_blocked(uint8_t id)
{
	return (last_control_usb_data.pumpState >> id) & 0x01;
}

#endif
