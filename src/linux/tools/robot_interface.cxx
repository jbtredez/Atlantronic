#include <string.h>
#include <pthread.h>
#include <stdlib.h>
#include <math.h>
#include "linux/tools/robot_interface.h"
#include "linux/tools/com.h"
#include "linux/tools/cli.h"
#include "kernel/hokuyo_tools.h"
#include "kernel/math/regression.h"
#include "kernel/log_level.h"
#include "kernel/systick.h"
#include "kernel/driver/usb.h"
#include "foo/control/trajectory.h"
#include "kernel/driver/dynamixel.h"

const char* fault_description[FAULT_MAX] =
{
	// HOKUYO
	[FAULT_HOKUYO_DISCONNECTED] = "hokuyo disconnected",
	[FAULT_HOKUYO_DATA_CORRUPTION] = "hokuyo - data corruption",

	// CAN
	[FAULT_CAN_READ_QUEUE_FULL] = "can : queue de lecture pleine",
	[FAULT_CAN_READ_FIFO_OVERFLOW] = "can : fifo de lecture qui deborde - perte de messages",
};

const char* log_level_description[LOG_MAX] =
{
	"Error",
	"Info",
	"Debug1",
	"Debug2",
};

const char* log_level_color_start[LOG_MAX] =
{
		"\033[31m",       // erreurs en rouge
		"",
		"",
		""
};

const char* log_level_color_end[LOG_MAX] =
{
		"\033[0m",       // fin erreurs en rouge
		"",
		"",
		""
};

int RobotInterface::init(const char* _name, const char* file_read, const char* file_write, void (*_callback)(void*), void* _callback_arg)
{
	int i;
	int err = 0;
	int res = 0;

	strncpy(name, _name, sizeof(name));
	callback = _callback;
	callback_arg = _callback_arg;
	stop_task = 1;
	start_time = 0;
	current_time = 0;

	if(file_read)
	{
		com.init(file_read, file_write);
	}
	else
	{
		com.init("/dev/discovery0", "/dev/discovery0");
	}

	control_usb_data_count = 0;
	memset(fault_status, 0x00, sizeof(fault_status));

	for(i = 0; i < HOKUYO_MAX ; i++)
	{
		detection_reg_num[i] = 0;
	}

	for( i = 0; i < COM_MAX ; i++)
	{
		stop_task = 0;
		res = pthread_create(&tid, NULL, RobotInterface::task_wrapper, this);
		if(res)
		{
			err = res;
			log_error_errno("pthread_create");
		}
	}

	return err;
}

void RobotInterface::destroy()
{
	stop_task = 1;
	usleep(100000);
	pthread_cancel(tid);

	com.close();
	com.destroy();

	rl_free_line_state();
	rl_cleanup_after_signal();
}

void* RobotInterface::task_wrapper(void* arg)
{
	RobotInterface* robot = (RobotInterface*) arg;
	return robot->task();
}

void* RobotInterface::task()
{
	int res;
	unsigned char lost[1024];
	unsigned int lost_count = 0;

	com.open_block();

	while( !stop_task)
	{
		uint16_t type;
		uint16_t size;

		// lecture entete
		res = com.read_header(&type, &size);

		if( stop_task)
		{
			goto end;
		}

		if( res )
		{
			com.open_block();
			continue;
		}

		// lecture du message
		res = com.read(size + 4);
		if( stop_task)
		{
			goto end;
		}

		if( res )
		{
			com.open_block();
			continue;
		}

		// copie du message (vers un buffer non circulaire)
		char msg[size+1];
		com.copy_msg(msg, size+1);

		// traitement du message
		switch( type )
		{
			case USB_LOG:
				res = process_log(msg, size);
				break;
			case USB_ERR:
				res = process_fault(msg, size);
				break;
			case USB_HOKUYO1:
				res = process_hokuyo(HOKUYO1, msg, size);
				break;
			case USB_HOKUYO2:
				res = process_hokuyo(HOKUYO2, msg, size);
				break;
			case USB_HOKUYO_FOO_SEG:
				res = process_hokuyo_seg(HOKUYO1, msg, size);
				break;
			case USB_CONTROL:
				res = process_control(msg, size);
				break;
			case USB_GO:
				res = process_go(msg, size);
				break;
			case USB_DETECTION_DYNAMIC_OBJECT_SIZE:
				res = process_detect_dyn_obj_size(msg, size);
				break;
			case USB_DETECTION_DYNAMIC_OBJECT:
				res = process_detect_dyn_obj(msg, size);
				break;
			case USB_CAN_TRACE:
				res = can_trace(msg, size);
				break;
			default:
				res = -1;
				break;
		}

		if( res )
		{
			unsigned char byte = com.buffer[com.buffer_begin];
			if(lost_count >= sizeof(lost)-2)
			{
				lost[lost_count+1] = 0;
				log_error("%s unknown data0 (%d) : %s", name, lost_count, lost);
				lost_count = 0;
			}

			lost[lost_count] = byte;
			lost_count ++;
			if(byte == 0 || byte == '\n')
			{
				lost[lost_count+1] = 0;
				log_error("%s unknown data1 (%d) : %s", name, lost_count, lost);
				lost_count = 0;
			}

			//printf("wrong format, type : %i, size = %i, - skip %#.2x (%c)\n", type, size, com->buffer[com->buffer_begin], com->buffer[com->buffer_begin]);
			com.skip(1);
		}
		else
		{
			if(lost_count)
			{
				lost[lost_count+1] = 0;
				log_error("%s unknown data2 (%d) : %s", name, lost_count, lost);
				lost_count = 0;
			}

			size += 4;
			com.skip(size);
			if(callback)
			{
				callback(callback_arg);
			}
		}
	}

end:
	return NULL;
}

int RobotInterface::process_log(char* msg, uint16_t size)
{
	int res = 0;
	struct systime current_time;
	double time;
	unsigned char level;
	char* log;

	if( msg[size-1] != '\n' )
	{
		res = -1;
		goto end;
	}

	memcpy(&current_time, msg, 8);
	time = current_time.ms/1000.0f + current_time.ns/1000000000.0f;

	msg[size-1] = 0;

	level = msg[8];
	uint16_t line;
	memcpy(&line, msg + 9, 2);

	log = strchr(msg+11, ':');
	*log = 0;
	log++;

	if(level < LOG_MAX)
	{
		log_info("%s%4s %13.6f %8s   %20s:%5d    %s%s", log_level_color_start[level], name, time, log_level_description[level], msg+11, line, log, log_level_color_end[level]);
	}
	else
	{
		log_info("%4s %13.6f %8s   %20s:%5d    %s", name, time, "unknown", msg+11, line, log);
	}

end:
	return res;
}

int RobotInterface::process_fault(char* msg, uint16_t size)
{
	int res = 0;
	int i = 0;
	struct fault_status* fault_list = (struct fault_status*) msg;

	const char* fault_color[] =
	{
		"\033[32m",
		"\033[31m"
	};
	if(size != sizeof(struct fault_status) * FAULT_MAX)
	{
		res = -1;
		goto end;
	}

	res = pthread_mutex_lock(&mutex);

	if(res)
	{
		log_error("pthread_mutex_lock : %i", res);
		goto end;
	}

	for(i=0; i< FAULT_MAX; i++)
	{
		unsigned char state = fault_list[i].state;
		if(state != fault_status[i].state)
		{
			log_info("%s%4s %13.6f    Fault\t%s (%d), num %d status %d\033[0m", fault_color[state & 0x01], name, fault_list[i].time / 1000.0f, fault_description[i], i, state >> 1, state & 0x01);
			fault_status[i] = fault_list[i];
		}
	}

	pthread_mutex_unlock(&mutex);

end:
	return res;
}

int RobotInterface::process_detect_dyn_obj_size(char* msg, uint16_t size)
{
	int res = 0;

	if(size != sizeof(detection_dynamic_object_size_tmp))
	{
		res = -1;
		goto end;
	}

	res = pthread_mutex_lock(&mutex);

	if(res)
	{
		log_error("pthread_mutex_lock : %i", res);
		goto end;
	}

	detection_dynamic_object_id = 0;
	detection_dynamic_object_pt_tmp_size = 0;
	memcpy(&detection_dynamic_object_size_tmp, msg, sizeof(detection_dynamic_object_size_tmp));

	// pas d'objets a attendre
	if( detection_dynamic_object_size_tmp == 0)
	{
		detection_dynamic_object_size = 0;
	}

	pthread_mutex_unlock(&mutex);

end:
	return res;
}

int RobotInterface::process_detect_dyn_obj(char* msg, uint16_t size)
{
	int res = 0;
	int obj_id;

	unsigned int num = size / sizeof(detection_dynamic_object_pt[0]);
	if(size != num * sizeof(detection_dynamic_object_pt[0]) )
	{
		res = -1;
		goto end;
	}

	res = pthread_mutex_lock(&mutex);

	if(res)
	{
		log_error("pthread_mutex_lock : %i", res);
		goto end;
	}

	obj_id = detection_dynamic_object_id;

	if(obj_id < detection_dynamic_object_size_tmp)
	{
		memcpy(&detection_dynamic_object_pt_tmp[detection_dynamic_object_pt_tmp_size], msg, size);
		detection_dynamic_obj_tmp[obj_id].pt = &detection_dynamic_object_pt_tmp[detection_dynamic_object_pt_tmp_size];
		detection_dynamic_obj_tmp[obj_id].size = num;
		detection_dynamic_object_pt_tmp_size += num;

		detection_dynamic_object_id++;
	}
	if(obj_id == detection_dynamic_object_size_tmp - 1)
	{
		memcpy(detection_dynamic_object_pt, detection_dynamic_object_pt_tmp, detection_dynamic_object_pt_tmp_size * sizeof(detection_dynamic_object_pt[0]));
		memcpy(detection_dynamic_obj, detection_dynamic_obj_tmp, detection_dynamic_object_size * sizeof(detection_dynamic_obj[0]));
		detection_dynamic_object_size = detection_dynamic_object_size_tmp;
	}

	pthread_mutex_unlock(&mutex);

end:
	return res;
}

int RobotInterface::can_trace(char* msg, uint16_t size)
{
	int res = 0;
	int i = 0;

	struct can_msg* can_msg = (struct can_msg*) msg;

	if( size != sizeof(struct can_msg))
	{
		res = -1;
		goto end;
	}

	if( can_msg->size > 8 )
	{
		can_msg->size = 8;
	}

	char buffer[1024];
	res = snprintf(buffer, sizeof(buffer), "%4s %13.6f %8s   id %6x size %u data", name, can_msg->time.ms/1000.0f + can_msg->time.ns/1000000000.0f,
			log_level_description[LOG_DEBUG1], (unsigned int)can_msg->id, can_msg->size);
	for(i=0; i < can_msg->size && res > 0; i++)
	{
		res += snprintf(buffer + res, sizeof(buffer) - res, " %2.2x", can_msg->data[i]);
	}
	log_info("%s", buffer);

	res = 0;

end:
	return res;
}

int RobotInterface::process_hokuyo(int id, char* msg, uint16_t size)
{
	int res = 0;

	if(size != sizeof(struct hokuyo_scan))
	{
		res = -1;
		goto end;
	}

	res = pthread_mutex_lock(&mutex);

	if(res)
	{
		log_error("pthread_mutex_lock : %i", res);
		goto end;
	}

	memcpy(&hokuyo_scan[id], msg, size);

	hokuyo_compute_xy(&hokuyo_scan[id], detection_hokuyo_pos + HOKUYO_NUM_POINTS * id);

	pthread_mutex_unlock(&mutex);

end:
	return res;
}

int RobotInterface::process_hokuyo_seg(int id, char* msg, uint16_t size)
{
	int res = 0;

	unsigned int num = size / sizeof(detection_hokuyo_reg[0]);
	if(size != num * sizeof(detection_hokuyo_reg[0]) )
	{
		res = -1;
		goto end;
	}

	if( num > HOKUYO_NUM_POINTS)
	{
		res = -1;
		goto end;
	}

	res = pthread_mutex_lock(&mutex);

	if(res)
	{
		log_error("pthread_mutex_lock : %i", res);
		goto end;
	}

	detection_reg_num[id] = num;
	memcpy(detection_hokuyo_reg + HOKUYO_NUM_POINTS * id, msg, size);

	pthread_mutex_unlock(&mutex);

end:
	return res;
}

int RobotInterface::process_control(char* msg, uint16_t size)
{
	int res = 0;
	systime t;

	if(size != sizeof(struct control_usb_data) )
	{
		res = -1;
		goto end;
	}

	res = pthread_mutex_lock(&mutex);

	if(res)
	{
		log_error("pthread_mutex_lock : %i", res);
		goto end;
	}

	memcpy(control_usb_data + control_usb_data_count, msg, size);
	t = control_usb_data[control_usb_data_count].current_time;
	current_time = t.ms / 1000.0f + t.ns / 1000000000.0f;
	control_usb_data_count = (control_usb_data_count + 1) % CONTROL_USB_DATA_MAX;

	pthread_mutex_unlock(&mutex);

end:
	return res;
}

int RobotInterface::process_go(char* msg, uint16_t size)
{
	int res = 0;
	struct systime t;
	uint32_t match_time;

	if(size != sizeof(t) + sizeof(match_time))
	{
		res = -1;
		goto end;
	}

	memcpy(&t, msg, sizeof(t));
	memcpy(&match_time, msg+sizeof(t), sizeof(match_time));

	current_time = t.ms / 1000.0f + t.ns / 1000000000.0f;
	start_time = current_time;

	log_info("%4s %13.6f %8s   GO - durée du match : %u ms", name, start_time, log_level_description[LOG_INFO], match_time);

end:
	return res;
}

int RobotInterface::dynamixel_scan(int dynamixel_type)
{
	struct dynamixel_cmd_param cmd_arg;

	cmd_arg.cmd_id = DYNAMIXEL_CMD_SCAN;
	cmd_arg.type = dynamixel_type;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_DYNAMIXEL;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com.write(buffer, sizeof(buffer));
}

int RobotInterface::dynamixel_set_id(int dynamixel_type, uint8_t id, uint8_t new_id)
{
	struct dynamixel_cmd_param cmd_arg;

	cmd_arg.cmd_id = DYNAMIXEL_CMD_SET_ID;
	cmd_arg.type = dynamixel_type;
	cmd_arg.id = id;
	cmd_arg.param = new_id;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_DYNAMIXEL;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com.write(buffer, sizeof(buffer));
}

int RobotInterface::dynamixel_set_goal_position(int dynamixel_type, uint8_t id, float alpha)
{
	struct dynamixel_cmd_param cmd_arg;

	cmd_arg.cmd_id = DYNAMIXEL_CMD_SET_GOAL_POSITION;
	cmd_arg.type = dynamixel_type;
	cmd_arg.id = id;
	cmd_arg.param = alpha * (1 << 26) / (2 * M_PI);

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_DYNAMIXEL;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com.write(buffer, sizeof(buffer));
}

int RobotInterface::dynamixel_get_position(int dynamixel_type, uint8_t id)
{
	struct dynamixel_cmd_param cmd_arg;

	cmd_arg.cmd_id = DYNAMIXEL_CMD_GET_POSITION;
	cmd_arg.type = dynamixel_type;
	cmd_arg.id = id;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_DYNAMIXEL;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com.write(buffer, sizeof(buffer));
}

int robot_interface_control_print_param(RobotInterface* data)
{
	char buffer[1];
	buffer[0] = USB_CMD_CONTROL_PRINT_PARAM;

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_control_set_param(RobotInterface* data, int kp_av, int ki_av, int kd_av, int kp_rot, int ki_rot, int kd_rot, int kx, int ky, int kalpha)
{
	struct control_cmd_param_arg cmd_arg;

	cmd_arg.kp_av = kp_av;
	cmd_arg.ki_av = ki_av;
	cmd_arg.kd_av = kd_av;
	cmd_arg.kp_rot = kp_rot;
	cmd_arg.ki_rot = ki_rot;
	cmd_arg.kd_rot = kd_rot;
	cmd_arg.kx = kx;
	cmd_arg.ky = ky;
	cmd_arg.kalpha = kalpha;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_CONTROL_PARAM;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_control_goto(RobotInterface* data, VectPlan dest, VectPlan cp, KinematicsParameters linearParam, KinematicsParameters angularParam)
{
	struct control_cmd_goto_arg cmd_arg;

	cmd_arg.type = CONTROL_GOTO;
	cmd_arg.dest = dest;
	cmd_arg.cp = cp;
	cmd_arg.linearParam = linearParam;
	cmd_arg.angularParam = angularParam;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_CONTROL_SET_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_straight(RobotInterface* data, float dist)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.dist = dist * 65536.0f;
	cmd_arg.type = TRAJECTORY_STRAIGHT;
	cmd_arg.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_straight_to_wall(RobotInterface* data)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.type = TRAJECTORY_STRAIGHT_TO_WALL;
	cmd_arg.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_rotate(RobotInterface* data, float alpha)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.alpha = alpha * (1 << 26) / (2 * M_PI);
	cmd_arg.type = TRAJECTORY_ROTATE;
	cmd_arg.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_rotate_to(RobotInterface* data, float alpha)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.alpha = alpha * (1 << 26) / (2 * M_PI);
	cmd_arg.type = TRAJECTORY_ROTATE_TO;
	cmd_arg.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_free(RobotInterface* data)
{
	struct control_cmd_goto_arg cmd_arg;

	cmd_arg.type = CONTROL_FREE;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_CONTROL_SET_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_goto_near_xy(RobotInterface* data, float x, float y, float dist, unsigned int way, unsigned int avoidance_type)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.x = x * 65536.0f;
	cmd_arg.y = y * 65536.0f;
	cmd_arg.dist = dist * 65536.0f;
	cmd_arg.type = TRAJECTORY_GOTO_XY;
	cmd_arg.avoidance_type = avoidance_type;
	cmd_arg.way = way;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_goto_near(RobotInterface* data, float x, float y, float alpha, float dist, unsigned int way, unsigned int avoidance_type)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.x = x * 65536.0f;
	cmd_arg.y = y * 65536.0f;
	cmd_arg.alpha = alpha * (1 << 26) / (2 * M_PI);
	cmd_arg.dist = dist * 65536.0f;
	cmd_arg.type = TRAJECTORY_GOTO_XYA;
	cmd_arg.avoidance_type = avoidance_type;
	cmd_arg.way = way;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_goto_graph(RobotInterface* data)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.type = TRAJECTORY_GOTO_GRAPH;
	cmd_arg.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_set_position(RobotInterface* data, VectPlan pos)
{
	char buffer[1+sizeof(pos)];
	buffer[0] = USB_CMD_LOCATION_SET_POSITION;
	memcpy(buffer+1, &pos, sizeof(pos));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_pince(RobotInterface* data, enum pince_cmd_type cmd_type_left, enum pince_cmd_type cmd_type_right)
{
	struct pince_cmd_arg cmd_arg;

	cmd_arg.type_left = cmd_type_left;
	cmd_arg.type_right = cmd_type_right;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_PINCE;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_arm_xyz(RobotInterface* data, float x, float y, float z, enum arm_cmd_type type)
{
	struct arm_cmd_goto_param cmd_arg;

	if(type == ARM_CMD_ART)
	{
		return -1;
	}

	cmd_arg.x = x * 65536.0f;
	cmd_arg.y = y * 65536.0f;
	cmd_arg.z = z * 65536.0f;
	cmd_arg.type = type;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_ARM_GOTO;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_arm_ventouse(RobotInterface* data, float x1, float y1, float x2, float y2, float z, int8_t tool_way)
{
	struct arm_cmd_goto_param cmd_arg;

	cmd_arg.x1 = x1 * 65536.0f;
	cmd_arg.y1 = y1 * 65536.0f;
	cmd_arg.x2 = x2 * 65536.0f;
	cmd_arg.y2 = y2 * 65536.0f;
	cmd_arg.z = z * 65536.0f;
	cmd_arg.tool_way = tool_way;
	cmd_arg.type = ARM_CMD_VENTOUSE_ABS;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_ARM_GOTO;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_arm_hook(RobotInterface* data, float x1, float y1, float x2, float y2, float z, int8_t tool_way)
{
	struct arm_cmd_goto_param cmd_arg;

	cmd_arg.x1 = x1 * 65536.0f;
	cmd_arg.y1 = y1 * 65536.0f;
	cmd_arg.x2 = x2 * 65536.0f;
	cmd_arg.y2 = y2 * 65536.0f;
	cmd_arg.z = z * 65536.0f;
	cmd_arg.tool_way = tool_way;
	cmd_arg.type = ARM_CMD_HOOK_ABS;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_ARM_GOTO;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_arm_abz(RobotInterface* data, float a, float b, float z)
{
	struct arm_cmd_goto_param cmd_arg;

	cmd_arg.a = a * (1 << 26) / (2 * M_PI);
	cmd_arg.b = b * (1 << 26) / (2 * M_PI);
	cmd_arg.z = z * 65536.0f;
	cmd_arg.type = ARM_CMD_ART;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_ARM_GOTO;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_arm_bridge(RobotInterface* data, uint8_t on)
{
	char buffer[2];
	buffer[0] = USB_CMD_ARM_BRIDGE;
	buffer[1] = on;

	return data->com.write(buffer, sizeof(buffer));
}

int RobotInterface::recalage()
{
	char buffer[1];
	buffer[0] = USB_CMD_RECALAGE;

	return com.write(buffer, sizeof(buffer));
}

int RobotInterface::go()
{
	char buffer[1];
	buffer[0] = USB_CMD_GO;

	return com.write(buffer, sizeof(buffer));
}

int RobotInterface::color(uint8_t color)
{
	char buffer[2];
	buffer[0] = USB_CMD_COLOR;
	buffer[1] = color;

	return com.write(buffer, sizeof(buffer));
}

int RobotInterface::set_match_time(uint32_t time)
{
	char buffer[5];
	buffer[0] = USB_CMD_MATCH_TIME;
	memcpy(buffer + 1, &time, 4);

	return com.write(buffer, sizeof(buffer));
}

int RobotInterface::can_set_baudrate(enum can_baudrate baudrate, int debug)
{
	char buffer[3];
	buffer[0] = USB_CMD_CAN_SET_BAUDRATE;
	buffer[1] = baudrate;
	buffer[2] = debug;

	return com.write(buffer, sizeof(buffer));
}

int RobotInterface::can_write(struct can_msg* msg)
{
	char buffer[sizeof(struct can_msg) + 1];
	buffer[0] = USB_CMD_CAN_WRITE;
	memcpy(buffer + 1, msg, sizeof(struct can_msg));

	return com.write(buffer, sizeof(buffer));
}

int robot_interface_set_max_speed(RobotInterface* data, float vmax_av, float vmax_rot)
{
	struct control_cmd_max_speed_arg cmd_arg;

	cmd_arg.vmax_av = fabsf(vmax_av) * 65536.0f;
	cmd_arg.vmax_rot = fabsf(vmax_rot) * 65536.0f;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_CONTROL_MAX_SPEED;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return data->com.write(buffer, sizeof(buffer));
}

int robot_interface_rotate_speed(RobotInterface* data, float v)
{
	(void) data;
	(void) v;
/*	enum control_state control_state = CONTROL_READY_FREE;
	int control_v_rot_cons = 0;
	int control_v_dist_cons = 0;
	int dir = 1;
	if(v < 0)
	{
		v = -v;
		dir = -1;
	}

	int res = pthread_mutex_lock(&data->mutex);
	if(res == 0)
	{
		if(data->control_usb_data_count > 0)
		{
			control_state = (enum control_state)data->control_usb_data[data->control_usb_data_count-1].control_state;
			control_v_dist_cons = data->control_usb_data[data->control_usb_data_count-1].control_v_dist_cons;
			control_v_rot_cons = data->control_usb_data[data->control_usb_data_count-1].control_v_rot_cons;
		}

		pthread_mutex_unlock(&data->mutex);
	}
	else
	{
		v = 0;
	}

	// on avance déjà, pas de rotation
	if( control_v_dist_cons != 0)
	{
		return 0;
	}

	if( v < 0.2f)
	{
		if( fabsf(control_v_rot_cons) > 0)
		{
			robot_interface_set_max_speed(data, 0, 0);
		}
	}
	else
	{
		robot_interface_set_max_speed(data, v, v);
		if(control_state != CONTROL_TRAJECTORY || dir * control_v_rot_cons < 0 || (control_v_dist_cons == 0 && control_v_rot_cons == 0))
		{
			robot_interface_rotate(data, 60*dir);
			usleep(50000);
		}
	}
*/
	return 0;
}

int robot_interface_straight_speed(RobotInterface* data, float v)
{
	(void) data;
	(void) v;
/*	enum control_state control_state = CONTROL_READY_FREE;
	int control_v_dist_cons = 0;
	int control_v_rot_cons = 0;
	int dir = 1;
	if(v < 0)
	{
		v = -v;
		dir = -1;
	}

	int res = pthread_mutex_lock(&data->mutex);
	if(res == 0)
	{
		if(data->control_usb_data_count > 0)
		{
			control_state = (enum control_state)data->control_usb_data[data->control_usb_data_count-1].control_state;
			control_v_dist_cons = data->control_usb_data[data->control_usb_data_count-1].control_v_dist_cons;
			control_v_rot_cons = data->control_usb_data[data->control_usb_data_count-1].control_v_rot_cons;
		}

		pthread_mutex_unlock(&data->mutex);
	}
	else
	{
		v = 0;
	}

	// rotation deja en cours
	if( control_v_rot_cons != 0)
	{
		return 0;
	}

	if( v < 0.1f)
	{
		if( fabsf(control_v_dist_cons) > 0)
		{
			robot_interface_set_max_speed(data, 0, 0);
		}
	}
	else
	{
		robot_interface_set_max_speed(data, v, v);
		if(control_state != CONTROL_TRAJECTORY || dir * control_v_dist_cons < 0 || (control_v_dist_cons == 0 && control_v_rot_cons == 0))
		{
			robot_interface_straight(data, dir*30000);
			usleep(50000);
		}
	}
*/
	return 0;
}
