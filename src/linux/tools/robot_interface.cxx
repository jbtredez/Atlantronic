#include <string.h>
#include <pthread.h>
#include <stdlib.h>
#include <math.h>
#include "linux/tools/robot_interface.h"
#include "linux/tools/cli.h"
#include "kernel/hokuyo_tools.h"
#include "kernel/math/regression.h"
#include "kernel/log_level.h"
#include "kernel/systick.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/xbee.h"
#include "kernel/pump.h"
#include "kernel/match.h"
#include "kernel/motion/graph.h"
#include "disco/elevator.h"

#ifndef VERSION
#error VERSION not defined
#endif

const char* fault_description[FAULT_MAX] =
{
	// erreurs OS
	"unaligned memory acces, task killed",
	"div by 0, task killed",

	// HOKUYO
	"hokuyo disconnected",
	"hokuyo - data corruption",

	// CAN
	"can : not connected - init failed",
	"can : queue de lecture pleine",
	"can : fifo de lecture qui deborde - perte de messages",

	"can motor 0 (right) disconnected",
	"can motor 1 (left ) disconnected",

	// Gyro
	"gyro disconnected",
	"gyro error",
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

const char RobotInterface::expected_version[41] = VERSION;

int RobotInterface::init(const char* _name, Com* _com, bool server_tcp, void (*_callback)(void*), void* _callback_arg)
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
	connected = false;
	com = _com;
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

	versionCompatible = ROBOT_VERSION_UNKNOWN;

	for( int i = 0; i < USB_DATA_MAX; i++)
	{
		process_func[i] = 0;
	}
	add_usb_data_callback(USB_LOG, &RobotInterface::process_log);
	add_usb_data_callback(USB_ERR, &RobotInterface::process_fault);
	add_usb_data_callback(USB_HOKUYO, &RobotInterface::process_hokuyo);
	add_usb_data_callback(USB_HOKUYO_SEG, &RobotInterface::process_hokuyo_seg);
	add_usb_data_callback(USB_CONTROL, &RobotInterface::process_control);
	add_usb_data_callback(USB_CONTROL_LIGHT, &RobotInterface::process_control_light);
	add_usb_data_callback(USB_GO, &RobotInterface::process_go);
	add_usb_data_callback(USB_DETECTION_DYNAMIC_OBJECT_SIZE, &RobotInterface::process_detect_dyn_obj_size1);
	add_usb_data_callback(USB_DETECTION_DYNAMIC_OBJECT_SIZE2, &RobotInterface::process_detect_dyn_obj_size2);
	add_usb_data_callback(USB_DETECTION_DYNAMIC_OBJECT_POLYLINE, &RobotInterface::process_detect_dyn_obj);
	add_usb_data_callback(USB_DETECTION_DYNAMIC_OBJECT, &RobotInterface::process_detect_obj1);
	add_usb_data_callback(USB_DETECTION_DYNAMIC_OBJECT2, &RobotInterface::process_detect_obj2);
	add_usb_data_callback(USB_CAN_TRACE, &RobotInterface::can_trace);
	add_usb_data_callback(USB_CMD_GET_VERSION, &RobotInterface::process_code_version);

	if(server_tcp )
	{
		serverTcp.configure(com, 41666);
		serverTcp.start();
	}

//	detection_dynamic_object_count = 0;

	detection_dynamic_object_count1 = 0;
	detection_dynamic_object_count2 = 0;

	return err;
}

void RobotInterface::destroy()
{
	stop_task = 1;
	usleep(100000);
	pthread_cancel(tid);

	serverTcp.stop();
	com->close();

	rl_free_line_state();
	rl_cleanup_after_signal();
}

int RobotInterface::add_usb_data_callback(uint8_t cmd, int (RobotInterface::*func)(char* msg, uint16_t size))
{
	if( cmd < USB_DATA_MAX)
	{
		process_func[cmd] = func;
		return 0;
	}

	return -1;
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

	com->open_block();
	get_stm_code_version();

	while( !stop_task)
	{
		struct usb_header header;

		// lecture entete
		res = com->read_header(&header);

		if( stop_task)
		{
			goto end;
		}

		if( res )
		{
			fault_reset();
			connected = false;
			com->open_block();
			get_stm_code_version();
			continue;
		}


		serverTcp.write(&header, sizeof(header));

		// lecture du message
		res = com->read(header.size + 4);
		if( stop_task)
		{
			goto end;
		}

		if( res )
		{
			fault_reset();
			connected = false;
			com->open_block();
			get_stm_code_version();
			continue;
		}

		// copie du message (vers un buffer non circulaire)
		com->copy(msg, 4, header.size);
		msg[header.size] = 0;

		// traitement du message
		if( header.type < USB_DATA_MAX)
		{
			if( process_func[header.type] )
			{
				res = (this->*process_func[header.type])(msg, header.size);
			}
		}
		else
		{
			res = -1;
		}

		if( res )
		{
			if( lost_count == 0)
			{
				// premiere perte : log header
				log_error("com error, header = %d %d", header.type, header.size);
			}
			unsigned char byte = com->buffer[com->buffer_begin];
			if(lost_count >= sizeof(lost)-2)
			{
				lost[lost_count+1] = 0;
				char buffer[10240];
				int offset = 0;
				for(int i = 0; i < (int)lost_count; i++)
				{
					int res = snprintf(buffer + offset, sizeof(buffer) - offset, "0x%.2x ", lost[i]);
					if( res > 0)
					{
						offset += res;
					}
					snprintf(buffer + offset, sizeof(buffer) - offset, "\n");
				}
				log_error("%s unknown data2 (%d) : %s", name, lost_count, buffer);
				lost_count = 0;
			}

			lost[lost_count] = byte;
			lost_count ++;

			//printf("wrong format, type : %i, size = %i, - skip %#.2x (%c)\n", type, size, com->buffer[com->buffer_begin], com->buffer[com->buffer_begin]);
			com->skip(1);
		}
		else
		{
			if(lost_count)
			{
				lost[lost_count+1] = 0;
				char buffer[10240];
				int offset = 0;
				for(int i = 0; i < (int)lost_count; i++)
				{
					int res = snprintf(buffer + offset, sizeof(buffer) - offset, "0x%.2x ", lost[i]);
					if( res > 0)
					{
						offset += res;
					}
					snprintf(buffer + offset, sizeof(buffer) - offset, "\n");
				}
				log_error("%s unknown data2 (%d) : %s", name, lost_count, buffer);
				lost_count = 0;
			}

			com->copy(msg, 4, header.size);
			serverTcp.write(msg, header.size);
			header.size += 4;
			com->skip(header.size);
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
	if( ! log )
	{
		res = -1;
		goto end;
	}
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
			log_info("%s%4s %13.6f    Fault\t%s (%d) : %s num %d status %d\033[0m", fault_color[state & 0x01], name, fault_list[i].time / 1000.0f, fault_description[i], i, fault_list[i].debugtext, state >> 1, state & 0x01);
			fault_status[i] = fault_list[i];
		}
	}

	pthread_mutex_unlock(&mutex);

end:
	return res;
}

int RobotInterface::process_code_version(char* msg, uint16_t size)
{
	int res = 0;
	int ret;

	if(size != sizeof(stm_code_version) || msg[40] != 0)
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

	memcpy(&stm_code_version, msg, sizeof(stm_code_version));

	ret = memcmp(stm_code_version, expected_version, sizeof(expected_version));
	if( !ret )
	{
		versionCompatible = ROBOT_VERSION_OK;
		log_info("stm_code_version compatible : %s", stm_code_version);
	}
	else
	{
		versionCompatible = ROBOT_VERSION_KO;
		log_error("stm_code_version not compatible : stm32 %s expected %s", stm_code_version, expected_version);
	}
	connected = true;

	pthread_mutex_unlock(&mutex);

end:
	return res;
}

int RobotInterface::process_detect_dyn_obj_size1(char* msg, uint16_t size)
{
	int res = 0;

	if(size != sizeof(detection_dynamic_object_count1))
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

	detection_dynamic_object_count1 = 0;
	memcpy(&detection_dynamic_object_count1, msg, sizeof(detection_dynamic_object_count1));

	detection_dynamic_object_size_tmp = detection_dynamic_object_count1;
	detection_dynamic_object_id = 0;
	detection_dynamic_object_pt_tmp_size = 0;
	if( detection_dynamic_object_count1 == 0)
	{
		detection_dynamic_object_size = 0;
	}

	pthread_mutex_unlock(&mutex);

end:
	return res;
}

int RobotInterface::process_detect_dyn_obj_size2(char* msg, uint16_t size)
{
	int res = 0;

	if(size != sizeof(detection_dynamic_object_count2))
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

	detection_dynamic_object_count2 = 0;
	memcpy(&detection_dynamic_object_count2, msg, sizeof(detection_dynamic_object_count2));

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

int RobotInterface::process_detect_obj1(char* msg, uint16_t size)
{
	int res = 0;

	if(size != sizeof(detection_obj1) )
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

	memcpy(detection_obj1, msg, sizeof(detection_obj1));

	pthread_mutex_unlock(&mutex);

end:
	return res;
}

int RobotInterface::process_detect_obj2(char* msg, uint16_t size)
{
	int res = 0;

	if(size != sizeof(detection_obj2) )
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

	memcpy(detection_obj2, msg, sizeof(detection_obj2));

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
		log_error("can msg size %d > 8", can_msg->size);
		can_msg->size = 8;
	}

	char buffer[1024];
	res = snprintf(buffer, sizeof(buffer), "%4s %13.6f %8s   id %6x format %d rtr %d size %u data", name, can_msg->time.ms/1000.0f + can_msg->time.ns/1000000000.0f,
			log_level_description[LOG_DEBUG1], (unsigned int)can_msg->id, can_msg->format, can_msg->type, can_msg->size);
	for(i=0; i < can_msg->size && res > 0; i++)
	{
		res += snprintf(buffer + res, sizeof(buffer) - res, " %2.2x", can_msg->data[i]);
	}
	log_info("%s", buffer);

	res = 0;

end:
	return res;
}

int RobotInterface::process_hokuyo(char* msg, uint16_t size)
{
	int res = 0;
	int id;

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

	id = ((struct hokuyo_scan*)msg)->id;
	memcpy(&hokuyo_scan[id], msg, size);

	hokuyo_compute_xy(&hokuyo_scan[id], detection_hokuyo_pos + HOKUYO_NUM_POINTS * id);

	pthread_mutex_unlock(&mutex);

end:
	return res;
}

int RobotInterface::process_hokuyo_seg(char* msg, uint16_t size)
{
	int res = 0;
	int id = HOKUYO1;
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

int RobotInterface::process_control_light(char* msg, uint16_t size)
{
	int res = 0;
	systime t;

	if(size != sizeof(struct control_usb_data_light) )
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

	// TODO pas propre, mettre en commun control_usb_data_light et control_usb_data
	memcpy(control_usb_data + control_usb_data_count, msg, size);
	last_control_usb_data = control_usb_data[control_usb_data_count];
	t = last_control_usb_data.current_time;
	current_time = t.ms / 1000.0f + t.ns / 1000000000.0f;
	control_usb_data_count = (control_usb_data_count + 1) % CONTROL_USB_DATA_MAX;

	for(int i = 0; i < AX12_MAX_ID; i++)
	{
		ax12[i].pos = (last_control_usb_data.dynamixel.ax12[i].pos - 0x1ff) * DYNAMIXEL_POS_TO_RD;
		ax12[i].flags = last_control_usb_data.dynamixel.ax12[i].flags;
		ax12[i].error = last_control_usb_data.dynamixel.ax12[i].error;
	}

	for(int i = 0; i < RX24_MAX_ID; i++)
	{
		rx24[i].pos = (last_control_usb_data.dynamixel.rx24[i].pos - 0x1ff) * DYNAMIXEL_POS_TO_RD;
		rx24[i].flags = last_control_usb_data.dynamixel.rx24[i].flags;
		rx24[i].error = last_control_usb_data.dynamixel.rx24[i].error;
	}

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
	last_control_usb_data = control_usb_data[control_usb_data_count];
	t = last_control_usb_data.current_time;
	current_time = t.ms / 1000.0f + t.ns / 1000000000.0f;
	control_usb_data_count = (control_usb_data_count + 1) % CONTROL_USB_DATA_MAX;

	for(int i = 0; i < AX12_MAX_ID; i++)
	{
		ax12[i].pos = (last_control_usb_data.dynamixel.ax12[i].pos - 0x1ff) * DYNAMIXEL_POS_TO_RD;
		ax12[i].flags = last_control_usb_data.dynamixel.ax12[i].flags;
		ax12[i].error = last_control_usb_data.dynamixel.ax12[i].error;
	}

	for(int i = 0; i < RX24_MAX_ID; i++)
	{
		rx24[i].pos = (last_control_usb_data.dynamixel.rx24[i].pos - 0x1ff) * DYNAMIXEL_POS_TO_RD;
		rx24[i].flags = last_control_usb_data.dynamixel.rx24[i].flags;
		rx24[i].error = last_control_usb_data.dynamixel.rx24[i].error;
	}

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

void RobotInterface::fault_reset()
{
	for(int i = 0; i < FAULT_MAX; i++)
	{
		fault_status[i].state = 0;
		fault_status[i].time = 0;
	}
}

int RobotInterface::usb_write(unsigned char cmd, void* data, int size)
{
	char buffer[256];

	if( size >= 254)
	{
		log_error("data size > 254 (%d)", size);
		return -1;
	}

	buffer[0] = cmd;
	buffer[1] = size + 2;
	if( size && data)
	{
		memcpy(buffer+2, data, size);
	}

	return com->write(buffer, buffer[1]);
}

int RobotInterface::ptask()
{
	return usb_write(USB_CMD_PTASK, NULL, 0);
}

int RobotInterface::reboot()
{
	connected = false;
	int res = usb_write(USB_CMD_REBOOT, NULL, 0);
	return res;
}

int RobotInterface::power_off(bool power_off)
{
	struct power_cmd_arg msg;
	if( power_off )
	{
		msg.power_off = 1;
	}
	else
	{
		msg.power_off = 0;
	}
	return usb_write(USB_CMD_POWER, &msg, sizeof(msg));
}

int RobotInterface::pwm_set(int id, float val)
{
	struct pwm_usb_cmd msg;
	msg.id = id;
	msg.val = val;
	return usb_write(USB_CMD_PWM, &msg, sizeof(msg));
}

int RobotInterface::get_stm_code_version()
{
	versionCompatible = ROBOT_VERSION_UNKNOWN;
	return usb_write(USB_CMD_GET_VERSION, NULL, 0);
}

//! fonction generique pour envoyer un ordre a un dynamixel
//! @return 0 s'il n'y a pas d'erreur d'envoi, -1 sinon
int RobotInterface::dynamixel_cmd(uint8_t cmd, int dynamixel_type, uint8_t id, float param)
{
	struct dynamixel_cmd_param cmd_arg;

	cmd_arg.cmd_id = cmd;
	cmd_arg.type = dynamixel_type;
	cmd_arg.id = id;
	cmd_arg.param = param;

	return usb_write(USB_CMD_DYNAMIXEL, &cmd_arg, sizeof(cmd_arg));
}

//! realise un scan de tout les id
//! @return 0 s'il n'y a pas d'erreur d'envoi, -1 sinon
int RobotInterface::dynamixel_scan(int dynamixel_type)
{
	return dynamixel_cmd(DYNAMIXEL_CMD_SCAN, dynamixel_type, 0, 0);
}

//! change l'id d'un dynamlixel
//! @return 0 s'il n'y a pas d'erreur d'envoi, -1 sinon
int RobotInterface::dynamixel_set_id(int dynamixel_type, uint8_t id, uint8_t new_id)
{
	return dynamixel_cmd(DYNAMIXEL_CMD_SET_ID, dynamixel_type, id, new_id);
}

//! envoi de la position desiree du dynamixel
//! @return 0 s'il n'y a pas d'erreur d'envoi, -1 sinon
int RobotInterface::dynamixel_set_goal_position(int dynamixel_type, uint8_t id, float alpha)
{
	return dynamixel_cmd(DYNAMIXEL_CMD_SET_GOAL_POSITION, dynamixel_type, id, alpha);
}

int RobotInterface::dynamixel_set_speed(int dynamixel_type, uint8_t id, float speed)
{
	return dynamixel_cmd(DYNAMIXEL_CMD_SET_SPEED, dynamixel_type, id, speed);
}

//! choix du couple max (entre 0 et 100 pour 100%)
//! @return 0 s'il n'y a pas d'erreur d'envoi, -1 sinon
int RobotInterface::dynamixel_set_max_torque(int dynamixel_type, uint8_t id, float val)
{
	return dynamixel_cmd(DYNAMIXEL_CMD_SET_MAX_TORQUE, dynamixel_type, id, val / 100.0f);
}

int RobotInterface::dynamixel_set_target_reached_threshold(int dynamixel_type, uint8_t id, float val)
{
	return dynamixel_cmd(DYNAMIXEL_CMD_SET_TARGET_REACHED_THRESHOLD, dynamixel_type, id, val);
}

int RobotInterface::dynamixel_enable_endless_turn_mode(int dynamixel_type, uint8_t id)
{
	return dynamixel_cmd(DYNAMIXEL_CMD_ENABLE_ENDLESS_TURN_MODE, dynamixel_type, id, 0);
}

int RobotInterface::dynamixel_disable_endless_turn_mode(int dynamixel_type, uint8_t id)
{
	return dynamixel_cmd(DYNAMIXEL_CMD_DISABLE_ENDLESS_TURN_MODE, dynamixel_type, id, 0);
}

//! mise a jour du baudrate du dynamixel a la valeur optimale (1Mb/s)
//! @return 0 s'il n'y a pas d'erreur d'envoi, -1 sinon
int RobotInterface::dynamixel_set_op_baudrate(int dynamixel_type, uint8_t id)
{
	return dynamixel_cmd(DYNAMIXEL_CMD_SET_BAUDRATE, dynamixel_type, id, 0);
}

int RobotInterface::dynamixel_set_manager_baudrate(int dynamixel_type, int freq)
{
	return dynamixel_cmd(DYNAMIXEL_CMD_SET_MANAGER_BAUDRATE, dynamixel_type, 0, freq);
}

int RobotInterface::dynamixel_get_position(int dynamixel_type, uint8_t id)
{
	return dynamixel_cmd(DYNAMIXEL_CMD_GET_POSITION, dynamixel_type, id, 0);
}

int RobotInterface::pump(uint8_t id, uint8_t val)
{
	struct pump_cmd_arg cmd_arg;

	cmd_arg.id = id;
	cmd_arg.val = val;

	return usb_write(USB_CMD_PUMP, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::motion_print_param()
{
	return usb_write(USB_CMD_MOTION_PRINT_PARAM, NULL, 0);
}

int RobotInterface::motion_set_param(float kp_av, float ki_av, float kd_av, float kp_rot, float ki_rot, float kd_rot)
{
	struct motion_cmd_param_arg cmd_arg;

	cmd_arg.kp_av = kp_av;
	cmd_arg.ki_av = ki_av;
	cmd_arg.kd_av = kd_av;
	cmd_arg.kp_rot = kp_rot;
	cmd_arg.ki_rot = ki_rot;
	cmd_arg.kd_rot = kd_rot;

	return usb_write(USB_CMD_MOTION_PARAM, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::motion_goto(VectPlan dest, VectPlan cp, enum motion_way way, enum motion_trajectory_type type, KinematicsParameters linearParam, KinematicsParameters angularParam)
{
	struct motion_cmd_goto_arg cmd_arg;

	cmd_arg.dest = dest;
	cmd_arg.cp = cp;
	cmd_arg.type = way;
	cmd_arg.way = type;
	cmd_arg.linearParam = linearParam;
	cmd_arg.angularParam = angularParam;

	return usb_write(USB_CMD_MOTION_GOTO, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::motion_set_speed(VectPlan u, float v)
{
	struct motion_cmd_set_speed_arg cmd_arg;

	cmd_arg.v = v;
	cmd_arg.u = u;

	return usb_write(USB_CMD_MOTION_SET_SPEED, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::motion_set_actuator_kinematics(struct motion_cmd_set_actuator_kinematics_arg cmd_arg)
{
	return usb_write(USB_CMD_MOTION_SET_ACTUATOR_KINEMATICS, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::straight(float dist)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.dist = dist;
	cmd_arg.type = TRAJECTORY_STRAIGHT;
	cmd_arg.avoidance_type = AVOIDANCE_STOP;

	return usb_write(USB_CMD_TRAJECTORY, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::rotate(float theta)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.dest.theta = theta;
	cmd_arg.type = TRAJECTORY_ROTATE;
	cmd_arg.avoidance_type = AVOIDANCE_STOP;

	return usb_write(USB_CMD_TRAJECTORY, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::rotate_to(float theta)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.dest.theta = theta;
	cmd_arg.type = TRAJECTORY_ROTATE_TO;
	cmd_arg.avoidance_type = AVOIDANCE_STOP;

	return usb_write(USB_CMD_TRAJECTORY, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::motion_enable(bool enable)
{
	struct motion_cmd_enable_arg cmd_arg;
	cmd_arg.enable = enable?1:0;

	return usb_write(USB_CMD_MOTION_ENABLE, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::motion_set_max_driving_current(float maxCurrent)
{
	struct motion_cmd_set_max_driving_current_arg cmd_arg;
	cmd_arg.maxDrivingCurrent = maxCurrent;

	return usb_write(USB_CMD_MOTION_SET_MAX_CURRENT, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::goto_near_xy(float x, float y, float dist, unsigned int way, unsigned int avoidance_type)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.dest.x = x;
	cmd_arg.dest.y = y;
	cmd_arg.dist = dist;
	cmd_arg.type = TRAJECTORY_GOTO_XY;
	cmd_arg.avoidance_type = avoidance_type;
	cmd_arg.way = way;

	return usb_write(USB_CMD_TRAJECTORY, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::goto_near(VectPlan dest, float dist, unsigned int way, unsigned int avoidance_type)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.dest = dest;
	cmd_arg.dist = dist;
	cmd_arg.type = TRAJECTORY_GOTO_XYA;
	cmd_arg.avoidance_type = avoidance_type;
	cmd_arg.way = way;

	return usb_write(USB_CMD_TRAJECTORY, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::goto_graph()
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.type = TRAJECTORY_GOTO_GRAPH;
	cmd_arg.avoidance_type = AVOIDANCE_STOP;

	return usb_write(USB_CMD_TRAJECTORY, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::goto_graph_node(int id)
{
	return goto_near_xy(graph_node[id].pos.x, graph_node[id].pos.y, 0, WAY_ANY, AVOIDANCE_GRAPH);
}

int RobotInterface::free()
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.type = TRAJECTORY_FREE;

	return usb_write(USB_CMD_TRAJECTORY, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::set_position(VectPlan pos)
{
	return usb_write(USB_CMD_LOCATION_SET_POSITION, &pos, sizeof(pos));
}

int RobotInterface::gyro_calibration(enum GyroCalibrationCmd cmd)
{
	int32_t cmd_arg = cmd;

	return usb_write(USB_CMD_GYRO_CALIB, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::gyro_stop_calibration(float theta)
{
	int res = gyro_set_position(theta);
	if( res )
	{
		return res;
	}
	return gyro_calibration(GYRO_CALIBRATION_STOP);
}

int RobotInterface::gyro_set_position(float theta)
{
	struct gyro_cmd_set_position_arg cmd_arg;
	cmd_arg.theta = theta;
	return usb_write(USB_CMD_GYRO_SET_POSITION, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::gyro_set_calibration_values(float scale, float bias, float dead_zone)
{
	struct gyro_cmd_set_calibration_values_arg cmd_arg;
	cmd_arg.scale = scale;
	cmd_arg.bias = bias;
	cmd_arg.dead_zone = dead_zone;

	return usb_write(USB_CMD_GYRO_SET_CALIBRATION_VALUES, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::wing(enum wing_cmd_type cmd_type_left, enum wing_cmd_type cmd_type_right)
{
	struct wing_cmd_arg cmd_arg;

	cmd_arg.type_left = cmd_type_left;
	cmd_arg.type_right = cmd_type_right;

	return usb_write(USB_CMD_WING, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::elevator_set_position(float pos)
{
	struct elevator_cmd_arg cmd_arg;
	cmd_arg.pos = pos;
	return usb_write(USB_CMD_ELEVATOR, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::finger_set_position(enum finger_type low, enum finger_type high, enum finger_bottom_type right, enum finger_bottom_type left)
{
	struct finger_cmd_arg cmd_arg;
	cmd_arg.low = low;
	cmd_arg.high = high;
	cmd_arg.right = right;
	cmd_arg.left = left;
	return usb_write(USB_CMD_FINGER, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::carpet_set_position(enum carpet_type right, enum carpet_type left)
{
	struct carpet_cmd_arg cmd_arg;
	cmd_arg.right = right;
	cmd_arg.left = left;
	return usb_write(USB_CMD_CARPET, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::arm_cmd(uint32_t cmdType)
{
	struct arm_cmd cmd_arg;

	cmd_arg.cmdType = cmdType;

	return usb_write(USB_CMD_ARM, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::arm_xyz(float x, float y, float z, enum arm_cmd_type type)
{
	(void) x;
	(void) y;
	(void) z;
	(void) type;
/*	struct arm_cmd_goto_param cmd_arg;

	if(type == ARM_CMD_ART)
	{
		return -1;
	}

	cmd_arg.x = x * 65536.0f;
	cmd_arg.y = y * 65536.0f;
	cmd_arg.z = z * 65536.0f;
	cmd_arg.type = type;

	return usb_write(USB_CMD_ARM_GOTO, &cmd_arg, sizeof(cmd_arg));*/
	return 0;
}

int RobotInterface::arm_ventouse(float x1, float y1, float x2, float y2, float z, int8_t tool_way)
{
	(void) x1;
	(void) y1;
	(void) x2;
	(void) y2;
	(void) z;
	(void) tool_way;
/*	struct arm_cmd_goto_param cmd_arg;

	cmd_arg.x1 = x1 * 65536.0f;
	cmd_arg.y1 = y1 * 65536.0f;
	cmd_arg.x2 = x2 * 65536.0f;
	cmd_arg.y2 = y2 * 65536.0f;
	cmd_arg.z = z * 65536.0f;
	cmd_arg.tool_way = tool_way;
	cmd_arg.type = ARM_CMD_VENTOUSE_ABS;

	return usb_write(USB_CMD_ARM_GOTO, &cmd_arg, sizeof(cmd_arg));*/
	return 0;
}

int RobotInterface::arm_abz(float a, float b, float z)
{
	(void) a;
	(void) b;
	(void) z;
/*	struct arm_cmd_goto_param cmd_arg;

	cmd_arg.a = a * (1 << 26) / (2 * M_PI);
	cmd_arg.b = b * (1 << 26) / (2 * M_PI);
	cmd_arg.z = z * 65536.0f;
	cmd_arg.type = ARM_CMD_ART;

	return usb_write(USB_CMD_ARM_GOTO, &cmd_arg, sizeof(cmd_arg));*/
	return 0;
}

int RobotInterface::recalage()
{
	struct gpio_cmd_match_arg cmd_arg;
	cmd_arg.cmd = MATCH_CMD_RECALAGE;
	return usb_write(USB_CMD_MATCH, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::go()
{
	struct gpio_cmd_match_arg cmd_arg;
	cmd_arg.cmd = MATCH_CMD_GO;
	return usb_write(USB_CMD_MATCH, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::go_enable()
{
	struct gpio_cmd_match_arg cmd_arg;
	cmd_arg.cmd = MATCH_CMD_ENABLE_GO;
	return usb_write(USB_CMD_MATCH, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::color(uint8_t color)
{
	return usb_write(USB_CMD_COLOR, &color, sizeof(color));
}

int RobotInterface::set_match_time(uint32_t time)
{
	return usb_write(USB_CMD_MATCH_TIME, &time, sizeof(time));
}

int RobotInterface::xbee_cmd(uint8_t cmd, float param)
{
	struct xbee_cmd_param cmd_arg;
	cmd_arg.cmd_id = cmd;
	cmd_arg.param = param;
	return usb_write(USB_CMD_XBEE, &cmd_arg, sizeof(cmd_arg));
}

int RobotInterface::xbee_set_op_baudrate()
{
	return xbee_cmd(XBEE_CMD_SET_OP_BAUDRATE, 0);
}

int RobotInterface::xbee_set_manager_baudrate(uint32_t baudrate)
{
	return xbee_cmd(XBEE_CMD_SET_MANAGER_BAUDRATE, baudrate);
}

int RobotInterface::can_set_baudrate(enum can_baudrate baudrate, int debug)
{
	char buffer[2];
	buffer[0] = baudrate;
	buffer[1] = debug;

	return usb_write(USB_CMD_CAN_SET_BAUDRATE, buffer, sizeof(buffer));

}

int RobotInterface::can_write(struct can_msg* msg)
{
	return usb_write(USB_CMD_CAN_WRITE, msg, sizeof(*msg));
}

int RobotInterface::can_lss(bool on)
{
	struct can_msg msg;
	msg.id = 0x7e5;
	msg.size = 8;
	msg.data[0] = 4;
	msg.data[1] = on?1:0;
	msg.data[2] = 0;
	msg.data[3] = 0;
	msg.data[4] = 0;
	msg.data[5] = 0;
	msg.data[6] = 0;
	msg.data[7] = 0;
	return can_write(&msg);
}

int RobotInterface::can_lss_set_nodeid(uint8_t nodeid)
{
	struct can_msg msg;
	msg.id = 0x7e5;
	msg.size = 8;
	msg.data[0] = 0x11;
	msg.data[1] = nodeid;
	msg.data[2] = 0;
	msg.data[3] = 0;
	msg.data[4] = 0;
	msg.data[5] = 0;
	msg.data[6] = 0;
	msg.data[7] = 0;
	return can_write(&msg);
}

int RobotInterface::can_lss_save()
{
	struct can_msg msg;
	msg.id = 0x7e5;
	msg.size = 8;
	msg.data[0] = 0x17;
	msg.data[1] = 0;
	msg.data[2] = 0;
	msg.data[3] = 0;
	msg.data[4] = 0;
	msg.data[5] = 0;
	msg.data[6] = 0;
	msg.data[7] = 0;
	return can_write(&msg);
}

int RobotInterface::set_max_speed(float vmax_av, float vmax_rot)
{
	struct motion_cmd_max_speed_arg cmd_arg;

	cmd_arg.vmax_av = fabsf(vmax_av);
	cmd_arg.vmax_rot = fabsf(vmax_rot);

	return usb_write(USB_CMD_CAN_WRITE, &cmd_arg, sizeof(cmd_arg));
}
