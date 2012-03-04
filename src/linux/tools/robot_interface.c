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
#include "kernel/driver/usb.h"
#include "kernel/rcc.h"
#include "foo/control/control.h"
#include "foo/control/trajectory.h"
#include "foo/ax12.h"

const char* err_description[ERR_MAX] =
{
	// CAN
	[ERR_CAN_READ_QUEUE_FULL] = "can : queue de lecture pleine",
	[ERR_CAN_READ_FIFO_OVERFLOW] = "can : fifo de lecture qui deborde - perte de messages",
	[ERR_CAN_FILTER_LIST_FULL] = "can : filtre plein",

	// USART
	[ERR_USART_UNKNOWN_DEVICE] = "usart : id invalide",

	// HOKUYO
	[FAULT_HOKUYO_DISCONNECTED] = "hokuyo débranché",
	[FAULT_HOKUYO_DATA_CORRUPTION] = "hokuyo : erreur de transmission",
};

const char* log_level_description[LOG_MAX] =
{
	[LOG_ERROR] = "Error",
	[LOG_INFO] = "Info",
	[LOG_DEBUG1] = "Debug1",
	[LOG_DEBUG2] = "Debug2",
};

const char* cartes[COM_MAX] =
{
	"foo",
	"bar"
};

struct robot_interface_arg
{
	struct robot_interface* robot;
	int com_id;
};

static void* robot_interface_task(void* arg);
static int robot_interface_process_control(struct robot_interface* data, int com_id, char* msg, uint16_t size);
static int robot_interface_process_hokuyo(struct robot_interface* data, int com_id, int id, char* msg, uint16_t size);
static int robot_interface_process_hokuyo_seg(struct robot_interface* data, int com_id, int id, char* msg, uint16_t size);
static int robot_interface_process_log(struct robot_interface* data, int com_id, char* msg, uint16_t size);
static int robot_interface_process_err(struct robot_interface* data, int com_id, char* msg, uint16_t size);

int robot_interface_init(struct robot_interface* data, const char* file_foo, const char* file_bar, void (*callback)(void*), void* callback_arg)
{
	int i;
	int err = 0;
	int res = 0;

	data->callback = callback;
	data->callback_arg = callback_arg;
	data->stop_task = 1;

	if(file_foo)
	{
		com_init(&data->com[COM_FOO], file_foo);
	}
	else
	{
		com_init(&data->com[COM_FOO], "/dev/foo0");
	}

	if(file_bar)
	{
		com_init(&data->com[COM_BAR], file_bar);
	}
	else
	{
		com_init(&data->com[COM_BAR], "/dev/bar0");
	}

	data->control_usb_data_count = 0;
	memset(data->error_status, 0x00, sizeof(data->error_status));

	for(i = 0; i < HOKUYO_MAX ; i++)
	{
		data->detection_reg_num[i] = 0;
	}

	for( i = 0; i < COM_MAX ; i++)
	{
		struct robot_interface_arg* args = (struct robot_interface_arg*) malloc(sizeof(struct robot_interface_arg));
		args->robot = data;
		args->com_id = i;
		data->stop_task = 0;
		res = pthread_create(&data->tid, NULL, robot_interface_task, args);
		if(res)
		{
			err = res;
			log_error_errno("pthread_create");
		}
	}

	return err;
}

void robot_interface_destroy(struct robot_interface* data)
{
	int i;

	data->stop_task = 1;
	usleep(100000);
	pthread_cancel(data->tid);

	for( i = 0; i < COM_MAX; i++)
	{
		com_close(&data->com[i]);
		com_destroy(&data->com[i]);
	}
	rl_free_line_state();
	rl_cleanup_after_signal();
}

static void* robot_interface_task(void* arg)
{
	struct robot_interface_arg* args = (struct robot_interface_arg*) arg;
	struct robot_interface* robot = args->robot;
	struct com* com = &robot->com[args->com_id];
	int res;
	unsigned char lost[1024];
	unsigned int lost_count = 0;

	com_open_block(com);

	while( !robot->stop_task)
	{
		uint16_t type;
		uint16_t size;

		// lecture entete
		res = com_read_header(com, &type, &size);
		if( robot->stop_task)
		{
			goto end;
		}

		if( res )
		{
			com_open_block(com);
			continue;
		}

		// lecture du message
		res = com_read(com, size + 4);
		if( robot->stop_task)
		{
			goto end;
		}

		if( res )
		{
			com_open_block(com);
			continue;
		}

		// copie du message (vers un buffer non circulaire)
		char msg[size+1];
		com_copy_msg(com, msg, size+1);

		// traitement du message
		switch( type )
		{
			case USB_LOG:
				res = robot_interface_process_log(robot, args->com_id, msg, size);
				break;
			case USB_ERR:
				res = robot_interface_process_err(robot, args->com_id, msg, size);
				break;
			case USB_HOKUYO_FOO:
				res = robot_interface_process_hokuyo(robot, args->com_id, HOKUYO_FOO, msg, size);
				break;
			case USB_HOKUYO_BAR:
				res = robot_interface_process_hokuyo(robot, args->com_id, HOKUYO_BAR, msg, size);
				break;
			case USB_HOKUYO_FOO_SEG:
				res = robot_interface_process_hokuyo_seg(robot, args->com_id, HOKUYO_FOO, msg, size);
				break;
			case USB_CONTROL:
				res = robot_interface_process_control(robot, args->com_id, msg, size);
				break;
			default:
				res = -1;
				break;
		}

		if( res )
		{
			unsigned char byte = com->buffer[com->buffer_begin];
			if(lost_count >= sizeof(lost)-2)
			{
				lost[lost_count+1] = 0;
				log_error("%s unknown data : %s", cartes[args->com_id], lost);
				lost_count = 0;
			}

			lost[lost_count] = byte;
			lost_count ++;
			if(byte == 0 || byte == '\n')
			{
				lost[lost_count+1] = 0;
				log_error("%s unknown data : %s", cartes[args->com_id], lost);
				lost_count = 0;
			}

			//printf("wrong format, type : %i, size = %i, - skip %#.2x (%c)\n", type, size, com->buffer[com->buffer_begin], com->buffer[com->buffer_begin]);
			com_skip(com, 1);
		}
		else
		{
			if(lost_count)
			{
				lost[lost_count+1] = 0;
				log_error("%s unknown data : %s", cartes[args->com_id], lost);
				lost_count = 0;
			}

			size += 4;
			com_skip(com, size);
			if(robot->callback)
			{
				robot->callback(robot->callback_arg);
			}
		}
	}

end:
	return NULL;
}

static int robot_interface_process_log(struct robot_interface* data, int com_id, char* msg, uint16_t size)
{
	int res = 0;
	(void) data;

	if( msg[size-1] != '\n' )
	{
		res = -1;
		goto end;
	}

	uint64_t current_time;
	memcpy(&current_time, msg, 8);

	msg[size-1] = 0;

	unsigned char level = msg[8];
	uint16_t line;
	memcpy(&line, msg + 9, 2);

	char* log = strchr(msg+11, ':');
	*log = 0;
	log++;

	if(level < LOG_MAX)
	{
		log_info("%4s %12lu %8s   %20s:%5d    %s", cartes[com_id], (unsigned long) tick_to_us(current_time), log_level_description[level], msg+11, line, log);
	}
	else
	{
		log_info("%4s %12lu %8s   %20s:%5d    %s", cartes[com_id], (unsigned long) tick_to_us(current_time), "unknown", msg+11, line, log);
	}

end:
	return res;
}

static int robot_interface_process_err(struct robot_interface* data, int com_id, char* msg, uint16_t size)
{
	int res = 0;
	int i = 0;
	struct error_status* err_list = (struct error_status*) msg;

	if(size != sizeof(struct error_status) * ERR_MAX)
	{
		res = -1;
		goto end;
	}

	res = pthread_mutex_lock(&data->mutex);

	if(res)
	{
		log_error("pthread_mutex_lock : %i", res);
		goto end;
	}

	for(i=0; i< ERR_MAX; i++)
	{
		unsigned char state = err_list[i].state;
		if(state != data->error_status[com_id][i].state)
		{
			if( state == 0)
			{
				log_info("\033[32m%4s %12lu    Fault\t%s (%d), status %d\033[0m", cartes[com_id], (unsigned long) tick_to_us(err_list[i].time), err_description[i], i, state);
			}
			else
			{
				log_info("\033[31m%4s %12lu    Fault\t%s (%d), status %d\033[0m", cartes[com_id], (unsigned long) tick_to_us(err_list[i].time), err_description[i], i, state);
			}
			data->error_status[com_id][i] = err_list[i];
		}
	}

	pthread_mutex_unlock(&data->mutex);

end:
	return res;
}

static int robot_interface_process_hokuyo(struct robot_interface* data, int com_id, int id, char* msg, uint16_t size)
{
	(void) com_id;
	int res = 0;

	if(size != sizeof(struct hokuyo_scan))
	{
		res = -1;
		goto end;
	}

	res = pthread_mutex_lock(&data->mutex);

	if(res)
	{
		log_error("pthread_mutex_lock : %i", res);
		goto end;
	}

	memcpy(&data->hokuyo_scan[id], msg, size);

	hokuyo_compute_xy(&data->hokuyo_scan[id], data->detection_hokuyo_pos + HOKUYO_NUM_POINTS * id);

	pthread_mutex_unlock(&data->mutex);

end:
	return res;
}

static int robot_interface_process_hokuyo_seg(struct robot_interface* data, int com_id, int id, char* msg, uint16_t size)
{
	int res = 0;

	(void) com_id;

	unsigned int num = size / sizeof(data->detection_hokuyo_reg[0]);
	if(size != num * sizeof(data->detection_hokuyo_reg[0]) )
	{
		res = -1;
		goto end;
	}

	if( num > HOKUYO_NUM_POINTS)
	{
		res = -1;
		goto end;
	}

	res = pthread_mutex_lock(&data->mutex);

	if(res)
	{
		log_error("pthread_mutex_lock : %i", res);
		goto end;
	}

	data->detection_reg_num[id] = num;
	memcpy(data->detection_hokuyo_reg + HOKUYO_NUM_POINTS * id, msg, size);

	pthread_mutex_unlock(&data->mutex);

end:
	return res;
}

static int robot_interface_process_control(struct robot_interface* data, int com_id, char* msg, uint16_t size)
{
	int res = 0;
	(void) com_id;

	if(size != sizeof(struct control_usb_data) )
	{
		res = -1;
		goto end;
	}

	res = pthread_mutex_lock(&data->mutex);

	if(res)
	{
		log_error("pthread_mutex_lock : %i", res);
		goto end;
	}

	memcpy(data->control_usb_data + data->control_usb_data_count, msg, size);
	data->control_usb_data_count = (data->control_usb_data_count + 1) % CONTROL_USB_DATA_MAX;

	pthread_mutex_unlock(&data->mutex);

end:
	return res;
}

int robot_interface_ax12_scan(struct robot_interface* data)
{
	struct ax12_cmd_param cmd_arg;

	cmd_arg.cmd_id = AX12_CMD_SCAN;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_AX12;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_ax12_set_id(struct robot_interface* data, uint8_t id, uint8_t new_id)
{
	struct ax12_cmd_param cmd_arg;

	cmd_arg.cmd_id = AX12_CMD_SET_ID;
	cmd_arg.id = id;
	cmd_arg.param = new_id;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_AX12;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_ax12_set_goal_position(struct robot_interface* data, uint8_t id, uint16_t pos)
{
	struct ax12_cmd_param cmd_arg;

	cmd_arg.cmd_id = AX12_CMD_SET_GOAL_POSITION;
	cmd_arg.id = id;
	cmd_arg.param = pos;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_AX12;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_control_print_param(struct robot_interface* data)
{
	char buffer[1];
	buffer[0] = USB_CMD_CONTROL_PRINT_PARAM;

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_control_set_param(struct robot_interface* data, int kp_av, int ki_av, int kd_av, int kp_rot, int ki_rot, int kd_rot, int kx, int ky, int kalpha)
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

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_straight(struct robot_interface* data, float dist)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.dist = dist * 65536.0f;
	cmd_arg.type = TRAJECTORY_STRAIGHT;
	cmd_arg.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_straight_to_wall(struct robot_interface* data, float dist)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.dist = dist * 65536.0f;
	cmd_arg.type = TRAJECTORY_STRAIGHT_TO_WALL;
	cmd_arg.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_rotate(struct robot_interface* data, float alpha)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.alpha = alpha * (1 << 26) / (2 * M_PI);
	cmd_arg.type = TRAJECTORY_ROTATE;
	cmd_arg.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_rotate_to(struct robot_interface* data, float alpha)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.alpha = alpha * (1 << 26) / (2 * M_PI);
	cmd_arg.type = TRAJECTORY_ROTATE_TO;
	cmd_arg.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_free(struct robot_interface* data)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.type = TRAJECTORY_FREE;
	cmd_arg.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_goto_near_xy(struct robot_interface* data, float x, float y, float dist, unsigned int way, unsigned int avoidance_type)
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

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_goto_near(struct robot_interface* data, float x, float y, float alpha, float dist, unsigned int way, unsigned int avoidance_type)
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

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_goto_graph(struct robot_interface* data)
{
	struct trajectory_cmd_arg cmd_arg;

	cmd_arg.type = TRAJECTORY_GOTO_GRAPH;
	cmd_arg.avoidance_type = TRAJECTORY_AVOIDANCE_STOP;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_TRAJECTORY;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_set_position(struct robot_interface* data, float x, float y, float alpha)
{
	struct location_cmd_arg cmd_arg;

	cmd_arg.x = x * 65536.0f;
	cmd_arg.y = y * 65536.0f;
	cmd_arg.alpha = alpha  * (1 << 26) / (2 * M_PI);

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_LOCATION_SET_POSITION;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_pince(struct robot_interface* data, enum pince_cmd_type cmd_type)
{
	struct pince_cmd_arg cmd_arg;

	cmd_arg.type = cmd_type;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_PINCE;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_set_max_speed(struct robot_interface* data, float vmax_av, float vmax_rot)
{
	struct control_cmd_max_speed_arg cmd_arg;

	cmd_arg.vmax_av = fabsf(vmax_av) * 65536.0f;
	cmd_arg.vmax_rot = fabsf(vmax_rot) * 65536.0f;

	char buffer[1+sizeof(cmd_arg)];
	buffer[0] = USB_CMD_CONTROL_MAX_SPEED;
	memcpy(buffer+1, &cmd_arg, sizeof(cmd_arg));

	return com_write(&data->com[COM_FOO], buffer, sizeof(buffer));
}

int robot_interface_rotate_speed(struct robot_interface* data, float v)
{
	enum control_state control_state = CONTROL_READY_FREE;
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
			control_state = data->control_usb_data[data->control_usb_data_count-1].control_state;
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

	return 0;
}

int robot_interface_straight_speed(struct robot_interface* data, float v)
{
	enum control_state control_state = CONTROL_READY_FREE;
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
			control_state = data->control_usb_data[data->control_usb_data_count-1].control_state;
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

	return 0;
}