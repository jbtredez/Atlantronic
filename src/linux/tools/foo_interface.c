#include <string.h>
#include <pthread.h>
#include "linux/tools/foo_interface.h"
#include "kernel/hokuyo_tools.h"
#include "linux/tools/com.h"
#include "kernel/driver/usb.h"
#include "linux/tools/cli.h"

static void* foo_interface_task(void* arg);
static int foo_interface_process_control(struct foo_interface* data, char* msg, uint16_t size);
static int foo_interface_process_hokuyo(struct foo_interface* data, char* msg, uint16_t size);
static int foo_interface_process_log(struct foo_interface* data, char* msg, uint16_t size);

int foo_interface_init(struct foo_interface* data, const char* file)
{
	pthread_t tid;

	com_init(&data->com, file);
	data->control_usb_data_count = 0;

	return pthread_create(&tid, NULL, foo_interface_task, data);
}

static void* foo_interface_task(void* arg)
{
	struct foo_interface* foo = (struct foo_interface*) arg;
	int res;

	com_open_block(&foo->com);

	while(1)
	{
		uint16_t type;
		uint16_t size;

		// lecture entete
		res = com_read_header(&foo->com, &type, &size);
		if( res )
		{
			com_open_block(&foo->com);
			continue;
		}

		// lecture du message
		res = com_read(&foo->com, size + 4);
		if( res )
		{
			com_open_block(&foo->com);
			continue;
		}

		// copie du message (vers un buffer non circulaire)
		char msg[size+1];
		com_copy_msg(&foo->com, msg, size+1);

		// traitement du message
		switch( type )
		{
			case USB_LOG:
				res = foo_interface_process_log(foo, msg, size);
				break;
			case USB_HOKUYO:
				res = foo_interface_process_hokuyo(foo, msg, size);
				break;
			case USB_CONTROL:
				res = foo_interface_process_control(foo, msg, size);
				break;
			default:
				res = -1;
				break;
		}

		if( res )
		{
//			printf("wrong format, type : %i, size = %i, - skip %#.2x (%c)\n", type, size, foo.buffer[foo.buffer_begin], foo.buffer[foo.buffer_begin]);
			com_skip(&foo->com, 1);
		}
		else
		{
			size += 4;
			com_skip(&foo->com, size);
		}
	}

	return NULL;
}

static int foo_interface_process_log(struct foo_interface* data, char* msg, uint16_t size)
{
	int res = 0;
	(void) data;

	if( msg[size-1] != '\n' )
	{
		res = -1;
		goto end;
	}

	msg[size-1] = 0;

	log_info("%s", msg);

end:
	return res;
}

static int foo_interface_process_hokuyo(struct foo_interface* data, char* msg, uint16_t size)
{
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

	memcpy(&data->hokuyo_scan, msg, size);

	hokuyo_compute_xy(data->hokuyo_scan.distance, HOKUYO_NUM_POINTS, data->hokuyo_x, data->hokuyo_y, -1);

	pthread_mutex_unlock(&data->mutex);

end:
	return res;
}

static int foo_interface_process_control(struct foo_interface* data, char* msg, uint16_t size)
{
	int res = 0;

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
