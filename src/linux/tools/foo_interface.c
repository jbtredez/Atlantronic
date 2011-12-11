#include <string.h>
#include <pthread.h>
#include "linux/tools/foo_interface.h"
#include "kernel/hokuyo_tools.h"
#include "linux/tools/com.h"
#include "kernel/driver/usb.h"
#include "kernel/rcc.h"
#include "linux/tools/cli.h"

const char* err_description[ERR_MAX] =
{
	// CAN
	[ERR_CAN_READ_QUEUE_FULL] = "can : queue de lecture pleine",
	[ERR_CAN_READ_FIFO_OVERFLOW] = "can : fifo de lecture qui deborde - perte de messages",
	[ERR_CAN_FILTER_LIST_FULL] = "can : filtre plein",

	// USART
	[ERR_USART_UNKNOWN_DEVICE] = "usart : id invalide",

	// AX12
	[ERR_AX12_DISCONNECTED] = "ax12 deconnecté",
	[ERR_AX12_USART_FE] = "ax12 : desynchro, bruit ou octet \"break\" sur l'usart",
	[ERR_AX12_USART_NE] = "ax12 : bruit sur l'usart",
	[ERR_AX12_USART_ORE] = "ax12 : overrun sur l'usart",
	[ERR_AX12_SEND_CHECK] = "ax12 : échec de la verification des octets envoyés",
	[ERR_AX12_PROTO] = "ax12 : erreur protocole",
	[ERR_AX12_CHECKSUM] = "ax12 : somme de verification incompatible",
	[ERR_AX12_INTERNAL_ERROR] = "ax12 : erreur interne",
	[ERR_AX12_INTERNAL_ERROR_INPUT_VOLTAGE] = "ax12 : erreur interne - problème de tension",
	[ERR_AX12_INTERNAL_ERROR_ANGLE_LIMIT] = "ax12 : erreur interne - angle invalide",
	[ERR_AX12_INTERNAL_ERROR_OVERHEATING] = "ax12 : erreur interne - surchauffe",
	[ERR_AX12_INTERNAL_ERROR_RANGE] = "ax12 : erreur interne - valeur non admissible",
	[ERR_AX12_INTERNAL_ERROR_CHECKSUM] = "ax12 : erreur interne - somme de verification incompatible",
	[ERR_AX12_INTERNAL_ERROR_OVERLOAD] = "ax12 : erreur interne - surcharge de l'actioneur",
	[ERR_AX12_INTERNAL_ERROR_INSTRUCTION] = "ax12 : erreur interne - instruction invalide",

	// HOKUYO
	[ERR_HOKUYO_DISCONNECTED] = "hokuyo débranché",
	[ERR_HOKUYO_USART_FE] = "hokuyo : desynchro, bruit ou octet \"break\" sur l'usart",
	[ERR_HOKUYO_USART_NE] = "hokuyo : bruit sur l'usart",
	[ERR_HOKUYO_USART_ORE] = "hokuyo : overrun sur l'usart",
	[ERR_HOKUYO_CHECK_CMD] = "hokuyo : échec de la vérification de la commande",
	[ERR_HOKUYO_UNKNOWN_STATUS] = "hokuyo : état inconnu",
	[ERR_HOKUYO_CHECKSUM] = "hokuyo : somme de vérification invalide",
	[ERR_HOKUYO_BAUD_RATE] = "hokuyo : vitesse invalide",
	[ERR_HOKUYO_LASER_MALFUNCTION] = "hokuyo : disfonctionnement du laser",
	[ERR_HOKUYO_SCAN_SIZE] = "hokuyo_tools_decode_buffer : buffer du scan trop petit",
	[ERR_HOKUYO_DISTANCE_BUFFER] = "hokuyo_tools_decode_buffer : buffer distance trop petit",
};

static void* foo_interface_task(void* arg);
static int foo_interface_process_control(struct foo_interface* data, char* msg, uint16_t size);
static int foo_interface_process_hokuyo(struct foo_interface* data, int id, char* msg, uint16_t size);
static int foo_interface_process_log(struct foo_interface* data, char* msg, uint16_t size);
static int foo_interface_process_err(struct foo_interface* data, char* msg, uint16_t size);

int foo_interface_init(struct foo_interface* data, const char* file, void (*callback)(void*), void* callback_arg)
{
	pthread_t tid;

	data->callback = callback;
	data->callback_arg = callback_arg;
	com_init(&data->com, file);
	data->control_usb_data_count = 0;
	memset(data->error_status, 0x00, sizeof(data->error_status));

	return pthread_create(&tid, NULL, foo_interface_task, data);
}

void foo_interface_destroy(struct foo_interface* data)
{
	com_close(&data->com);
	com_destroy(&data->com);
	rl_free_line_state();
	rl_cleanup_after_signal();
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
			case USB_ERR:
				res = foo_interface_process_err(foo, msg, size);
				break;
			case USB_HOKUYO_FOO:
				res = foo_interface_process_hokuyo(foo, HOKUYO_FOO, msg, size);
				break;
			case USB_HOKUYO_BAR:
				res = foo_interface_process_hokuyo(foo, HOKUYO_BAR, msg, size);
				break;
			case USB_HOKUYO_FOO_BAR:
				res = foo_interface_process_hokuyo(foo, HOKUYO_FOO_BAR, msg, size);
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
			printf("wrong format, type : %i, size = %i, - skip %#.2x (%c)\n", type, size, foo->com.buffer[foo->com.buffer_begin], foo->com.buffer[foo->com.buffer_begin]);
			com_skip(&foo->com, 1);
		}
		else
		{
			size += 4;
			com_skip(&foo->com, size);
			if(foo->callback)
			{
				foo->callback(foo->callback_arg);
			}
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

static int foo_interface_process_err(struct foo_interface* data, char* msg, uint16_t size)
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
		if(state != data->error_status[i].state)
		{
			if( state == 0)
			{
				log_info("\033[32m%12lu\tError\t%s (%d), status %d\033[0m", (unsigned long) tick_to_us(err_list[i].time), err_description[i], i, state);
			}
			else
			{
				log_info("\033[31m%12lu\tError\t%s (%d), status %d\033[0m", (unsigned long) tick_to_us(err_list[i].time), err_description[i], i, state);
			}
			data->error_status[i] = err_list[i];
		}
	}

	pthread_mutex_unlock(&data->mutex);

end:
	return res;
}

static int foo_interface_process_hokuyo(struct foo_interface* data, int id, char* msg, uint16_t size)
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

	memcpy(&data->hokuyo_scan[id], msg, size);

	hokuyo_compute_xy(data->hokuyo_scan[id].distance, HOKUYO_NUM_POINTS, data->hokuyo_x + HOKUYO_NUM_POINTS * id, data->hokuyo_y + HOKUYO_NUM_POINTS * id, data->hokuyo_scan[id].sens);

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
