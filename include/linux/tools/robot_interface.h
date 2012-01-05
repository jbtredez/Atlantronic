#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <pthread.h>
#include "linux/tools/com.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/error_codes.h"
#include "kernel/error.h"
#include "foo/control/control.h"

#define CONTROL_USB_DATA_MAX        120000 //!< 600s (10 mn) de données avec l'asservissement à 200Hz

enum
{
	HOKUYO_FOO,
	HOKUYO_FOO_BAR,
	HOKUYO_BAR,
	HOKUYO_MAX,
};

enum
{
	COM_FOO,
	COM_BAR,
	COM_MAX
};

struct robot_interface
{
	struct com com[COM_MAX]; //!< communication
	pthread_mutex_t mutex; //!< mutex de protection des donnees ci-dessous
	void (*callback)(void*);
	void* callback_arg;

	// données brutes
	struct hokuyo_scan hokuyo_scan[3];
	struct control_usb_data control_usb_data[CONTROL_USB_DATA_MAX];
	int control_usb_data_count;
	struct error_status error_status[COM_MAX][ERR_MAX];

	// calculs
	struct vect_pos hokuyo_pos[HOKUYO_NUM_POINTS*3];
	char detection_seg[HOKUYO_NUM_POINTS*3];
};

int robot_interface_init(struct robot_interface* data, const char* file_foo, const char* file_bar, void (*callback)(void*), void* callback_arg);

void robot_interface_destroy(struct robot_interface* data);

#endif