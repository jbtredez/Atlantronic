#ifndef FOO_INTERFACE_H
#define FOO_INTERFACE_H

#include <pthread.h>
#include "linux/tools/com.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/error_codes.h"
#include "kernel/error.h"
#include "foo/control/control.h"

#define CONTROL_USB_DATA_MAX        120000 //!< 600s (10 mn) de données avec l'asservissement à 200Hz
#define HOKUYO_FOO                     0
#define HOKUYO_FOO_BAR                 1
#define HOKUYO_BAR                     2

struct foo_interface
{
	struct com com; //!< communication
	pthread_mutex_t mutex; //!< mutex de protection des donnees ci-dessous
	void (*callback)(void*);
	void* callback_arg;

	// données brutes
	struct hokuyo_scan hokuyo_scan[3];
	struct control_usb_data control_usb_data[CONTROL_USB_DATA_MAX];
	int control_usb_data_count;
	struct error_status error_status[ERR_MAX];

	// calculs
	float hokuyo_x[HOKUYO_NUM_POINTS*3]; //!< x des points 44 à 725 - hokuyo 0 puis hokuyo 1
	float hokuyo_y[HOKUYO_NUM_POINTS*3]; //!< y des points 44 à 725 - hokuyo 0 puis hokuyo 1
};

int foo_interface_init(struct foo_interface* data, const char* file, void (*callback)(void*), void* callback_arg);

void foo_interface_destroy(struct foo_interface* data);

#endif

