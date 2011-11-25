#ifndef FOO_INTERFACE_H
#define FOO_INTERFACE_H

#include <pthread.h>
#include "linux/tools/com.h"
#include "kernel/driver/hokuyo.h"
#include "foo/control/control.h"

#define CONTROL_USB_DATA_MAX        18000 //!< 90s de données avec l'asservissement à 200Hz

struct foo_interface
{
	struct com com; //!< communication
	pthread_mutex_t mutex; //!< mutex de protection des donnees ci-dessous
	void (*callback)(void*);
	void* callback_arg;

	// données brutes
	struct hokuyo_scan hokuyo_scan;
	struct control_usb_data control_usb_data[CONTROL_USB_DATA_MAX];
	int control_usb_data_count;

	// calculs
	float hokuyo_x[682]; //!< x des points 44 à 725
	float hokuyo_y[682]; //!< y des points 44 à 725
};

int foo_interface_init(struct foo_interface* data, const char* file, void (*callback)(void*), void* callback_arg);

void foo_interface_destroy(struct foo_interface* data);

#endif

