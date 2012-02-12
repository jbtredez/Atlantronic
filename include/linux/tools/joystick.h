#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <pthread.h>
#include <linux/joystick.h>

#define JOYSTICK_BTN_BASE        0xFF00

struct joystick
{
	int fd;               //!< descripteur de fichier
	unsigned char axes_num; //!< nombre d'axes (ATTENTION : le type est important pour l'ioctl)
	unsigned char btn_num; //!< nombre de btn (ATTENTION : le type est important pour l'ioctl)
	int* axes; //!< valeurs des axes
	int* range_min;
	int* range_max;
	char* btn; //!< valeurs des btn
	struct js_corr* corr; //!< corrections

	pthread_t tid;
	volatile int stop_task;
	void (*event_callback)(int event, float val); //!< fonction de traitement des axes
};

int joystick_init(struct joystick* joy, const char* dev, void (*event_callback)(int event, float val));

void joystick_destroy(struct joystick* joy);

#endif