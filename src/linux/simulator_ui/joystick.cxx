#include "linux/simulator_ui/joystick.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include "linux/usb_interface_common/cli.h"

void* joystick_task(void* arg);

int joystick_init(struct joystick* joy, const char* dev, void (*event_callback)(int event, float val))
{
	joy->axes = 0;
	joy->btn = 0;
	joy->range_min = 0;
	joy->range_max = 0;
	joy->corr = 0;
	joy->event_callback = event_callback;
	joy->tid = -1;
	joy->stop_task = 1;

	if( !event_callback)
	{
		return -1;
	}

	joy->fd = open(dev,O_RDONLY);

	if(joy->fd < 0)
	{
		log_error_errno("open");
		return -1;
	}

	// recuperation du nombre d'axes
	ioctl(joy->fd, JSIOCGAXES, &joy->axes_num);

	// recuperation du nombre de boutons
	ioctl(joy->fd, JSIOCGBUTTONS, &joy->btn_num);

	joy->axes = (int*) malloc(joy->axes_num * sizeof(int));
	joy->range_min = (int*) malloc(joy->axes_num * sizeof(int));
	joy->range_max = (int*) malloc(joy->axes_num * sizeof(int));
	joy->btn = (char*) malloc(joy->btn_num * sizeof(char));
	joy->corr = (struct js_corr*) malloc(joy->axes_num * sizeof(struct js_corr));

	// recuperation des corrections des axes
	ioctl(joy->fd, JSIOCGCORR, joy->corr);

	// nom du joystick ( debug)
	char joy_name[80];
	ioctl(joy->fd, JSIOCGNAME(80), &joy_name);
	log_info("joystick %s détecté - %d axes et %d btn", joy_name, joy->axes_num, joy->btn_num);
	int i;
	for(i = 0; i < joy->axes_num; i++)
	{
// debug
#if 0
		log_info("axe %d : type %d precision %d", i, joy->corr[i].type, joy->corr[i].prec, joy->corr[i].coef[0], joy->corr[i].coef[1], joy->corr[i].coef[2],
			joy->corr[i].coef[3], joy->corr[i].coef[4], joy->corr[i].coef[5], joy->corr[i].coef[6], joy->corr[i].coef[7]);
#endif
		if(joy->corr[i].type == JS_CORR_BROKEN)
		{
			int center_min = joy->corr[i].coef[0];
			int center_max = joy->corr[i].coef[1];
			int dir = 1;
			if(joy->corr[i].coef[2] < 0)
			{
				joy->corr[i].coef[2] = -joy->corr[i].coef[2];
				dir = -1;
			}
			if(joy->corr[i].coef[3] < 0)
			{
				joy->corr[i].coef[3] = -joy->corr[i].coef[3];
				dir = -1;
			}
			joy->range_min[i] = rint(center_min - ((32767.0 * 16384) / joy->corr[i].coef[2]));
			joy->range_max[i] = rint((32767.0 * 16384) / joy->corr[i].coef[3] + center_max);

			log_info("axe %d type %2d precision %3d range = [%8d ; %8d] center = [%8d ; %8d] direction %d", i, joy->corr[i].type, joy->corr[i].prec, joy->range_min[i], joy->range_max[i], center_min, center_max, dir);
		}
		else
		{
			log_info("axe %d type %d - non gere", i, joy->corr[i].type);
		}
	}


	joy->stop_task = 0;

	pthread_create(&joy->tid, NULL, joystick_task, joy);

	return 0;
}

void* joystick_task(void* arg)
{
	struct joystick* joy = (struct joystick*) arg;
	struct js_event js;
	struct timeval tv;
	fd_set set;

	while(!joy->stop_task)
	{
		FD_ZERO(&set);
		FD_SET(joy->fd, &set);
		tv.tv_sec = 0;
		tv.tv_usec = 500000;

		int res = select(joy->fd + 1, &set, NULL, NULL, &tv);
		if( res < 0)
		{
			log_error_errno("select");
			continue;
		}

		// timeout
		if( res == 0)
		{
			continue;
		}

		if(FD_ISSET(joy->fd, &set))
		{
			res = read(joy->fd, &js, sizeof(js));
			if( res != sizeof(js) )
			{
				log_error_errno("read");
				continue;
			}

			switch(js.type & ~JS_EVENT_INIT)
			{
				case JS_EVENT_AXIS:
					if(joy->corr[js.number].type == JS_CORR_BROKEN)
					{
						int val = 0;
						if( js.value < joy->corr[js.number].coef[0])
						{
							val = rint(joy->corr[js.number].coef[0] + (16384.0f * js.value) / joy->corr[js.number].coef[2]);
						}
						else if(js.value > joy->corr[js.number].coef[1])
						{
							val = rint(joy->corr[js.number].coef[1] + (16384.0f * js.value) / joy->corr[js.number].coef[3]);
						}

						if(joy->axes[js.number] != val)
						{
							double v;
							joy->axes[js.number] = val;
							if( val < 0)
							{
								v = - ((float)val) / joy->range_min[js.number];
							}
							else
							{
								v = ((float)val) / joy->range_max[js.number];
							}
							joy->event_callback(js.number, v);
						}
					}
					break;
				case JS_EVENT_BUTTON:
					if(joy->btn[js.number] != js.value)
					{
						joy->btn[js.number] = js.value;
						joy->event_callback(0xFF00|(js.number&0xFF),js.value);
					}
					break;
			}
		}
	}

	return NULL;
}

void joystick_destroy(struct joystick* joy)
{
	void* res;
	if( ! joy->stop_task)
	{
		joy->stop_task = 1;
		pthread_join(joy->tid, &res);
	}

	if(joy->axes)
	{
		free(joy->axes);
	}

	if(joy->btn)
	{
		free(joy->btn);
	}

	if(joy->range_min)
	{
		free(joy->range_min);
	}

	if(joy->range_max)
	{
		free(joy->range_max);
	}

	if(joy->corr)
	{
		free(joy->corr);
	}
}