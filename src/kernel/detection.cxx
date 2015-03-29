#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/hokuyo_tools.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/robot_parameters.h"
#include "kernel/math/regression.h"
#include "kernel/math/segment_intersection.h"
#include "kernel/math/polyline.h"
#include "kernel/location/location.h"
#include "table.h"
#include "detection.h"

//! @todo réglage au pif
#define DETECTION_STACK_SIZE         600
#define DETECTION_QUEUE_SIZE          20
#define HOKUYO_REG_SEG               200
#define SIMILARITY_ACCEPTANCE        200

enum
{
	DETECTION_EVENT_HOKUYO_1,
	DETECTION_EVENT_HOKUYO_2,
};

static void detection_task(void* arg);
static int detection_module_init();
static void detection_compute(int id);
static void detection_remove_static_elements_from_dynamic_list(int id);
static float detection_get_segment_similarity(const Vect2* a, const Vect2* b, const Vect2* m, const Vect2* n);
static void detection_hokuyo1_callback();
static void detection_hokuyo2_callback();
static xQueueHandle detection_queue;
static detection_callback detection_callback_function = (detection_callback)nop_function;

// données privées à la tache detection
static Vect2 detection_hokuyo_pos[HOKUYO_NUM_POINTS];
static int detection_reg_ecart = 25;

// données partagées par la tache et des méthodes d'accés
static xSemaphoreHandle detection_mutex;
static Vect2 detection_hokuyo_reg[HOKUYO_REG_SEG];
static int detection_reg_size;
static struct polyline detection_object_polyline[DETECTION_NUM_OBJECT]; // TODO aligner
static struct detection_object detection_obj1[DETECTION_NUM_OBJECT]; // TODO aligner
static struct detection_object detection_obj2[DETECTION_NUM_OBJECT]; // TODO aligner
static int32_t detection_num_obj[HOKUYO_MAX];

int detection_module_init()
{
	portBASE_TYPE err = xTaskCreate(detection_task, "detect", DETECTION_STACK_SIZE, NULL, PRIORITY_TASK_DETECTION, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_DETECTION;
	}

	detection_mutex = xSemaphoreCreateMutex();

	if( ! detection_mutex )
	{
		return ERR_INIT_DETECTION;
	}

	detection_queue = xQueueCreate(DETECTION_QUEUE_SIZE, 1);

	if( ! detection_queue )
	{
		return ERR_INIT_DETECTION;
	}

	hokuyo[HOKUYO1].register_callback(detection_hokuyo1_callback);
	hokuyo[HOKUYO2].register_callback(detection_hokuyo2_callback);

	detection_reg_size = 0;

	return 0;
}

module_init(detection_module_init, INIT_DETECTION);

static void detection_task(void* arg)
{
	(void) arg;
	unsigned char event;

	while(1)
	{
		// attente d'un evenement hokuyo ou sick
		if( xQueueReceive(detection_queue, &event, portMAX_DELAY) )
		{
			//xSemaphoreTake(hokuyo_scan_mutex, portMAX_DELAY);
			if( event == DETECTION_EVENT_HOKUYO_1 )
			{
		//		struct systime last_time = systick_get_time();
				detection_compute(HOKUYO1);
		//		struct systime current_time = systick_get_time();
		//		struct systime dt = timediff(current_time, last_time);
		//		log_format(LOG_INFO, "compute_time : %lu us", dt.ms * 1000 + dt.ns/1000);

				//TODO
				//xSemaphoreGive(hokuyo_scan_mutex);

				int16_t detect_size = detection_num_obj[HOKUYO1];
				usb_add(USB_DETECTION_DYNAMIC_OBJECT_SIZE1, &detect_size, sizeof(detect_size));

				//log_format(LOG_INFO, "%d obj", (int)detection_num_obj);
				/*for(i = 0 ; i < detection_num_obj; i++)
				{
					usb_add(USB_DETECTION_DYNAMIC_OBJECT_POLYLINE, detection_object_polyline[i].pt, sizeof(detection_object_polyline[i].pt[0]) * detection_object_polyline[i].size);
				}*/

				usb_add(USB_DETECTION_DYNAMIC_OBJECT1, detection_obj1, DETECTION_NUM_OBJECT_USB * sizeof(detection_obj1[0]));
				/*for(i = 0 ; i < detection_num_obj; i++)
				{
					log_format(LOG_INFO, "obj = %d %d", (int)detection_obj[i].x, (int)detection_obj[i].y);
				}*/
			}
			if( event == DETECTION_EVENT_HOKUYO_2 )
			{
		//		struct systime last_time = systick_get_time();
				detection_compute(HOKUYO2);
		//		struct systime current_time = systick_get_time();
		//		struct systime dt = timediff(current_time, last_time);
		//		log_format(LOG_INFO, "compute_time : %lu us", dt.ms * 1000 + dt.ns/1000);

				//TODO
				//xSemaphoreGive(hokuyo_scan_mutex);

				int16_t detect_size = detection_num_obj[HOKUYO2];
				usb_add(USB_DETECTION_DYNAMIC_OBJECT_SIZE2, &detect_size, sizeof(detect_size));

				//log_format(LOG_INFO, "%d obj", (int)detection_num_obj);
				/*for(i = 0 ; i < detection_num_obj; i++)
				{
					usb_add(USB_DETECTION_DYNAMIC_OBJECT_POLYLINE, detection_object_polyline[i].pt, sizeof(detection_object_polyline[i].pt[0]) * detection_object_polyline[i].size);
				}*/

				usb_add(USB_DETECTION_DYNAMIC_OBJECT2, detection_obj2, DETECTION_NUM_OBJECT_USB * sizeof(detection_obj2[0]));
				/*for(i = 0 ; i < detection_num_obj; i++)
				{
					log_format(LOG_INFO, "obj = %d %d", (int)detection_obj[i].x, (int)detection_obj[i].y);
				}*/
			}

			detection_callback_function();
		}
	}
}

void detection_register_callback(detection_callback callback)
{
	detection_callback_function = callback;
}

static void detection_hokuyo1_callback()
{
	unsigned char event = DETECTION_EVENT_HOKUYO_1;
	xQueueSend(detection_queue, &event, 0);
}

static void detection_hokuyo2_callback()
{
	unsigned char event = DETECTION_EVENT_HOKUYO_2;
	xQueueSend(detection_queue, &event, 0);
}

static void detection_compute(int id)
{
	int i;

	// scan et position des points en x,y privé à la tache hokuyo
	hokuyo_compute_xy(&hokuyo[id].scan, detection_hokuyo_pos);

	// section critique - objets et segments partagés par les méthodes de calcul et la tache de mise à jour
	xSemaphoreTake(detection_mutex, portMAX_DELAY);
	detection_num_obj[id] = hokuyo_find_objects(&hokuyo[id].scan, detection_hokuyo_pos, HOKUYO_NUM_POINTS, detection_object_polyline, DETECTION_NUM_OBJECT);
	detection_reg_size = 0;

	for( i = 0 ; i < detection_num_obj[id] ; i++)
	{
		detection_object_polyline[i].size = regression_poly(detection_object_polyline[i].pt, detection_object_polyline[i].size, detection_reg_ecart, detection_hokuyo_reg + detection_reg_size, HOKUYO_REG_SEG - detection_reg_size);
		detection_object_polyline[i].pt = &detection_hokuyo_reg[detection_reg_size];
		detection_reg_size += detection_object_polyline[i].size;
	}
	detection_remove_static_elements_from_dynamic_list(id);

	for( i = 0; i < detection_num_obj[id] ; i++)
	{
		float xmin = detection_object_polyline[i].pt[0].x;
		float xmax = xmin;
		float ymin = detection_object_polyline[i].pt[0].y;
		float ymax = ymin;
		for(int j = 1; j < detection_object_polyline[i].size; j++ )
		{
			if( detection_object_polyline[i].pt[j].x < xmin )
			{
				xmin = detection_object_polyline[i].pt[j].x;
			}
			else if( detection_object_polyline[i].pt[j].x > xmax )
			{
				xmax = detection_object_polyline[i].pt[j].x;
			}

			if( detection_object_polyline[i].pt[j].y < ymin )
			{
				ymin = detection_object_polyline[i].pt[j].y;
			}
			else if( detection_object_polyline[i].pt[j].y > ymax )
			{
				ymax = detection_object_polyline[i].pt[j].y;
			}
		}

		if(id == HOKUYO1 )
		{
			detection_obj1[i].x = (xmin + xmax) / 2;
			detection_obj1[i].y = (ymin + ymax) / 2;
			detection_obj1[i].size = xmax - xmin;
			if( ymax - ymin > detection_obj1[i].size)
			{
				detection_obj1[i].size = ymax - ymin;
			}
		}
		else
		{
			detection_obj2[i].x = (xmin + xmax) / 2;
			detection_obj2[i].y = (ymin + ymax) / 2;
			detection_obj2[i].size = xmax - xmin;
			if( ymax - ymin > detection_obj2[i].size)
			{
				detection_obj2[i].size = ymax - ymin;
			}
		}
	}

	xSemaphoreGive(detection_mutex);
}

static void detection_remove_static_elements_from_dynamic_list(int id)
{
	int32_t nb_objects_to_test = detection_num_obj[id];
	int32_t i,j,k,l;
	
	//pour chaque objet détecté
	for(i=0; i<nb_objects_to_test; i++)
	{
		struct polyline* current_dyn_object=&detection_object_polyline[i];
		int8_t dynamic_segment_in_object = 0;
		//tester chaque segment (itération sur second point du segment)
		for(j=1; j< current_dyn_object->size; j++)
		{
			int8_t similar_static_segment_found = 0;
			//comparer aux segments statiques
			for(k=0; (k< TABLE_OBJ_SIZE)&&(!similar_static_segment_found); k++)
			{
				for(l=1; (l<table_obj[k].size)&&(!similar_static_segment_found); l++)
				{
					int32_t similarity = detection_get_segment_similarity(
						current_dyn_object->pt +j-1, current_dyn_object->pt +j,
						table_obj[k].pt +l-1, table_obj[k].pt +l);

					if ( similarity < SIMILARITY_ACCEPTANCE)
					{
						similar_static_segment_found = 1;
					}
				}
			}

			if(similar_static_segment_found)
			{
				//le vecteur appartient à un objet statique
				if(!dynamic_segment_in_object)
				{
					//On ampute l'objet du point précédent
					(current_dyn_object->pt)++;
					(current_dyn_object->size)--;
					j--;
				}
				else
				{
					//On réduit l'objet aux segments dynamiques précédents et
					//on reporte les segments non évalués dans un nouvel objet
					detection_object_polyline[detection_num_obj[id]].pt=(current_dyn_object->pt)+j;
					detection_object_polyline[detection_num_obj[id]].size=(current_dyn_object->size)-j;
					current_dyn_object->size=j;
					current_dyn_object=&(detection_object_polyline[detection_num_obj[id]]);
					detection_num_obj[id]++;
					j = 0;
					dynamic_segment_in_object = 0;
				}
			}
			else
			{
				//le vecteur n'appartient pas à un objet statique
				dynamic_segment_in_object = 1;
			}

		}
		if(current_dyn_object->size == 1)
		{
			//si un objet ne contient plus qu'un point, on l'élimine
			current_dyn_object->size=0;
			//si l'objet est en fin de liste, on libère cette position de la liste
			if(current_dyn_object == (detection_object_polyline+detection_num_obj[id]-1))
			{
				detection_num_obj[id]--;
			}
		}
	}
}

//méthode heuristique pour estimer une resemblance entre deux segments
static float detection_get_segment_similarity(const Vect2* a, const Vect2* b, const Vect2* m, const Vect2* n)
{
	float similarity = distance_point_to_segment(*a, *m, *n);
	similarity += distance_point_to_segment(*b, *m, *n);

	return similarity;
}

static float detection_compute_object_on_trajectory(const VectPlan& pos, const struct polyline* polyline, int size, Vect2* a, Vect2* b)
{
	Vect2 a1( 0,  PARAM_LEFT_CORNER_Y );
	Vect2 b1( 1e30,  PARAM_LEFT_CORNER_Y );
	Vect2 a2( 0, PARAM_RIGHT_CORNER_Y );
	Vect2 b2( 1e30, PARAM_RIGHT_CORNER_Y );

	Vect2 c;
	Vect2 d;
	Vect2 h;

	int i;
	int j;
	float x_min = 1e30;
	float y_c = 0;
	float y_d = 0;

	for(i = 0; i < size; i++)
	{
		c = abs_to_loc(pos, polyline[i].pt[0]);
		for(j = 1; j < polyline[i].size; j++)
		{
			d = abs_to_loc(pos, polyline[i].pt[j]);

			// point c devant le robot et dans le tube
			if( c.x > 0 && c.y > a1.y && c.y < a2.y)
			{
				if( c.x < x_min)
				{
					x_min = c.x;
					y_c = c.y;
					y_d = d.y;
				}
			}

			// point d devant le robot et dans le tube
			if( d.x > 0 && d.y > a1.y && d.y < a2.y)
			{
				if( d.x < x_min)
				{
					x_min = d.x;
					y_c = c.y;
					y_d = d.y;
				}
			}

			// gestion du cas c et/ou d ne sont pas dans le tube
			int err1 = segment_intersection(a1, b1, c, d, &h);
			if(err1 == 0)
			{
				if( h.x < x_min)
				{
					x_min = h.x;
					y_c = c.y;
					y_d = d.y;
				}
			}

			int err2 = segment_intersection(a2, b2, c, d, &h);
			if(err2 == 0)
			{
				if( h.x < x_min)
				{
					x_min = h.x;
					y_c = c.y;
					y_d = d.y;
				}
			}

			c = d;
		}
	}

	if(y_c != y_d)
	{
		b1.x = x_min;
		b2.x = x_min;

		if( y_c < y_d)
		{
			b1.y = y_c;
			b2.y = y_d;
		}
		else
		{
			b1.y = y_d;
			b2.y = y_c;
		}
	}

	*a = loc_to_abs(pos, b1);
	*b = loc_to_abs(pos, b2);

	return x_min;
}

float detection_compute_front_object(enum detection_type type, const VectPlan& pos, Vect2* a, Vect2* b)
{
	float x_min = 1e30;
	float x_min_table = 1e30;
	Vect2 c;
	Vect2 d;
#if 0
	if(type == DETECTION_FULL || type == DETECTION_DYNAMIC_OBJ)
	{
		xSemaphoreTake(detection_mutex, portMAX_DELAY);
		//x_min = detection_compute_object_on_trajectory(pos, detection_object_polyline, detection_num_obj[1], a, b); // TODO regrouper obj1 et 2 ?
		xSemaphoreGive(detection_mutex);
	}
	else
	{
		c.x = x_min;
		d.y = PARAM_LEFT_CORNER_Y;

		d.x = x_min;
		d.y = PARAM_RIGHT_CORNER_Y;

		*a = loc_to_abs(pos, c);
		*b = loc_to_abs(pos, d);
	}
#endif
	if(type == DETECTION_FULL || type == DETECTION_STATIC_OBJ)
	{
		x_min_table = detection_compute_object_on_trajectory(pos, table_obj, TABLE_OBJ_SIZE, &c, &d);

		if(x_min_table < x_min)
		{
			x_min = x_min_table;
			*a = c;
			*b = d;
		}
	}

	return x_min;
}
