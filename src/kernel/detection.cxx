#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/hokuyo_tools.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/io.h"
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
#define AVOIDANCE_MARGIN             1.1

enum
{
	DETECTION_EVENT_HOKUYO_1,
//	DETECTION_EVENT_HOKUYO_2,
};

static void detection_task(void* arg);
static int detection_module_init();
static void detection_compute();
static void detection_remove_static_elements_from_dynamic_list();
static float detection_get_segment_similarity( Vect2* a,  Vect2* b,  Vect2* m,  Vect2* n);
static void detection_hokuyo1_callback();
//static void detection_hokuyo2_callback();
static xQueueHandle detection_queue;
static detection_callback detection_callback_function = (detection_callback)nop_function;

// données privées à la tache detection
static Vect2 detection_hokuyo_pos[HOKUYO_NUM_POINTS];
static int detection_reg_ecart = 25;

// données partagées par la tache et des méthodes d'accés
static xSemaphoreHandle detection_mutex;
static Vect2 detection_hokuyo_reg[HOKUYO_REG_SEG];
static Vect2 detection_omron_rectangle[5];
static int detection_reg_size;
static struct polyline detection_object_polyline[DETECTION_NUM_OBJECT];
static struct detection_object detection_obj[DETECTION_NUM_OBJECT];
//static struct detection_object detection_obj2[DETECTION_NUM_OBJECT];
static int32_t detection_num_obj;

static Vect2 detectionOpponentRobotPt[5];

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
//	hokuyo[HOKUYO2].register_callback(detection_hokuyo2_callback);

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
		// attente d'un evenement hokuyo ou omron
		if( xQueueReceive(detection_queue, &event, portMAX_DELAY) )
		{
			if( event == DETECTION_EVENT_HOKUYO_1 )
			{
				xSemaphoreTake(hokuyo[HOKUYO1].scan_mutex, portMAX_DELAY);
		//		struct systime last_time = systick_get_time();
				detection_compute();
		//		struct systime current_time = systick_get_time();
		//		struct systime dt = timediff(current_time, last_time);
		//		log_format(LOG_INFO, "compute_time : %lu us", dt.ms * 1000 + dt.ns/1000);

				xSemaphoreGive(hokuyo[HOKUYO1].scan_mutex);
			}
			detection_callback_function();
		}

		int16_t detect_size = detection_num_obj;
		usb_add(USB_DETECTION_DYNAMIC_OBJECT_SIZE, &detect_size, sizeof(detect_size));

		//log_format(LOG_INFO, "%d obj", (int)detection_num_obj);
		for(int i = 0 ; i < detect_size; i++)
		{
			usb_add(USB_DETECTION_DYNAMIC_OBJECT_POLYLINE, detection_object_polyline[i].pt, sizeof(detection_object_polyline[i].pt[0]) * detection_object_polyline[i].size);
		}

		usb_add(USB_DETECTION_DYNAMIC_OBJECT, detection_obj, DETECTION_NUM_OBJECT_USB * sizeof(detection_obj[0]));
		/*for(int i = 0 ; i < detect_size; i++)
		{
			log_format(LOG_INFO, "obj = %3d %3d size %3d", (int)detection_obj1[i].x, (int)detection_obj1[i].y, detection_object_polyline[i].size);
		}*/
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

/*static void detection_hokuyo2_callback()
{
	unsigned char event = DETECTION_EVENT_HOKUYO_2;
	xQueueSend(detection_queue, &event, 0);
}*/

static void detection_compute()
{
	int i;

	// scan et position des points en x,y privé à la tache hokuyo
	hokuyo_compute_xy(&hokuyo[HOKUYO1].scan, detection_hokuyo_pos);

	// section critique - objets et segments partagés par les méthodes de calcul et la tache de mise à jour
	xSemaphoreTake(detection_mutex, portMAX_DELAY);
	detection_num_obj = hokuyo_find_objects(&hokuyo[HOKUYO1].scan, detection_hokuyo_pos, HOKUYO_NUM_POINTS, detection_object_polyline, DETECTION_NUM_OBJECT);
	detection_reg_size = 0;

	for( i = 0 ; i < detection_num_obj ; i++)
	{
		detection_object_polyline[i].size = regression_poly(detection_object_polyline[i].pt, detection_object_polyline[i].size, detection_reg_ecart, detection_hokuyo_reg + detection_reg_size, HOKUYO_REG_SEG - detection_reg_size);
		detection_object_polyline[i].pt = &detection_hokuyo_reg[detection_reg_size];
		detection_reg_size += detection_object_polyline[i].size;
	}

	detection_remove_static_elements_from_dynamic_list();

	// TODO revoir comment c est fait pour separer objets calcules par hokuyo et ceux par sick
	// TODO pour le moment, on utilise un seul tableau et on met a jour les omron ici
	VectPlan pos = location_get_position();
	bool opponentBehind = ! gpio_get(IO_OMRON1) || ! gpio_get(IO_OMRON2) || ! gpio_get(IO_OMRON3);
	if( opponentBehind )
	{
		// TODO ameliorer avec le lidarlite
		// objet derriere
		detection_omron_rectangle[0] = loc_to_abs(pos, Vect2(PARAM_NP_X, PARAM_LEFT_CORNER_Y));
		detection_omron_rectangle[1] = loc_to_abs(pos, Vect2(PARAM_NP_X, PARAM_RIGHT_CORNER_Y));
		detection_omron_rectangle[2] = loc_to_abs(pos, Vect2(PARAM_NP_X - REAR_OMRON_RANGE, PARAM_RIGHT_CORNER_Y));
		detection_omron_rectangle[3] = loc_to_abs(pos, Vect2(PARAM_NP_X - REAR_OMRON_RANGE, PARAM_LEFT_CORNER_Y));
		detection_omron_rectangle[4] = detection_omron_rectangle[0];

		// on regarde si ce n'est pas un point en dehors de la table
		bool allInsideTable = true;
		for(int i = 2; i < 4; i++)
		{
			if( fabsf(detection_omron_rectangle[i].x) > 1400 || fabsf(detection_omron_rectangle[i].y) > 900 )
			{
				allInsideTable = false;
			}
		}

		// on ne sait pas ou il est exactement. Dans le pire des cas, le robot est colle. On ajoute un rectangle d'un robot virtuel colle au notre
		detection_omron_rectangle[2] = loc_to_abs(pos, Vect2(PARAM_NP_X - 2*DETECTION_OPPONENT_ROBOT_RADIUS, PARAM_RIGHT_CORNER_Y));
		detection_omron_rectangle[3] = loc_to_abs(pos, Vect2(PARAM_NP_X - 2*DETECTION_OPPONENT_ROBOT_RADIUS, PARAM_LEFT_CORNER_Y));

		if( allInsideTable )
		{
			int num = detection_num_obj;
			Vect2 u(cosf(pos.theta), sinf(pos.theta));
			Vect2 v(u.y, -u.x);
			Vect2 p(pos.x, pos.y);

			detection_object_polyline[num].pt = detection_omron_rectangle;
			detection_object_polyline[num].size = 5;
			detection_num_obj++;
		}
	}

	for( i = 0; i < detection_num_obj ; i++)
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

		detection_obj[i].x = (xmin + xmax) / 2;
		detection_obj[i].y = (ymin + ymax) / 2;
		detection_obj[i].size = xmax - xmin;
		if( ymax - ymin > detection_obj[i].size)
		{
			detection_obj[i].size = ymax - ymin;
		}
	}

	xSemaphoreGive(detection_mutex);
}

static void detection_remove_static_elements_from_dynamic_list()
{
	int32_t nb_objects_to_test = detection_num_obj;
	int32_t i,j,k,l;
	int new_num_obj = 0;
	
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
					detection_object_polyline[detection_num_obj].pt=(current_dyn_object->pt)+j;
					detection_object_polyline[detection_num_obj].size=(current_dyn_object->size)-j;
					current_dyn_object->size=j;
					current_dyn_object=&(detection_object_polyline[detection_num_obj]);
					detection_num_obj++;
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
		if(current_dyn_object->size > 1)
		{
			//si un objet contient au moins un segment (2 points), on le garde
			detection_object_polyline[new_num_obj] = detection_object_polyline[i];
			new_num_obj++;
		}
	}

	detection_num_obj = new_num_obj;
}

//méthode heuristique pour estimer une resemblance entre deux segments
static float detection_get_segment_similarity( Vect2* a,  Vect2* b,  Vect2* m,  Vect2* n)
{
	float similarity = distance_point_to_segment(*a, *m, *n);
	similarity += distance_point_to_segment(*b, *m, *n);

	return similarity;
}

static float detection_compute_object_on_trajectory(const VectPlan& pos, const struct polyline* polyline, int size, Vect2* a, Vect2* b)
{
	Vect2 a1( 0,    PARAM_LEFT_CORNER_Y  * AVOIDANCE_MARGIN);
	Vect2 b1( 1e30, PARAM_LEFT_CORNER_Y  * AVOIDANCE_MARGIN);
	Vect2 a2( 0,    PARAM_RIGHT_CORNER_Y * AVOIDANCE_MARGIN);
	Vect2 b2( 1e30, PARAM_RIGHT_CORNER_Y * AVOIDANCE_MARGIN);

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

	if(type == DETECTION_FULL || type == DETECTION_DYNAMIC_OBJ)
	{
		xSemaphoreTake(detection_mutex, portMAX_DELAY);

		int16_t detect_size = detection_num_obj;
		for(int i = 0 ; i < detect_size; i++)
		{
			struct polyline detectionOpponentRobot = {detectionOpponentRobotPt, sizeof(detectionOpponentRobotPt)/sizeof(detectionOpponentRobotPt[0])};
			Vect2 opponentRobotPos(detection_obj[i].x, detection_obj[i].y);
			detectionOpponentRobotPt[0] = opponentRobotPos + Vect2(DETECTION_OPPONENT_ROBOT_RADIUS/2, DETECTION_OPPONENT_ROBOT_RADIUS/2);
			detectionOpponentRobotPt[1] = opponentRobotPos + Vect2(DETECTION_OPPONENT_ROBOT_RADIUS/2, -DETECTION_OPPONENT_ROBOT_RADIUS/2);
			detectionOpponentRobotPt[2] = opponentRobotPos + Vect2(-DETECTION_OPPONENT_ROBOT_RADIUS/2, -DETECTION_OPPONENT_ROBOT_RADIUS/2);
			detectionOpponentRobotPt[3] = opponentRobotPos + Vect2(-DETECTION_OPPONENT_ROBOT_RADIUS/2, DETECTION_OPPONENT_ROBOT_RADIUS/2);
			detectionOpponentRobotPt[4] = opponentRobotPos + Vect2(DETECTION_OPPONENT_ROBOT_RADIUS/2, DETECTION_OPPONENT_ROBOT_RADIUS/2);

			float x_min_tmp = detection_compute_object_on_trajectory(pos, &detectionOpponentRobot, 1, a, b);
			if( x_min_tmp < x_min )
			{
				x_min = x_min_tmp;
			}
		}

		xSemaphoreGive(detection_mutex);
	}
	else
	{
		c.x = x_min;
		d.y = PARAM_LEFT_CORNER_Y;

		d.x = x_min;
		d.y = PARAM_RIGHT_CORNER_Y;

		if( a )
		{
			*a = loc_to_abs(pos, c);
		}

		if( b )
		{
			*b = loc_to_abs(pos, d);
		}
	}

	if(type == DETECTION_FULL || type == DETECTION_STATIC_OBJ)
	{
		xSemaphoreTake(detection_mutex, portMAX_DELAY);
		x_min_table = detection_compute_object_on_trajectory(pos, table_obj, TABLE_OBJ_SIZE, &c, &d);
		xSemaphoreGive(detection_mutex);

		if(x_min_table < x_min)
		{
			x_min = x_min_table;
			if( a)
			{
				*a = c;
			}
			if( b )
			{
				*b = d;
			}
		}
	}

	return x_min;
}

//! calcul de la distance avec le robot adverse le plus proche et situe sur le trajet (a +- 90 degres du vecteur directeur)
float detection_compute_opponent_in_range_distance(Vect2 a, Vect2 u)
{
	float minDistance = 1e30;

	xSemaphoreTake(detection_mutex, portMAX_DELAY);
	int16_t detect_size = detection_num_obj;
	for(int i = 0 ; i < detect_size; i++)
	{
		Vect2 b(detection_obj[i].x, detection_obj[i].y);
		Vect2 ab = b - a;
		float ps = u.scalarProd(ab);
		if( ps > 0 )
		{
			// le robot adverse est devant
			float d = ab.norm();
			if( d < minDistance )
			{
				minDistance = d;
			}
		}
	}
	xSemaphoreGive(detection_mutex);

	minDistance -= DETECTION_OPPONENT_ROBOT_RADIUS;

	if( minDistance < 0)
	{
		minDistance = 0;
	}

	return minDistance;
}

//! calcul de la distance avec le robot adverse le plus proche
float detection_compute_opponent_distance(Vect2 a)
{
	float minDistance = 1e30;

	xSemaphoreTake(detection_mutex, portMAX_DELAY);
	int16_t detect_size = detection_num_obj;
	for(int i = 0 ; i < detect_size; i++)
	{
		Vect2 b(detection_obj[i].x, detection_obj[i].y);
		Vect2 ab = b - a;
		float d = ab.norm();
		if( d < minDistance )
		{
			minDistance = d;
		}
	}
	xSemaphoreGive(detection_mutex);

	minDistance -= DETECTION_OPPONENT_ROBOT_RADIUS;

	if( minDistance < 0)
	{
		minDistance = 0;
	}

	return minDistance;
}
