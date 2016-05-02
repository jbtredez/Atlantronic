#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "middleware/hokuyo_tools.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/io.h"
#include "disco/bot.h"
#include "kernel/math/regression.h"
#include "kernel/math/segment_intersection.h"
#include "kernel/location/location.h"
#include "disco/table.h"
#include "detection.h"

//! @todo réglage au pif
#define DETECTION_STACK_SIZE         600
#define DETECTION_QUEUE_SIZE          20
#define SIMILARITY_ACCEPTANCE        200
#define AVOIDANCE_MARGIN             1.1

enum
{
	DETECTION_EVENT_HOKUYO_1,
	DETECTION_EVENT_HOKUYO_2,
};

int Detection::init(Hokuyo* hokuyo1, Hokuyo* hokuyo2, Location* location)
{
	return Detection::init(hokuyo1, hokuyo2, NULL, location);
}

int Detection::init(Rplidar* rplidar, Location* location)
{
	return this->init(NULL, NULL, rplidar, location);
}

int Detection::init(Hokuyo* hokuyo1, Hokuyo* hokuyo2, Rplidar* rplidar, Location* location)
{
	m_regEcart = 25;
	m_callbackFunction = (DetectionCallback)nop_function;
	m_hokuyo1 = hokuyo1;
	m_hokuyo2 = hokuyo2;
	m_rpLidar = rplidar;
	m_location = location;

	portBASE_TYPE err = xTaskCreate(Detection::taskWrapper, "detect", DETECTION_STACK_SIZE, this, PRIORITY_TASK_DETECTION, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_DETECTION;
	}

	m_mutex = xSemaphoreCreateMutex();

	if( ! m_mutex )
	{
		return ERR_INIT_DETECTION;
	}

	m_queue = xQueueCreate(DETECTION_QUEUE_SIZE, 1);

	if( ! m_queue )
	{
		return ERR_INIT_DETECTION;
	}

	if( m_hokuyo1 )
	{
		m_hokuyo1->registerCallback(hokuyo1Callback, this);
	}

	if( m_hokuyo2 )
	{
		m_hokuyo2->registerCallback(hokuyo2Callback, this);
	}

	if( m_rpLidar )
	{
		//m_rpLidar->registerCallback()
	}

	m__regSize = 0;

	return 0;
}

void Detection::taskWrapper(void* arg)
{
	Detection* detect = (Detection*) arg;
	detect->task();
}

void Detection::task()
{
	unsigned char event;

	while(1)
	{
		// attente d'un evenement hokuyo ou omron
		if( xQueueReceive(m_queue, &event, portMAX_DELAY) )
		{
			if( event == DETECTION_EVENT_HOKUYO_1 )
			{
				xSemaphoreTake(m_hokuyo1->scan_mutex, portMAX_DELAY);
		//		struct systime last_time = systick_get_time();
				compute();
		//		struct systime current_time = systick_get_time();
		//		struct systime dt = timediff(current_time, last_time);
		//		log_format(LOG_INFO, "compute_time : %lu us", dt.ms * 1000 + dt.ns/1000);

				xSemaphoreGive(m_hokuyo1->scan_mutex);
			}
			m_callbackFunction(m_callbackArg);
		}

		int16_t detect_size = m_numObj;
		usb_add(USB_DETECTION_DYNAMIC_OBJECT_SIZE, &detect_size, sizeof(detect_size));

		//log_format(LOG_INFO, "%d obj", (int)detection_num_obj);
		for(int i = 0 ; i < detect_size; i++)
		{
			usb_add(USB_DETECTION_DYNAMIC_OBJECT_POLYLINE, m__objectPolyline[i].pt, sizeof(m__objectPolyline[i].pt[0]) * m__objectPolyline[i].size);
		}

		usb_add(USB_DETECTION_DYNAMIC_OBJECT, m__obj, DETECTION_NUM_OBJECT_USB * sizeof(m__obj[0]));
		/*for(int i = 0 ; i < detect_size; i++)
		{
			log_format(LOG_INFO, "obj = %3d %3d size %3d", (int)detection_obj1[i].x, (int)detection_obj1[i].y, detection_object_polyline[i].size);
		}*/
	}
}

void Detection::registerCallback(DetectionCallback callback, void* arg)
{
	m_callbackFunction = callback;
	m_callbackArg = arg;
}

void Detection::hokuyo1Callback(void* arg)
{
	Detection* detect = (Detection*) arg;
	unsigned char event = DETECTION_EVENT_HOKUYO_1;
	xQueueSend(detect->m_queue, &event, 0);
}

void Detection::hokuyo2Callback(void* arg)
{
	Detection* detect = (Detection*) arg;
	unsigned char event = DETECTION_EVENT_HOKUYO_2;
	xQueueSend(detect->m_queue, &event, 0);
}

void Detection::compute()
{
	int i;

	// scan et position des points en x,y privé à la tache hokuyo
	hokuyo_compute_xy(&m_hokuyo1->scan, m_hokuyoPos);

	// section critique - objets et segments partagés par les méthodes de calcul et la tache de mise à jour
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_numObj = hokuyo_find_objects(&m_hokuyo1->scan, m_hokuyoPos, HOKUYO_NUM_POINTS, m__objectPolyline, DETECTION_NUM_OBJECT);
	m__regSize = 0;

	for( i = 0 ; i < m_numObj ; i++)
	{
		m__objectPolyline[i].size = regression_poly(m__objectPolyline[i].pt, m__objectPolyline[i].size, m_regEcart, m_hokuyoReg + m__regSize, HOKUYO_REG_SEG - m__regSize);
		m__objectPolyline[i].pt = &m_hokuyoReg[m__regSize];
		m__regSize += m__objectPolyline[i].size;
	}

	removeStaticElementsFromDynamicList();

	// TODO revoir comment c est fait pour separer objets calcules par hokuyo et ceux par sick
	// TODO pour le moment, on utilise un seul tableau et on met a jour les omron ici
	VectPlan pos = m_location->getPosition();
	bool opponentBehind = ! gpio_get(IO_OMRON1) || ! gpio_get(IO_OMRON2) || ! gpio_get(IO_OMRON3);
	if( opponentBehind )
	{
		// TODO ameliorer avec le lidarlite
		// objet derriere
		m_omronRectangle[0] = loc_to_abs(pos, Vect2(-Bot::halfLength, Bot::halfWidth));
		m_omronRectangle[1] = loc_to_abs(pos, Vect2(-Bot::halfLength, -Bot::halfWidth));
		m_omronRectangle[2] = loc_to_abs(pos, Vect2(-Bot::halfLength - Bot::rearOmronRange, - Bot::halfWidth));
		m_omronRectangle[3] = loc_to_abs(pos, Vect2(-Bot::halfLength - Bot::rearOmronRange, Bot::halfWidth));
		m_omronRectangle[4] = m_omronRectangle[0];

		// on regarde si ce n'est pas un point en dehors de la table
		bool allInsideTable = true;
		for(int i = 2; i < 4; i++)
		{
			if( fabsf(m_omronRectangle[i].x) > 1400 || fabsf(m_omronRectangle[i].y) > 900 )
			{
				allInsideTable = false;
			}
		}

		// on ne sait pas ou il est exactement. Dans le pire des cas, le robot est colle. On ajoute un rectangle d'un robot virtuel colle au notre
		m_omronRectangle[2] = loc_to_abs(pos, Vect2(-Bot::halfLength - 2*DETECTION_OPPONENT_ROBOT_RADIUS, -Bot::halfWidth));
		m_omronRectangle[3] = loc_to_abs(pos, Vect2(-Bot::halfLength - 2*DETECTION_OPPONENT_ROBOT_RADIUS, Bot::halfWidth));

		if( allInsideTable )
		{
			int num = m_numObj;
			Vect2 u(cosf(pos.theta), sinf(pos.theta));
			Vect2 v(u.y, -u.x);
			Vect2 p(pos.x, pos.y);

			m__objectPolyline[num].pt = m_omronRectangle;
			m__objectPolyline[num].size = 5;
			m_numObj++;
		}
	}

	for( i = 0; i < m_numObj ; i++)
	{
		float xmin = m__objectPolyline[i].pt[0].x;
		float xmax = xmin;
		float ymin = m__objectPolyline[i].pt[0].y;
		float ymax = ymin;
		for(int j = 1; j < m__objectPolyline[i].size; j++ )
		{
			if( m__objectPolyline[i].pt[j].x < xmin )
			{
				xmin = m__objectPolyline[i].pt[j].x;
			}
			else if( m__objectPolyline[i].pt[j].x > xmax )
			{
				xmax = m__objectPolyline[i].pt[j].x;
			}

			if( m__objectPolyline[i].pt[j].y < ymin )
			{
				ymin = m__objectPolyline[i].pt[j].y;
			}
			else if( m__objectPolyline[i].pt[j].y > ymax )
			{
				ymax = m__objectPolyline[i].pt[j].y;
			}
		}

		m__obj[i].x = (xmin + xmax) / 2;
		m__obj[i].y = (ymin + ymax) / 2;
		m__obj[i].size = xmax - xmin;
		if( ymax - ymin > m__obj[i].size)
		{
			m__obj[i].size = ymax - ymin;
		}
	}

	xSemaphoreGive(m_mutex);
}

void Detection::removeStaticElementsFromDynamicList()
{
	int32_t nb_objects_to_test = m_numObj;
	int32_t i,j,k,l;
	int new_num_obj = 0;
	
	//pour chaque objet détecté
	for(i=0; i<nb_objects_to_test; i++)
	{
		struct polyline* current_dyn_object=&m__objectPolyline[i];
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
					int32_t similarity = getSegmentSimilarity(
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
					m__objectPolyline[m_numObj].pt=(current_dyn_object->pt)+j;
					m__objectPolyline[m_numObj].size=(current_dyn_object->size)-j;
					current_dyn_object->size=j;
					current_dyn_object=&(m__objectPolyline[m_numObj]);
					m_numObj++;
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
			m__objectPolyline[new_num_obj] = m__objectPolyline[i];
			new_num_obj++;
		}
	}

	m_numObj = new_num_obj;
}

//méthode heuristique pour estimer une resemblance entre deux segments
float Detection::getSegmentSimilarity( Vect2* a,  Vect2* b,  Vect2* m,  Vect2* n)
{
	float similarity = distance_point_to_segment(*a, *m, *n);
	similarity += distance_point_to_segment(*b, *m, *n);

	return similarity;
}

float Detection::computeObjectOnTrajectory(const VectPlan& pos, const struct polyline* polyline, int size, Vect2* a, Vect2* b)
{
	Vect2 a1( 0,    Bot::halfWidth  * AVOIDANCE_MARGIN);
	Vect2 b1( 1e30, Bot::halfWidth  * AVOIDANCE_MARGIN);
	Vect2 a2( 0,    -Bot::halfWidth * AVOIDANCE_MARGIN);
	Vect2 b2( 1e30, -Bot::halfWidth * AVOIDANCE_MARGIN);

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

float Detection::computeFrontObject(enum detection_type type, const VectPlan& pos, Vect2* a, Vect2* b)
{
	float x_min = 1e30;
	float x_min_table = 1e30;
	Vect2 c;
	Vect2 d;

	if(type == DETECTION_FULL || type == DETECTION_DYNAMIC_OBJ)
	{
		xSemaphoreTake(m_mutex, portMAX_DELAY);

		int16_t detect_size = m_numObj;
		for(int i = 0 ; i < detect_size; i++)
		{
			struct polyline detectionOpponentRobot = {m_detectionOpponentRobotPt, sizeof(m_detectionOpponentRobotPt)/sizeof(m_detectionOpponentRobotPt[0])};
			Vect2 opponentRobotPos(m__obj[i].x, m__obj[i].y);
			m_detectionOpponentRobotPt[0] = opponentRobotPos + Vect2(DETECTION_OPPONENT_ROBOT_RADIUS/2, DETECTION_OPPONENT_ROBOT_RADIUS/2);
			m_detectionOpponentRobotPt[1] = opponentRobotPos + Vect2(DETECTION_OPPONENT_ROBOT_RADIUS/2, -DETECTION_OPPONENT_ROBOT_RADIUS/2);
			m_detectionOpponentRobotPt[2] = opponentRobotPos + Vect2(-DETECTION_OPPONENT_ROBOT_RADIUS/2, -DETECTION_OPPONENT_ROBOT_RADIUS/2);
			m_detectionOpponentRobotPt[3] = opponentRobotPos + Vect2(-DETECTION_OPPONENT_ROBOT_RADIUS/2, DETECTION_OPPONENT_ROBOT_RADIUS/2);
			m_detectionOpponentRobotPt[4] = opponentRobotPos + Vect2(DETECTION_OPPONENT_ROBOT_RADIUS/2, DETECTION_OPPONENT_ROBOT_RADIUS/2);

			float x_min_tmp = computeObjectOnTrajectory(pos, &detectionOpponentRobot, 1, a, b);
			if( x_min_tmp < x_min )
			{
				x_min = x_min_tmp;
			}
		}

		xSemaphoreGive(m_mutex);
	}
	else
	{
		c.x = x_min;
		d.y = Bot::halfWidth;

		d.x = x_min;
		d.y = - Bot::halfWidth;

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
		xSemaphoreTake(m_mutex, portMAX_DELAY);
		x_min_table = computeObjectOnTrajectory(pos, table_obj, TABLE_OBJ_SIZE, &c, &d);
		xSemaphoreGive(m_mutex);

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
float Detection::computeOpponentInRangeDistance(Vect2 a, Vect2 u)
{
	float minDistance = 1e30;

	xSemaphoreTake(m_mutex, portMAX_DELAY);
	int16_t detect_size = m_numObj;
	for(int i = 0 ; i < detect_size; i++)
	{
		Vect2 b(m__obj[i].x, m__obj[i].y);
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
	xSemaphoreGive(m_mutex);

	minDistance -= DETECTION_OPPONENT_ROBOT_RADIUS;

	if( minDistance < 0)
	{
		minDistance = 0;
	}

	return minDistance;
}

//! calcul de la distance avec le robot adverse le plus proche
float Detection::computeOpponentDistance(Vect2 a)
{
	float minDistance = 1e30;

	xSemaphoreTake(m_mutex, portMAX_DELAY);
	int16_t detect_size = m_numObj;
	for(int i = 0 ; i < detect_size; i++)
	{
		Vect2 b(m__obj[i].x, m__obj[i].y);
		Vect2 ab = b - a;
		float d = ab.norm();
		if( d < minDistance )
		{
			minDistance = d;
		}
	}
	xSemaphoreGive(m_mutex);

	minDistance -= DETECTION_OPPONENT_ROBOT_RADIUS;

	if( minDistance < 0)
	{
		minDistance = 0;
	}

	return minDistance;
}
