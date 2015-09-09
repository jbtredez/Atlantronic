//! @file hokuyo_tools.c
//! @brief Hokuyo tools
//! @author Atlantronic

#include "kernel/hokuyo_tools.h"
#include "kernel/robot_parameters.h"
#include "kernel/systick.h"
#include "kernel/math/vect2.h"
#include "kernel/error_codes.h"

#include <stdio.h>
#include <stdlib.h>

#define GAP 90 //150
#define HOKUYO_TABLE_DELTA       200

void hokuyo_compute_xy(struct hokuyo_scan* scan, Vect2 *pos)
{
	int size = HOKUYO_NUM_POINTS;
	uint16_t* distance = scan->distance;
	VectPlan hokuyo_pos_table = loc_to_abs(scan->pos_robot, scan->pos_hokuyo);
	float theta = scan->sens * HOKUYO_START_ANGLE + hokuyo_pos_table.theta;
	float hokuyoTheta = HOKUYO_START_ANGLE;

	for( ; size--; )
	{
		if(*distance > scan->min_distance && *distance < HOKUYO_MAX_RANGE && hokuyoTheta > scan->theta_min && hokuyoTheta < scan->theta_max)
		{
			pos->x = *distance * cosf(theta) + hokuyo_pos_table.x;
			pos->y = *distance * sinf(theta) + hokuyo_pos_table.y;
		}
		else
		{
			pos->x = 1000000;
			pos->y = 1000000;
			*distance = 0;
		}

		distance++;
		pos++;
		theta += scan->sens * HOKUYO_DTHETA;
		hokuyoTheta += HOKUYO_DTHETA;
	}
}

int hokuyo_find_objects(struct hokuyo_scan* scan, Vect2* hokuyo_pos, unsigned int size, struct polyline* obj, unsigned int obj_size)
{
	int res = 0;
	unsigned int i = 0;
	unsigned int object_start = 0;
	int object_size = 0;
	int gap = 0;
	int last_dist;
	int dist;

	while(i < size)
	{
		// on passe les points erronés ou en dehors de la table
		while( ( i < size && scan->distance[i] < scan->min_distance) || fabsf(hokuyo_pos[i].x) > 1500 - HOKUYO_TABLE_DELTA || fabsf(hokuyo_pos[i].y) > 1000 - HOKUYO_TABLE_DELTA)
		{
			i++;
		}

		if (i >= size - 1)
		{
			goto end;
		}

		// debut de l'objet
		object_start = i;
		last_dist = scan->distance[i];
		gap = 0;
		i++;
		while(i < size && abs(gap) < GAP)
		{
			dist = scan->distance[i];
			if( dist < scan->min_distance || fabsf(hokuyo_pos[i].x) > 1500 - HOKUYO_TABLE_DELTA || fabsf(hokuyo_pos[i].y) > 1000 - HOKUYO_TABLE_DELTA )
			{
				gap = GAP;
			}
			else
			{
				gap = dist - last_dist;
				last_dist = dist;
			}
			i++;
		}
		i--;

		// taille de l'objet
		object_size = i - (int) object_start;

		// on filtre les objets avec une vue angulaire faible : scan->min_object_size points mini
		if(object_size >= scan->min_object_size)
		{
			if(obj_size == 0)
			{
				goto end;
			}
			obj_size--;
			obj->pt = hokuyo_pos + object_start;
			obj->size = object_size;
			obj++;
			res++;
		}
	}

end:
	return res;
}
