//! @file hokuyo_tools.c
//! @brief Hokuyo tools
//! @author Atlantronic

#include "kernel/hokuyo_tools.h"
#include "kernel/robot_parameters.h"
#include "kernel/systick.h"
#include "kernel/vect_pos.h"
#include "kernel/error_codes.h"
#include "kernel/math/trigo.h"

#include <stdio.h>
#include <stdlib.h>

#define GAP 90 //150

#define HOKUYO_DTHETA         	              65536
#define HOKUYO_START_ANGLE               (- 22282240 )      //!< 135 degrés + 44 HOKUYO_DTHETA


void hokuyo_compute_xy(struct hokuyo_scan* scan, struct fx_vect2 *pos)
{
	int size = HOKUYO_NUM_POINTS;
	uint16_t* distance = scan->distance;
	struct fx_vect_pos hokuyo_pos_table;
	pos_robot_to_table(&scan->pos_robot, &scan->pos_hokuyo, &hokuyo_pos_table);
	int32_t alpha = scan->sens * HOKUYO_START_ANGLE + hokuyo_pos_table.alpha;

	for( ; size--; )
	{
		if(*distance > 19 && *distance < 4000)
		{
			// distance en mm, ca en virgule fixe
			pos->x = ((*distance * (int64_t)fx_cos(alpha)) >> 14) + hokuyo_pos_table.x;
			pos->y = ((*distance * (int64_t)fx_sin(alpha)) >> 14) + hokuyo_pos_table.y;
		}
		else
		{
			pos->x = 0;
			pos->y = 0;
		}

		distance++;
		pos++;
		alpha += scan->sens * HOKUYO_DTHETA;
	}
}

#if 0
// TODO : à demenager, pas utile cette année (mais pourrait l'être plus tard)
int hokuyo_object_is_pawn(uint16_t* distance, struct hokuyo_object* obj, struct fx_vect_pos *pawn_pos)
{
	int res = 0;

	unsigned int delta = obj->stop - obj->start;
	float tanAlpha = fx_tan(HOKUYO_DTHETA * delta / 2.0f);

	float r1 = distance[obj->start] * tanAlpha;
	float r2 = distance[obj->stop] * tanAlpha;

	// le rayon vu est forcement plus petit a cause de la resolution du capteur
	if(70.0f < r1 && r1 < 105.0f && 70.0f < r2 && r2 < 105.0f )
	{
		unsigned int med = obj->start + delta/2;
		// distance du point du milieu + rayon du pion
		float dist = distance[med] + 100.0f;
		pawn_pos->alpha = HOKUYO_START_ANGLE + HOKUYO_DTHETA * med;
		pawn_pos->ca = fx_cos(pawn_pos->alpha);
		pawn_pos->sa = fx_sin(pawn_pos->alpha);
		pawn_pos->x = dist * pawn_pos->ca;
		pawn_pos->y = - dist * pawn_pos->sa;
		res = 1;
	}

	return res;
}
#endif

int hokuyo_find_objects(uint16_t* distance, unsigned int size, struct hokuyo_object* obj, unsigned int obj_size)
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
		// on passe les points erronés
		while( i<size && distance[i] < 20 )
		{ 
			i++;
		}

		if (i >= size - 1)
		{
			goto end;
		}

		// debut de l'objet
		object_start = i;
		last_dist = distance[i];
		gap = 0;
		i++;
		while(i < size && abs(gap) < GAP)
		{
			dist = distance[i];
			if( dist < 20 )
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

		// on filtre les objets avec une vue angulaire faible : 3 points mini (1 degré)
		if(object_size > 2)
		{
			if(obj_size == 0)
			{
				goto end;
			}
			obj_size--;
			obj->start = object_start;
			obj->size = object_size;
			obj++;
			res++;
		}
	}

end:
	return res;
}
