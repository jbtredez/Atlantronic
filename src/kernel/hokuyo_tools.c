//! @file hokuyo_tools.c
//! @brief Hokuyo tools
//! @author Atlantronic

#include "kernel/hokuyo_tools.h"
#include "kernel/robot_parameters.h"
#include "kernel/systick.h"
#include "kernel/vect_pos.h"
#include "kernel/error_codes.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define GAP 90 //150
#define HOKUYO_START_ANGLE               (-(135 / 180.0f - 44 / 512.0f) * PI)
#define HOKUYO_DTHETA         	         (PI / 512.0f)

void hokuyo_precompute_angle(struct hokuyo_scan* scan, struct vect_pos *pos)
{
	float alpha = scan->sens * HOKUYO_START_ANGLE + scan->pos_hokuyo.alpha;
	int size = HOKUYO_NUM_POINTS;

	for( ; size--; )
	{
		pos->alpha = alpha;
		pos->ca = cosf(alpha);
		pos->sa = sinf(alpha);
		pos++;
		alpha += scan->sens * HOKUYO_DTHETA;
	}
}

void hokuyo_compute_xy(struct hokuyo_scan* scan, struct vect_pos *pos)
{
	int size = HOKUYO_NUM_POINTS;
	uint16_t* distance = scan->distance;
	float dx = scan->pos_hokuyo.x;
	float dy = scan->pos_hokuyo.y;

	for( ; size--; )
	{
		if(*distance > 19 && *distance < 4000)
		{
			pos->x = *distance * pos->ca + dx;
			pos->y = *distance * pos->sa + dy;
		}
		else
		{
			pos->x = 0;
			pos->y = 0;
		}

		distance++;
		pos++;
	}
}

// TODO : à demenager, pas utile cette année (mais pourrait l'être plus tard)
int hokuyo_object_is_pawn(uint16_t* distance, struct hokuyo_object* obj, struct vect_pos *pawn_pos)
{
	int res = 0;

	unsigned int delta = obj->stop - obj->start;
	float tanAlpha = tanf(HOKUYO_DTHETA * delta / 2.0f);

	float r1 = distance[obj->start] * tanAlpha;
	float r2 = distance[obj->stop] * tanAlpha;

	// le rayon vu est forcement plus petit a cause de la resolution du capteur
	if(70.0f < r1 && r1 < 105.0f && 70.0f < r2 && r2 < 105.0f )
	{
		unsigned int med = obj->start + delta/2;
		// distance du point du milieu + rayon du pion
		float dist = distance[med] + 100.0f;
		pawn_pos->alpha = HOKUYO_START_ANGLE + HOKUYO_DTHETA * med;
		pawn_pos->ca = cosf(pawn_pos->alpha);
		pawn_pos->sa = sinf(pawn_pos->alpha);
		pawn_pos->x = dist * pawn_pos->ca;
		pawn_pos->y = - dist * pawn_pos->sa;
		res = 1;
	}

	return res;
}

int hokuyo_find_objects(uint16_t* distance, unsigned int size, struct hokuyo_object* obj, unsigned int obj_size)
{
	int res = 0;
	unsigned int i = 0;
	unsigned int object_start = 0;
	unsigned int object_end = 0;
	int gap = 0;
	int object_start_distance;
	int dist;

	while(i < size)
	{
		// on passe les points erronés
		while( i<size && distance[i] < 20 )
		{ 
			i++;
		}

		if (i >= size)
		{
			goto end;
		}

		// debut de l'objet
		object_start = i;
		object_start_distance = distance[object_start];
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
				gap = dist - object_start_distance;
			}
			i++;
		}
		i--;

		// fin de l'objet
		object_end = i - 1;

		// on filtre les objets avec une vue angulaire faible
		if(object_end - object_start > 5)
		{
			if(obj_size == 0)
			{
				goto end;
			}
			obj_size--;
			obj->start = object_start;
			obj->stop = object_end;
			obj++;
			res++;
		}
	}

end:
	return res;
}
#if 0
/**
 * renvoie 1 si chemin degage, 0 sinon
 * 
 */
uint8_t hoku_check_path(float *distances)
{
	int point1 = 334 ;
	int point2 = 347;
	int point3 = 355;
	int point4 = 326; 
	//carré de COTE sur COTE 
	uint8_t res = 0;
	//on commence en haut a gauche puis sens horaire
	//
	//	1 	2
	// 	_________
	//	|	|
	//	|	|
	//	|	|
	//	_________	
	//	4	3

	int limite1 = 265;
	int limite2 = 577;
  /*
  float result1, result2;
  float limite1, limite2;
  float oppose = COTE / 2;
  float adjacent1 = 200; //70; //pt3 et pt4
  float adjacent2 = adjacent1 + COTE; //pt1 et pt2
  float angle_rad;
  //TODO atan2f
  result1 = atanf (oppose/adjacent1);  //pt3 et pt4
  result2 = atanf (oppose/adjacent2); //pt1 et pt2
  
  point1 = (NB_POINT/2) - (result2 * HOKUYO_DTHETA);
  point2 = (NB_POINT/2) + (result2 * HOKUYO_DTHETA);
  point3 = (NB_POINT/2) + (result1 * HOKUYO_DTHETA);
  point4 = (NB_POINT/2) - (result1 * HOKUYO_DTHETA);

  //pt1 et pt2
  angle_rad = PI * ( result2 / 180); 
  limite2 = oppose / (float)sinf(angle_rad);

  //pt3 et pt4
  angle_rad = PI * ( result1 / 180); 
  limite1 = oppose / (float)sinf(angle_rad);*/

	if( ( (distances[point1] > limite2) || (distances[point1] == 0) ) && 
	  ( (distances[point2] > limite2) || (distances[point2] == 0) ) &&
	  ( (distances[point3] > limite1) || (distances[point3] == 0) ) && 
	  ( (distances[point4] > limite1) || (distances[point4] == 0) ) )
	{
		res = 1;
		// chemin degage!!!!!
	}	
	return res;
}
#endif
