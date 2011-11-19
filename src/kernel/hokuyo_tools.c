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
//#define THETA 0.35f
//#define SEUIL_HAUT 110 //pion
//#define SEUIL_BAS 70 
//#define COTE 350
//#define ECHANTILLIONNAGE_MIN 5
//#define ROBOT_Y_MAX 1600
//#define ROBOT_X_MAX 2600
//#define HOKU_RANGE_MIN 180 //donnée peu faible avant
//#define HOKU_RANGE_MAX 500 //donnée peu faible après
//#define HOKU_DECALAGE_START 200 //offset en y vis à vis du bord de table
//#define HOKU_SEUIL_PION 210 //distance max pour que l on considère que c est un pion
//#define HOKU_SEUIL_PION_CARRE HOKU_SEUIL_PION*HOKU_SEUIL_PION

#define HOKUYO_START_ANGLE               (-(135 / 180.0f - 44 / 512.0f) * PI)
#define HOKUYO_DTHETA         	         (PI / 512.0f)

#if 0
typedef struct
{
	int16_t x, y; 
	int64_t timestamp;
	char objet;
} hoku_pion_t;

//TODO probleme de taille
hoku_pion_t hoku_pion_table[NB_PION];
#endif

uint16_t hokuyo_tools_decode16(const unsigned char* data)
{
	uint16_t val = *data++ - 0x30;
	val <<= 6;
	val &= ~0x3f;
	val |= *data - 0x30;

	return val;
}

int hokuyo_tools_decode_buffer(const unsigned char* buffer, unsigned int buffer_size, uint16_t* distance, unsigned int distance_size)
{
	int j = 0;
	int i = 0;
	uint8_t sum = 0;
	int res = 0;
	unsigned int num_pack;
	unsigned int num_last_data;

	if( buffer_size < 23)
	{
		res = ERR_HOKUYO_SCAN_SIZE;
		goto end;
	}

	// on passe l'entête
	buffer += 23;
	buffer_size -= 23;

	num_pack = (buffer_size - 1) / 66;
	num_last_data = (buffer_size - 3 - num_pack * 66) >> 1;

	if( distance_size < 32 * num_pack + num_last_data)
	{
		res = ERR_HOKUYO_DISTANCE_BUFFER;
		goto end;
	}

	// traitement des pack de 64 data + sum + LF
	for( i = num_pack; i--; )
	{
		sum = 0;
		for( j = 32 ; j-- ; )
		{
			*distance = hokuyo_tools_decode16(buffer);
			distance++;
			sum += *buffer;
			buffer++;
			sum += *buffer;
			buffer++;
		}

		sum &= 0x3F;
		sum += 0x30;

		if( sum != *buffer)
		{
			// par sécurité, on met le code d'erreur 10
			distance -= 32;
			for( j = 32 ; j-- ; )
			{
				*distance = 10;
				distance++;
			}
			res = ERR_HOKUYO_CHECKSUM;
		}
		buffer+=2;
	}

	// TODO checksum
	// traitement du reste
	for(  ; num_last_data-- ; buffer += 2)
	{
		*distance = hokuyo_tools_decode16(buffer);
		distance++;
	}

end:
	return res;
}

void hokuyo_compute_xy(uint16_t* distance, unsigned int size, float* x, float* y, int standup)
{
	float alpha = HOKUYO_START_ANGLE;

	for( ; size--; )
	{
		if(*distance > 19)
		{
			*x = *distance * cosf(alpha);
			*y = standup * *distance * sinf(alpha);
		}
		else
		{
			*x = 0;
			*y = 0;
		}

		distance++;
		x++;
		y++;
		alpha += HOKUYO_DTHETA;
	}
}

#if 0
void hoku_init_pion(void)
{
  int i;
  
  for (i=0; i<NB_PION; i++)
    hoku_pion_table[i].objet = VIDE;

  hoku_pion_table[0].x=1225;
  hoku_pion_table[0].y=-525;
  hoku_pion_table[0].objet=PION;
  hoku_pion_table[0].timestamp=0;
  
  hoku_pion_table[1].x=1225;
  hoku_pion_table[1].y=-175;
  hoku_pion_table[1].objet=PION;
  hoku_pion_table[1].timestamp=0;  
  
  hoku_pion_table[2].x=1225;
  hoku_pion_table[2].y=175;
  hoku_pion_table[2].objet=PION;
  hoku_pion_table[2].timestamp=0;
    
  hoku_pion_table[3].x=1225;
  hoku_pion_table[3].y=525;
  hoku_pion_table[3].objet=PION;
  hoku_pion_table[3].timestamp=0;
  
  hoku_pion_table[4].x=1225;
  hoku_pion_table[4].y=875;
  hoku_pion_table[4].objet=PION;
  hoku_pion_table[4].timestamp=0;
  
  hoku_pion_table[5].x=-1225;
  hoku_pion_table[5].y=-525;
  hoku_pion_table[5].objet=PION;
  hoku_pion_table[5].timestamp=0;
  
  hoku_pion_table[6].x=-1225;
  hoku_pion_table[6].y=-175;
  hoku_pion_table[6].objet=PION;
  hoku_pion_table[6].timestamp=0;
  
  hoku_pion_table[7].x=-1225;
  hoku_pion_table[7].y=175;
  hoku_pion_table[7].objet=PION;
  hoku_pion_table[7].timestamp=0;
  
  hoku_pion_table[8].x=-1225;
  hoku_pion_table[8].y=525;
  hoku_pion_table[8].objet=PION;
  hoku_pion_table[8].timestamp=0;
  
  hoku_pion_table[9].x=-1225;
  hoku_pion_table[9].y=875;
  hoku_pion_table[9].objet=PION;
  hoku_pion_table[9].timestamp=0;
  
  hoku_pion_table[10].x=0;
  hoku_pion_table[10].y=0;
  hoku_pion_table[10].objet=PION;
  hoku_pion_table[10].timestamp=0;
  
}

void hoku_get_pion(uint16_t index, unsigned char *objet, float* x, float* y, int64_t *timestamp)
{
  if( index<NB_PION)
  {	
    *objet = hoku_pion_table[index].objet;
    *x = hoku_pion_table[index].x;
    *y = hoku_pion_table[index].y;   
    *timestamp = hoku_pion_table[index].timestamp;
  }
  
}

int hoku_update_pion(char objet, int x, int y)
{
    int i=0;
    int distance;
    int found = 0;
    int Xa, Ya;
    int Xb = x;
    int Yb = y;

    
    while( (i<NB_PION) && (hoku_pion_table[i].objet != VIDE) )
    {
  
      Xa = hoku_pion_table[i].x;
      Ya = hoku_pion_table[i].y;
      
      distance =  ((Xb - Xa)*(Xb - Xa)) + ((Yb - Ya)*(Yb - Ya));

      if( distance < HOKU_SEUIL_PION_CARRE )
      {

	found = 1;
	break; //on update seulement le timestamp
      }
      i++;
    }
    if(i==NB_PION) return 0;
	  
    hoku_pion_table[i].objet = objet;
    if(!found)
    {
      hoku_pion_table[i].x = x;
      hoku_pion_table[i].y = y;
      
    }
    hoku_pion_table[i].timestamp = systick_get_match_time();
    return 1;
    
}
#endif

//TODO à tester
/*uint8_t hoku_pion_isAlreadyKnown(int Xa, int Ya)
{
  uint8_t found = 0;
  uint8_t i=0;
  
  while( (i<NB_PION) && (hoku_pion_table[i].objet != VIDE) )
  {
  
      Xa = hoku_pion_table[i].x;
      Ya = hoku_pion_table[i].y;
      
      uint16_t distance =  ((Xb - Xa)*(Xb - Xa)) + ((Yb - Ya)*(Yb - Ya));

      if( distance < HOKU_SEUIL_PION_CARRE )
      {

	found = 1;
	break; 
      }
  }
  return found;
}*/

#if 0
float hoku_getAngleBetweenRobotAndPion(int Xrobot, int Yrobot, int Xpawn, int Ypawn)
{
	return atan2f( (Ypawn - Yrobot), (Xpawn - Xrobot) );  
}
#endif
#if 0
//TODO a tester
void hoku_updatePawnIfVisible(int Xrobot, int Yrobot, int Xpawn, float Ypawn )
{
  float angle = hoku_getAngleBetweenRobotAndPion(Xrobot, Yrobot, Xpawn, Ypawn);
  
  if( fabsf (angle) < (PI/4) )
  {
      float hokuyo_scan_pt_rad=0.00614192111437f;
      int nbPoint = angle / hokuyo_scan_pt_rad;
      int milieuPion = 341 + nbPoint;
      
      uint16_t scanDistance = hoku_scan_table[milieuPion].distance;
      
      uint16_t theroricalDistance =  ((Xrobot - Xpawn)*(Xrobot - Xpawn)) 
				      + ((Yrobot - Ypawn)*(Yrobot - Ypawn));


      if(abs( (scanDistance * scanDistance) - theroricalDistance) < 5 )
      {
	hoku_pion_table[milieuPion].timestamp = systick_get_match_time();
	//CONFIRMED PAWNNNNNNNNNNNNNN!!!!!
      }
      else 
      {
	///si on a un autre pion en vision, on considère qu il n y a plus rien 
	 if( (theroricalDistance - (scanDistance * scanDistance)) < -225 )
	   hoku_pion_table[milieuPion].objet = VIDE;
      }
      
  }      
}

/// on verifie si il y a bien un pion dans l angle de vue actuelle
void hoku_pion_table_verify_pawn(struct vect_pos *pPosRobot)
{
  uint8_t i=0;
  for(i=0; i<NB_PION; i++)
  {
    if(hoku_pion_table[i].objet!=VIDE)
    {
      hoku_updatePawnIfVisible(pPosRobot->x, 
			       pPosRobot->y, 
			       hoku_pion_table[i].x,
			       hoku_pion_table[i].y);
    }
  }
}  
#endif

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
  result1 = atanf (oppose/adjacent1) * 180 / PI;  //pt3 et pt4
  result2 = atanf (oppose/adjacent2) * 180 / PI; //pt1 et pt2
  
  point1 = (NB_POINT/2) - (result2 * THETA);
  point2 = (NB_POINT/2) + (result2 * THETA);
  point3 = (NB_POINT/2) + (result1 * THETA);
  point4 = (NB_POINT/2) - (result1 * THETA);

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
#if 0
/**
 * algo de depart
 * à eviter en match
 * 
 */
void guess_startup_conf(int start, int end)
{
	int x, y;
	int middle = ((end - start)/2) + start;
	int pionX = hoku_scan_table[middle].x/350;
	int pionY = ((abs(hoku_scan_table[middle].y)+HOKU_DECALAGE_START)/350) + 1;

	x = -1050 + (pionX * 350);
	y = -1050 + (pionY * 350);

	hoku_update_pion(PION, x, y);
	if( (pionX != 3) && (pionY != 3))
	{
		//symetrie seulement pour les autres
		hoku_update_pion(PION, (x*-1), y);
	}
}
#endif

#if 0
void parse_before_match_tab()
{

	int i=0, cmp=0;
	int first=0, second=0;
	int start, end;
	//int found=0;
	int diff=0;

	while( (i<NB_POINT) && ((hoku_scan_table[i].distance == 0) || 
		((hoku_scan_table[i].x == 0) && (hoku_scan_table[i].y == 0)))) 
		i++;
	if (i>=NB_POINT) return;
	first = i;
	start = first;
	i++;
	while(i<NB_POINT)
	{
		while( (i<NB_POINT) && (hoku_scan_table[i].distance == 0) )
			i++;
		if (i>=NB_POINT) break;
		second = i;
	
		diff = abs(hoku_scan_table[first].distance - hoku_scan_table[second].distance);

		if( diff > GAP ) /*&& (found == 0) )*/
		{

			if( cmp>=ECHANTILLIONNAGE_MIN) //filtre min 5 points pour eviter les merdes
			{
			    end = first;
			    
			    //filtre mur
			    //TODO rajouter filtre sur mur de coté getposition si -x -y alors x<200 sinon x>-200
			    if( (abs(hoku_scan_table[start].x)<ROBOT_X_MAX)&&
				(abs(hoku_scan_table[end].x)<ROBOT_X_MAX)&&	(abs(hoku_scan_table[start].y)<ROBOT_Y_MAX)&&
				(abs(hoku_scan_table[end].y)<ROBOT_Y_MAX)&&
				(start>HOKU_RANGE_MIN)&& //oeilleire gauche
				(end<HOKU_RANGE_MAX)) //oeilleire droite
			    {
			
// 			      //found something:
			      guess_startup_conf(start, end);
			    }
			    
			}
			start = second;
			cmp = 0;
// 			found = 0; 
		}
		first = second;
		cmp++;
		i++;
		
	}
}
#endif
