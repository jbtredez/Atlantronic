//! @file hokuyo_tools.c
//! @brief Hokuyo tools
//! @author Atlantronic

#include "kernel/hokuyo_tools.h"
#include "kernel/robot_parameters.h"
#include "kernel/systick.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define GAP 90 //150
#define THETA 0.35f
#define SEUIL_HAUT 110 //pion
#define SEUIL_BAS 70 
#define COTE 350
#define ECHANTILLIONNAGE_MIN 5
#define ROBOT_Y_MAX 1600
#define ROBOT_X_MAX 2600
#define HOKU_RANGE_MIN 180 //donnée peu faible avant
#define HOKU_RANGE_MAX 500 //donnée peu faible après
#define HOKU_DECALAGE_START 200 //offset en y vis à vis du bord de table
#define HOKU_SEUIL_PION 210 //distance max pour que l on considère que c est un pion
#define HOKU_SEUIL_PION_CARRE HOKU_SEUIL_PION*HOKU_SEUIL_PION


typedef struct {
 int16_t x, y; 
 int64_t timestamp;
 char objet;

} hoku_pion_t;

typedef struct {
  uint16_t distance;
  float x;
  float y;

} hoku_scan_t;

hoku_scan_t hoku_scan_table[NB_POINT];
//TODO probleme de taille
hoku_pion_t hoku_pion_table[NB_PION];


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
		// TODO ERR code
		res = -1;
		goto end;
	}

	// on passe l'entête
	buffer += 23;
	buffer_size -= 23;

	num_pack = (buffer_size - 1) / 66;
	num_last_data = (buffer_size - 3 - num_pack * 66) >> 1;

	if( distance_size < 32 * num_pack + num_last_data)
	{
		// TODO ERR code
		res = -2;
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
			// erreur checksum
			// TODO led
			// par sécurité, on met le code d'erreur 10
			distance -= 32;
			for( j = 32 ; j-- ; )
			{
				*distance = 10;
				distance++;
			}
			res = -1;
		}
		buffer+=2;
	}

	// traitement du reste
	for(  ; num_last_data-- ; buffer += 2)
	{
		*distance = hokuyo_tools_decode16(buffer);
		distance++;
	}

end:
	return res;
}

void hokuyo_compute_xy(uint16_t* distance, unsigned int size, float* x, float* y)
{
	float alpha = -(135 / 180.0f - 44 / 512.0f) * PI;
	const float pas = PI / 512.0f;

	for( ; size--; )
	{
		if(*distance > 19)
		{
			*x = *distance * (float)cos(alpha);
			*y = - *distance * (float)sin(alpha);
		}
		else
		{
			*x = 0;
			*y = 0;
		}

		distance++;
		x++;
		y++;
		alpha += pas;
	}
}

///////// the chupa touch
void hoku_init_tab(uint16_t* distance, unsigned int size, float* x, float* y)
{
	unsigned int i;
	for(i=0;i<size;i++)
	{	     
             hoku_scan_table[i].distance = distance[i];
             hoku_scan_table[i].x = x[i];
             hoku_scan_table[i].y = y[i]; 
	}
}

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

#if 0
void hoku_print_pion()
{
  int i;
  for (i=0; i<NB_PION; i++)
    if( hoku_pion_table[i].objet != VIDE) printf("%c en (%d,%d) soit ligne=%d et croisement=%d\n", 
	hoku_pion_table[i].objet, 
	hoku_pion_table[i].x,
	hoku_pion_table[i].y,
	hoku_pion_table[i].ligne,
	hoku_pion_table[i].croisement);
}
#endif 


unsigned char check_shape(int start, int end)
{
	
	int D1 = hoku_scan_table[start].distance;
	//int D2 = hoku_scan_table[end].distance;
	float adjacent = 0.0;
	//milieu du point
	//int milieu = (end + start)/2;
	
	float angle_deg = THETA*((end-start)/2);
	float angle_rad = PI * ( angle_deg / 180); 
	
	adjacent = (float)sin( angle_rad ) * D1;	

	if( (adjacent < SEUIL_HAUT) && (adjacent > SEUIL_BAS) )
	{
		//PION DETECTED 
		return PION ;//TODO 
		//hoku_scan_table[milieu].x, hoku_scan_table[milieu].y
	}
	return AUTRE;
}

void hoku_parse_tab(void)
{

	int i=0;
	int first=0, second=0;
	int start, end;
	int found=0;
	int diff=0;

	while( (i<NB_POINT) && ((hoku_scan_table[i].distance == 0) || 
		((hoku_scan_table[i].x == 0) && (hoku_scan_table[i].y == 0)))) 
		i++;
	if (i>=NB_POINT) return;
	first = i;
	i++;
	while(i<NB_POINT)
	{
		while( (i<NB_POINT) && (hoku_scan_table[i].distance == 0) )
			i++;
		if (i>=NB_POINT) break;
		second = i;
	
		diff = hoku_scan_table[first].distance - hoku_scan_table[second].distance;

		if( diff > GAP ) /*&& (found == 0) )*/
		{

			found = 1;
			start = second;
		}
		else if( (diff < (GAP*(-1)) ) && (found == 1) )
		{
			if( (i - start)>ECHANTILLIONNAGE_MIN) //filtre min 5 points pour eviter les merdes
			{
			  end = first;
			  //on verifie la forme
			  if(check_shape(start, end)==PION)
			  {
			    int milieu = (end + start)/2;
			    //TODO coordonnée de la table
			    hoku_update_pion(PION, 
					     hoku_scan_table[milieu].x, 
					     hoku_scan_table[milieu].y);
			  }
			}
			start = second;
			
			found = 0; 
		}
		first = second;
		i++;
	}
}

/**
 * renvoie 1 si chemin degager, 0 sinon
 * 
 */
uint8_t hoku_check_path()
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
  //TODO atan2
  result1 = (float)atan (oppose/adjacent1) * 180 / PI;  //pt3 et pt4
  result2 = (float)atan (oppose/adjacent2) * 180 / PI; //pt1 et pt2
  
  point1 = (NB_POINT/2) - (result2 * THETA);
  point2 = (NB_POINT/2) + (result2 * THETA);
  point3 = (NB_POINT/2) + (result1 * THETA);
  point4 = (NB_POINT/2) - (result1 * THETA);

  //pt1 et pt2
  angle_rad = PI * ( result2 / 180); 
  limite2 = oppose / (float)sin(angle_rad);

  //pt3 et pt4
  angle_rad = PI * ( result1 / 180); 
  limite1 = oppose / (float)sin(angle_rad);*/

  if( ( (hoku_scan_table[point1].distance > limite2) || (hoku_scan_table[point1].distance == 0) ) && 
      ( (hoku_scan_table[point2].distance > limite2) || (hoku_scan_table[point2].distance == 0) ) &&
      ( (hoku_scan_table[point3].distance > limite1) || (hoku_scan_table[point3].distance == 0) ) && 
      ( (hoku_scan_table[point4].distance > limite1) || (hoku_scan_table[point4].distance == 0) ) )
  {
	res = 1;
	// chemin degage!!!!!
  }	
  return res;
}


/**
 * algo de depart
 * à eviter en mathc
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
    if( (pionX != 3) && (pionY != 3)) //symetrie seulement pour les autres
      hoku_update_pion(PION, (x*-1), y);

    
}


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