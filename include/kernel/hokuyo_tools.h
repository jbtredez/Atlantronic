#ifndef HOKUYO_TOOLS_H
#define HOKUYO_TOOLS_H

//! @file hokuyo_tools.h
//! @brief Hokuyo tools
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"

#define NB_PION 19*2

#define VIDE 'V'
#define PION 'P'
#define FIGURE 'F'
#define TOUR 'T'
#define AUTRE 'A'

struct hokuyo_object
{
	uint16_t start;
	uint16_t stop;
};

uint16_t hokuyo_tools_decode16(const unsigned char* data);

int hokuyo_tools_decode_buffer(const unsigned char* buffer, unsigned int buffer_size, uint16_t* distance, unsigned int distance_size);

void hokuyo_compute_xy(uint16_t* distance, unsigned int size, float* x, float* y, int standup);

int hokuyo_find_objects(uint16_t* distance, unsigned int size, struct hokuyo_object* obj, unsigned int obj_size);

int hokuyo_object_is_pawn(uint16_t* distance, struct hokuyo_object* obj, struct vect_pos *pawn_pos);

void hoku_init_pion(void);
void hoku_get_pion(uint16_t index, unsigned char *objet, float* x, float* y, int64_t *timestamp);

void parse_before_match_tab();

uint8_t hoku_check_path();

uint8_t check_shape(uint16_t* distance, unsigned int start, unsigned int end);

void hoku_pion_table_verify_pawn(struct vect_pos *pPosRobot);

uint16_t distance_forward_shape(uint16_t* distance, unsigned int size);

int is_pawn_front_start();

#endif
