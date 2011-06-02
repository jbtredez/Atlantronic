#ifndef HOKUYO_TOOLS_H
#define HOKUYO_TOOLS_H

//! @file hokuyo_tools.h
//! @brief Hokuyo tools
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"

#define NB_POINT 682
#define NB_PION 19*2

#define VIDE 'V'
#define PION 'P'
#define FIGURE 'F'
#define TOUR 'T'
#define AUTRE 'A'

uint16_t hokuyo_tools_decode16(const unsigned char* data);

int hokuyo_tools_decode_buffer(const unsigned char* buffer, unsigned int buffer_size, uint16_t* distance, unsigned int distance_size);

void hokuyo_compute_xy(uint16_t* distance, unsigned int size, float* x, float* y);

void hoku_init_tab(uint16_t* distance, unsigned int size, float* x, float* y);
void hoku_parse_tab(struct vect_pos *pPosRobot);
void hoku_init_pion(void);
void hoku_get_pion(uint16_t index, unsigned char *objet, float* x, float* y, int64_t *timestamp);

void parse_before_match_tab();

uint8_t hoku_check_path();

uint8_t hoku_isPionThere(float x, float y, struct vect_pos *pPosRobot);
uint8_t check_shape(int start, int end);

void hoku_pion_table_verify_pawn(struct vect_pos *pPosRobot);

#endif
