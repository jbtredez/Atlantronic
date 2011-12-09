#ifndef HOKUYO_H
#define HOKUYO_H

//! @file hokuyo.h
//! @brief Hokuyo module
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"

#define HOKUYO_NUM_POINTS            682

struct hokuyo_scan
{
	struct vect_pos pos_robot; //!< position absolue du robot au moment du scan
	struct vect_pos pos_hokuyo; //!< position du hokuyo dans le repère robot
	char sens; //!< sens du hokuyo (1 = vers le haut, -1 = vers le bas)
	uint16_t distance[HOKUYO_NUM_POINTS]; //!< distances des angles 44 à 725 du hokuyo
};

uint32_t hokuyo_init();
void hokuyo_start_scan();
uint32_t hokuto_wait_decode_scan(uint16_t* distance, int size);

#endif