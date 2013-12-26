#ifndef HOKUYO_H
#define HOKUYO_H

//! @file hokuyo.h
//! @brief Hokuyo module
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/vect_plan.h"

#ifndef LINUX
#include "kernel/FreeRTOS.h"
#include "kernel/semphr.h"
#endif

#define HOKUYO_NUM_POINTS            682

enum hokuyo_id
{
	HOKUYO1,
	HOKUYO2,
	HOKUYO_MAX,
};

struct hokuyo_scan
{
	VectPlan pos_robot; //!< position absolue du robot au moment du scan
	VectPlan pos_hokuyo; //!< position du hokuyo dans le repère robot
	signed char sens; //!< sens du hokuyo (1 = vers le haut, -1 = vers le bas)
	uint16_t distance[HOKUYO_NUM_POINTS]; //!< distances des angles 44 à 725 du hokuyo
};

typedef void (*hokuyo_callback)(void);

//!< enregistrement d'un hokyuo
void hokuyo_register(enum hokuyo_id hokuyo_id, hokuyo_callback callback);

#endif
