#ifndef CONTROL_PINCE_H
#define CONTROL_PINCE_H

//! @file control_pince.h
//! @brief Asservissement des pinces
//! @author Atlantronic

#include <stdint.h>


#define PINCE_POS_LOW      150
#define PINCE_POS_MED     2000
#define PINCE_POS_HI      3300

enum control_pince_state
{
	CONTROL_PINCE_READY_ASSER,
	CONTROL_PINCE_READY_FREE,
	CONTROL_PINCE_INDEPENDANT,
	CONTROL_PINCE_DUAL,
	CONTROL_PINCE_END,    //!< end : halted forever
};

void control_pince_independant(float h1, float h2);

void control_pince_dual(float d, float alpha);

void control_pince_free();

int32_t control_pince_get_state();

#endif
