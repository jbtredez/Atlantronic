#ifndef SYSTICK_H
#define SYSTICK_H

//! @file systick.h
//! @brief Time module
//! @author Atlantronic
#ifndef LINUX
#include "kernel/cpu/cpu.h"
#endif
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! structure pour le temps
//! on utilise des ms et ns au lieu des secondes et ns pour simplifier le code
typedef struct systime
{
	uint32_t ms; //!< temps en ms
	int32_t ns; //!< reste du temps en ns
} Systime;

//!< soustraction entre 2 systime
Systime timediff(const Systime t2, const Systime t1);

//!< addition entre 2 systime
Systime timeadd(const Systime t1, const Systime t2);

//!< recuperation du temps depuis le demarrage
Systime systick_get_time(void);

//!< recuperation du temps depuis le demarrage (depuis une IT)
Systime systick_get_time_from_isr(void);

//!< recuperation du temps depuis le debut du match
Systime systick_get_match_time(void);

//!< enregistrement du temps du debut du match (si match non débuté)
void systick_start_match(void);

//!< enregistrement du temps du debut du match (si match non débuté)
void systick_start_match_from_isr(void);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
static inline Systime operator +(const Systime t1, const Systime t2)
{
	return timeadd(t1, t2);
}

static inline Systime operator -(const Systime t2, const Systime t1)
{
	return timediff(t2, t1);
}
#endif

#endif
