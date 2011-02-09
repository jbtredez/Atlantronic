//! @file time2s.c
//! @brief Time module
//! @author Jean-Baptiste Trédez

#include "FreeRTOS.h"
#include "task.h"
#include "time2.h"
#include "module.h"
#include "log.h"
#include "event.h"

// configUSE_TICK_HOOK has to be set
#if ( configUSE_TICK_HOOK == 0 )
	#error "configUSE_TICK_HOOK needed"
#endif

//! durée du match en tick
#define DUREE_MATCH_TICK      (90 * configTICK_RATE_HZ)

//! time (modulo 2^32 tick) depuis le lancement du pic (depuis l'appel de vTaskStartScheduler())
//!   - codé sur au moins 32 bits : à 1kHz, il faut 90000 tick pour un match (> 2^16)
//!
//! ATTENTION : membre privé au fichier :
//!   - mis à jour en interruption.
//!   - utiliser les fonctions d'accès définies dans ce fichier, elles sont sécurisées
static volatile unsigned long time;

//! time en tick au début du match
static volatile unsigned long time_start;

//! time en tick indiquant la fin du du match
static volatile unsigned long time_end;

static int time_module_init()
{
	// scheduler not started, no lock
	time = 0;
	time_start = 0;
	time_end = 0;

	return 0;
}

module_init(time_module_init, INIT_TIME);

//! Fonction appelée automatiquement à chaque interruption de l'horloge (configUSE_TICK_HOOK == 1)
//! BUT :
//!   - gestion du time
//!   - arrêt du robot à la fin du match
//!
//! ATTENTION : elle est exécutée en interruption
//!
//! remarque importante : la variable xTickCount (de tasks.c) compte le nombre de tick réalisés par l'ordonanceur.
//! Elle n'est pas utilisée pour le time car :
//!   - xTickCount n'est PAS mis à jour quand l'ordonanceur est désactivé. La variable uxMissedTicks (de tasks.c) compte alors le nombre de tick raté et dès que l'ordonanceur est de nouveau lancé, il met à jour xTickCount.
//!   - on ne doit pas toucher (mettre à zéro par exemple) à cette variable sinon on aura des problèmes avec les taches retardées.
void vApplicationTickHook()
{
	time++;
	if(time == time_end)
	{
		if(time_start != time_end)
		{
			meslogFromISR(_info_, 1, "fin du match");
			vTaskSetEventFromISR(EVENT_END);
			time_start = time_end;
		}
	}
}

unsigned long time_sys()
{
	unsigned long t;

	portENTER_CRITICAL();
	t = time;
	portEXIT_CRITICAL();

	return t;
}

unsigned long time_match()
{
	unsigned long t = 0;

	portENTER_CRITICAL();
	if(time_start != time_end)
	{
		t = time - time_start;
	}
	portEXIT_CRITICAL();

	return t;
}

unsigned long time_sys_from_isr()
{
	return time;
}

unsigned long time_match_from_isr()
{
	unsigned long t = 0;

	if(time_start != time_end)
	{
		t = time - time_start;
	}

	return t;
}

void time_start_match()
{
	portENTER_CRITICAL();
	time_start = time;
	time_end = time + DUREE_MATCH_TICK;
	portEXIT_CRITICAL();
}

void time_start_match_from_isr()
{
	time_start = time;
	time_end = time + DUREE_MATCH_TICK;
}
