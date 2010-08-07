//! @file temps.c
//! @brief Gestion du temps et de la durée du match
//! @author Jean-Baptiste Trédez

#include "FreeRTOS.h"
#include "task.h"
#include "temps.h"
#include "module.h"
#include "log.h"

// on souhaite que la fonction vApplicationTickHook() soit appelée à chaque interruption du timer
// autant générer une erreur si on le désactive par mégarde 
#if ( configUSE_TICK_HOOK == 0 )
	#error "il faut mettre configUSE_TICK_HOOK à 1 pour la gestion du temps"
#endif

//! durée du match en tick
#define DUREE_MATCH_TICK      (90 * configTICK_RATE_HZ)

//! temps (modulo 2^32 tick) depuis le lancement du pic (depuis l'appel de vTaskStartScheduler())
//!   - codé sur au moins 32 bits : à 1kHz, il faut 90000 tick pour un match (> 2^16)
//!
//! ATTENTION : membre privé au fichier :
//!   - mis à jour en interruption.
//!   - utiliser les fonctions d'accès définies dans ce fichier, elles sont sécurisées
static volatile unsigned long temps;

//! temps en tick au début du match
static volatile unsigned long tempsDebutMatch;

//! temps en tick indiquant la fin du du match
static volatile unsigned long tempsFinMatch;

//! fonction d'initialisation du module
//! ATTENTION : la fonction doit être appelée avant l'activation du timer (donc avant vTaskStartScheduler())
static int _initModuleTemps()
{
	// interruption du timer non lancée, pas de protection
	temps = 0;
	tempsDebutMatch = 0;
	tempsFinMatch = 0;

	// pas d'erreur
	return 0;
}

// enregistrement de la fonction d'initialisation
module_init(_initModuleTemps, INIT_TEMPS);

//! Fonction appelée automatiquement à chaque interruption de l'horloge (configUSE_TICK_HOOK == 1)
//! BUT :
//!   - gestion du temps
//!   - arrêt du robot à la fin du match
//!
//! ATTENTION : elle est exécutée en interruption
//!
//! remarque importante : la variable xTickCount (de tasks.c) compte le nombre de tick réalisés par l'ordonanceur.
//! Elle n'est pas utilisée pour le temps car :
//!   - xTickCount n'est PAS mis à jour quand l'ordonanceur est désactivé. La variable uxMissedTicks (de tasks.c) compte alors le nombre de tick raté et dès que l'ordonanceur est de nouveau lancé, il met à jour xTickCount.
//!   - on ne doit pas toucher (mettre à zéro par exemple) à cette variable sinon on aura des problèmes avec les taches retardées.
void vApplicationTickHook()
{
	temps++;
	if(temps == tempsFinMatch)
	{
		if(tempsDebutMatch != tempsFinMatch)
		{
			tempsDebutMatch = tempsFinMatch;
			meslogFromISR(_info_, 1, "fin du match");
		}
	}
}

unsigned long tempsSysteme()
{
	unsigned long tps;

	portENTER_CRITICAL();
	tps = temps;
	portEXIT_CRITICAL();

	return tps;
}

unsigned long tempsMatch()
{
	unsigned long tps = 0;

	portENTER_CRITICAL();
	if(tempsDebutMatch != tempsFinMatch)
	{
		tps = temps - tempsDebutMatch;
	}
	portEXIT_CRITICAL();

	return tps;
}

unsigned long tempsSystemeFromISR()
{
	return temps;
}

unsigned long tempsMatchFromISR()
{
	unsigned long tps = 0;

	if(tempsDebutMatch != tempsFinMatch)
	{
		tps = temps - tempsDebutMatch;
	}

	return tps;
}

void tempsStartMatch()
{
	portENTER_CRITICAL();
	tempsDebutMatch = temps;
	tempsFinMatch = temps + DUREE_MATCH_TICK;
	portEXIT_CRITICAL();
}

void tempsStartMatchFromISR()
{
	tempsDebutMatch = temps;
	tempsFinMatch = temps + DUREE_MATCH_TICK;
}
