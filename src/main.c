//! @file main.c
//! @brief Programme principal
//! @author Jean-Baptiste Trédez

#ifdef __GCC_POSIX__
	#include <time.h>
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "module.h"
#include "log.h"

int main()
{
	int init = initModules();
	if(init){
		if(init < ERR_INIT_LOG){
			// on a initialisé les log
			meslog(_erreur_, 0, "init : %i", init);
		}
		//! @todo allumer une led
	}

	vTaskStartScheduler();

	// on n'arrive jamais ici sur un pic

	return 0;
}

void vApplicationIdleHook()
{
	// si on utilise les coroutines, on va ordonancer le tout dans la tache idle
	#if(configUSE_CO_ROUTINES == 1)
		vCoRoutineSchedule();
	#endif

	// en simulation, histoire de ne pas pomper 100% du temps processeur, on met un nanosleep dans la tache idle
	#ifdef __GCC_POSIX__
		struct timespec xTimeToSleep, xTimeSlept;
		xTimeToSleep.tv_sec = 1;
		xTimeToSleep.tv_nsec = 0;
		nanosleep( &xTimeToSleep, &xTimeSlept );
	#endif
}

