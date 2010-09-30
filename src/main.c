//! @file main.c
//! @brief Programme principal
//! @author Jean-Baptiste Trédez

#ifdef __GCC_POSIX__
	#include <time.h>
	#include <signal.h>
	#include <unistd.h>
	#include <pthread.h>
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "module.h"
#include "log.h"

#ifdef __GCC_POSIX__
static struct sigaction action;
static int n_sig_int;

void* end_sched(void *arg)
{
	(void) arg;
	vTaskEndScheduler();

	return NULL;
}

static void signal_handler(int sig)
{
	(void) sig;
	n_sig_int++;
	if(n_sig_int > 2)
	{
		kill(getpid(),SIGKILL);
	}
	else
	{
		pthread_t id;
		pthread_create(&id,NULL,end_sched,NULL);
	}
}
#endif

void init_panic(int init)
{
	(void) init;

	//! @todo allumer une led
	while(1)
	{

	}
}

int main()
{
	#ifdef __GCC_POSIX__
	n_sig_int = 0;
	action.sa_handler = signal_handler;
	sigemptyset(&(action.sa_mask));
	if(sigaction(SIGINT, &action, NULL))
	{
		logerror("sigaction");
	}
	#endif

	int init = initModules();

	if(init)
	{		
		init_panic(init);
	}

	vTaskStartScheduler();

	// on n'arrive jamais ici sur un pic
	#ifdef __GCC_POSIX__
	exitModules();
	#endif

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
		xTimeToSleep.tv_sec = 0;
		xTimeToSleep.tv_nsec = 1000;
		nanosleep( &xTimeToSleep, &xTimeSlept );
	#endif
}

