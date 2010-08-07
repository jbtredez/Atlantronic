//! @file log.c
//! @brief Tache de log
//! @author Jean-Baptiste Trédez
//!
//! Le but est de gérer l'écriture des log dans une tache de faible priorité.
//! L'utilisation d'une tache permet aux autres taches de ne pas attendre l'écriture
//! avant de poursuivre leur exécution.
//! On se contente d'allouer de la mémoire et de donner un pointeur vers le message à envoyer
//! (cf macro meslog). La tache de log s'occupera d'envoyer les log quand on aura le temps de 
//! le faire et à sa vitesse. Elle s'occupe de libérer la mémoire alloué.

#include "FreeRTOS.h"
#include "task.h"
#include "module.h"
#include "queue.h"
#include "log.h"
#include "string.h"
#include "priorite.h"
#include <stdarg.h>

//! @todo réglage au pif
#define TAILLE_STACK_LOG       10

void tacheLog(void *);

static xQueueHandle queueLog;

//! fonction d'initialisation du module
//! ATTENTION : la fonction doit être appelée avant l'activation du timer (donc avant vTaskStartScheduler())
int _initModuleLog()
{
	queueLog = xQueueCreate(NB_ELEMENT_QUEUE_LOG, sizeof(char* ));

	if(queueLog == 0){
		return ERR_INIT_LOG;
	}

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(tacheLog, (const signed char *) "log", TAILLE_STACK_LOG, NULL, PRIORITE_TACHE_LOG, &xHandle);

	if(err != pdPASS){
		return ERR_INIT_LOG;
	}

	// pas d'erreur
	return 0;
}

// enregistrement de la fonction d'initialisation
module_init(_initModuleLog, INIT_LOG);

//!@todo vTaskDelete dans exit + gestionaire de signal pour le pc => fin propre et vérification de la mémoire avec valgrind/memcheck

int ajouterLog(const char* msg, ...)
{
	char* buffer = pvPortMalloc(TAILLE_MESSAGE_MAX);
	//! @todo gérer une erreur d'alloc

	va_list ap;
	va_start(ap, msg);

	vsnprintf(buffer, TAILLE_MESSAGE_MAX, msg, ap);
	if(xQueueSendToBack(queueLog, &buffer, 0) != pdTRUE) {
		// la queue est pleine, on libère la mémoire et le message est perdu
		//! @todo led d'erreur de log ??
		vPortFree(buffer);
	}

	va_end(ap);

	return 0;
}

int ajouterLogFromISR(const char* msg, ...)
{
	portBASE_TYPE xHigherPriorityTaskWoken;

	char* buffer = pvPortMalloc(TAILLE_MESSAGE_MAX);
	//! @todo gérer une erreur d'alloc

	va_list ap;
	va_start(ap, msg);

	vsnprintf(buffer, TAILLE_MESSAGE_MAX, msg, ap);

	va_end(ap);

	if(xQueueSendToBackFromISR(queueLog, &buffer, &xHigherPriorityTaskWoken) != pdTRUE) {
		// la queue est pleine, on libère la mémoire et le message est perdu
		//! @todo led d'erreur de log ??
		vPortFree(buffer);
	}

	// on a réveillé une tache de priorité plus importante que la tache actuelle (interrompue)
	// la fonction bug de temps en temps avec la simulation sous linux, on s'en passe
	#ifndef __GCC_POSIX__
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	#endif

	return 0;
}


//! tache de log
//! en cas d'erreur d'initialisation, la tache n'est pas crée et ne sera donc pas lancée
void tacheLog(void * arg)
{
	(void) arg;

	char* msg = NULL;

	while(1){
		if(xQueueReceive(queueLog, &msg, portMAX_DELAY))
		{
			size_t taille = strlen(msg);
			if( fwrite(msg, 1, taille, stdout) == taille)
			{
				//! @todo on n'a pas pu écrire le message
			}

			// on libère la mémoire
			vPortFree(msg);
		}
	}
}
