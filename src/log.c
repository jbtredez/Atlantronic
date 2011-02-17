//! @file log.c
//! @brief Log Task
//! @author Jean-Baptiste Trédez
//!
//! @todo translate
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
#include "priority.h"
#include <stdarg.h>

//! @todo réglage au pif
#define LOG_STACK_SIZE       64

void log_task(void *);

static xQueueHandle log_queue;

static int log_module_init()
{
	log_queue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(char* ));

	if(log_queue == 0)
	{
		return ERR_INIT_LOG;
	}

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(log_task, "log", LOG_STACK_SIZE, NULL, PRIORITY_TASK_LOG, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_LOG;
	}

	return 0;
}

module_init(log_module_init, INIT_LOG);

//!@todo vTaskDelete dans exit + gestionaire de signal pour le pc => fin propre et vérification de la mémoire avec valgrind/memcheck

int log_add(const char* msg, ...)
{
	char* buffer = pvPortMalloc(LOG_SIZE);
	//! @todo gérer une erreur d'alloc

	va_list ap;
	va_start(ap, msg);

	vsnprintf(buffer, LOG_SIZE, msg, ap);
	if(xQueueSendToBack(log_queue, &buffer, 0) != pdTRUE)
	{
		// la queue est pleine, on libère la mémoire et le message est perdu
		//! @todo led d'erreur de log ??
		vPortFree(buffer);
	}

	va_end(ap);

	return 0;
}

portBASE_TYPE log_add_from_isr(const char* msg, ...)
{
	portBASE_TYPE xHigherPriorityTaskWoken;

	char* buffer = pvPortMalloc(LOG_SIZE);
	//! @todo gérer une erreur d'alloc

	va_list ap;
	va_start(ap, msg);

	vsnprintf(buffer, LOG_SIZE, msg, ap);

	va_end(ap);

	if(xQueueSendToBackFromISR(log_queue, &buffer, &xHigherPriorityTaskWoken) != pdTRUE)
	{
		// la queue est pleine, on libère la mémoire et le message est perdu
		//! @todo led d'erreur de log ??
		vPortFree(buffer);
	}

	return xHigherPriorityTaskWoken;
}

//! Log task
void log_task(void * arg)
{
	(void) arg;

	char* msg = NULL;

	while(1)
	{
		if(xQueueReceive(log_queue, &msg, portMAX_DELAY))
		{
			size_t len = strlen(msg);
			if( fwrite(msg, 1, len, stdout) == len)
			{
				//! @todo on n'a pas pu écrire le message
			}

			vPortFree(msg);
		}
	}
}
