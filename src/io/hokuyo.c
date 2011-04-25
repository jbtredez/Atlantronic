//! @file hokuyo.c
//! @brief Hokuyo module
//! @author Jean-Baptiste Trédez

#include "io/hokuyo.h"
#include "module.h"
#include "io/usart.h"

#define HOKUYO_STACK_SIZE       100


const char* hokuyo_scip2_cmd = "SCIP2.0\n";
const char* hokuyo_speed_cmd = "SS750000\n";
const char* hokuyo_laser_on_cmd = "BM\n";
static uint8_t hokuyo_read_dma_buffer[30];

static void hokuyo_task(void *arg);
static void hokuyo_scip2();
static void hokuyo_set_speed();
static void hokuyo_laser_on();

static int hokuyo_module_init(void)
{
	usart_open(USART3_FULL_DUPLEX, 19200);

	usart_set_read_dma_buffer(USART3_FULL_DUPLEX, hokuyo_read_dma_buffer);

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(hokuyo_task, "hokuyo", HOKUYO_STACK_SIZE, NULL, PRIORITY_TASK_HOKUYO, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_HOKUYO;
	}

	return 0;
}

module_init(hokuyo_module_init, INIT_HOKUYO);

static void hokuyo_task(void *arg)
{
	(void) arg;

	hokuyo_scip2();
	hokuyo_set_speed();

	usart_set_frequency(USART3_FULL_DUPLEX, 750000);

	hokuyo_laser_on();

	while(1)
	{
	
	}
}

static void hokuyo_scip2()
{
	usart_set_write_dma_buffer(USART3_FULL_DUPLEX, (unsigned char*) hokuyo_scip2_cmd);

	usart_set_read_dma_size(USART3_FULL_DUPLEX, 12);
	usart_send_dma_buffer(USART3_FULL_DUPLEX, 8);

	// TODO timeout, gestion erreur
	usart_wait_read(USART3_FULL_DUPLEX, 720000);
}

static void hokuyo_set_speed()
{
	usart_set_write_dma_buffer(USART3_FULL_DUPLEX, (unsigned char*) hokuyo_speed_cmd);

	usart_set_read_dma_size(USART3_FULL_DUPLEX, 14);
	usart_send_dma_buffer(USART3_FULL_DUPLEX, 9);

	// TODO timeout, gestion erreur
	usart_wait_read(USART3_FULL_DUPLEX, 720000);
}

static void hokuyo_laser_on()
{
	usart_set_write_dma_buffer(USART3_FULL_DUPLEX, (unsigned char*) hokuyo_laser_on_cmd);

	usart_set_read_dma_size(USART3_FULL_DUPLEX, 8);
	usart_send_dma_buffer(USART3_FULL_DUPLEX, 3);

	// TODO timeout, gestion erreur
	usart_wait_read(USART3_FULL_DUPLEX, 720000);

	// reponse, rep = BM\n ; status (00) ; sum (P) ; \n\n
	
}
