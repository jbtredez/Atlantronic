#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/hokuyo_tools.h"
#include "gpio.h"
#include "kernel/rcc.h"

//! @todo réglage au pif
#define TEST_HOKUYO_STACK_SIZE       100
#define HOKUYO_NUM_POINTS            682

static void test_hokuyo_task();
int test_hokuyo_module_init();

static uint16_t hokuyo_distance[HOKUYO_NUM_POINTS]; //!< distances des angles 44 à 725 du hokuyo
static float hokuyo_x[682]; //!< x des points 44 à 725
static float hokuyo_y[682]; //!< y des points 44 à 725

int test_hokuyo_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(test_hokuyo_task, "t_hoku", TEST_HOKUYO_STACK_SIZE, NULL, PRIORITY_TASK_HOKUYO, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	return 0;
}

module_init(test_hokuyo_module_init, INIT_TEST_DEPLACEMENT);

static void test_hokuyo_task()
{
//	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);

	uint32_t err;
	uint8_t res = 0;

	do
	{
		err = hokuyo_init();
		if( err)
		{
			error_raise(err);
		}
	} while(err);
	
	hoku_init_pion();
	
	while(1)
	{
		err = hokuyo_scan();
		if( err)
		{
			error_raise(err);
//			continue;
		}

		hokuyo_decode_distance(hokuyo_distance, HOKUYO_NUM_POINTS);
		
		hokuyo_compute_xy(hokuyo_distance, HOKUYO_NUM_POINTS, hokuyo_x, hokuyo_y);

		hoku_init_tab(hokuyo_distance, 682, hokuyo_x, hokuyo_y);
		//hoku_parse_tab();
		parse_before_match_tab();
		
		res = hoku_check_path();
		if(res)
		 setLed(0x55);
		else 
		  setLed(0x00);
		vTaskDelay(ms_to_tick(1000));
	}

	vTaskDelete(NULL);
}
