#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "gpio.h"
#include "location/location.h"
#include "avoidance/evitement.h"
#include "avoidance/macro_fonction.h"
#include "pince.h"

//! @todo réglage au pif
#define STRATEGY_TEST_CONTROL_STACK_SIZE       100

static void strategy_test_control_task();
int strategy_test_control_module_init();

int strategy_test_control_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(strategy_test_control_task, "strategy_test_control", STRATEGY_TEST_CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	return 0;
}

module_init(strategy_test_control_module_init, INIT_STRATEGY);

static void strategy_test_control_task()
{
	int prbCase = 0;
	int numCase = 7;
	int destCase = 43; 
	
//	if(getcolor() == COLOR_BLUE)
	{
		location_set_position(-1410.0f, -850.0f, 0.0f);
	}

//	pince_configure();
#if 0
	while(1)
	{
		pince_open();
		vTaskDelay(72000000);
		pince_close();
		vTaskDelay(72000000);
	}
#endif
//	pince_open();

	//init table + robots	
	avoidance_init_table();

	//get robot color
// 	startCase = CASE_BLEU;
	
	if (getcolor() == COLOR_BLUE) 
	{
                update_table(CASE_ROUGE,ROBOT_ADV);
                //update_table(CASE_BLEU,ROBOT_ATL); inutile only
	}
	else
	{
		update_table(CASE_BLEU,ROBOT_ADV);
		//update_table(CASE_ROUGE,ROBOT_ATL); inutile debug only
	}

//	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);

	//control_straight(800);
	//control_goto(-350-350.0/2.0f, -1050+200);
	control_straight(350);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	//control_rotate(1.57);
	//vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	
	table[5+(3*COLONNE)]=OBSTACLE;
	table[4+(1*COLONNE)]=OBSTACLE;
	table[5+(4*COLONNE)]=OBSTACLE;
	table[5+(5*COLONNE)]=OBSTACLE;
	table[3+(4*COLONNE)]=OBSTACLE;

	goTo(numCase, destCase, &prbCase);

/*	control_straight(850-350);
	vTaskWaitEvent(EVENT_CONTROL_READY);
	control_rotate(1.57/2.0f);
	vTaskWaitEvent(EVENT_CONTROL_READY);
	control_straight(1000);
	vTaskWaitEvent(EVENT_CONTROL_READY);
	control_rotate(1.57/2.0f);
	vTaskWaitEvent(EVENT_CONTROL_READY);
	control_straight(1100);
	vTaskWaitEvent(EVENT_CONTROL_READY);
*/
//	control_rotate(10570);
//	vTaskWaitEvent(EVENT_CONTROL_READY);
		
//	control_straight(1800);
#if 0
	if(getcolor() == COLOR_BLUE)
	{
		control_straight(1800);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(1000);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(10570);
		vTaskWaitEvent(EVENT_CONTROL_READY);
#if 0
		control_straight(550);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(-1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(200);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(-1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(1550);
#endif
		vTaskWaitEvent(EVENT_CONTROL_READY);
	}
	else
	{
/*		control_straight(500);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(-1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(1550);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(550);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_rotate(1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY);
		control_straight(1550);*/
	}
#endif
	vTaskDelete(NULL);
}
