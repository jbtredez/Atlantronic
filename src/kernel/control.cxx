#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/driver/xbee.h"
#include "kernel/location/location.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/gyro/gyro.h"
#include "kernel/driver/adc.h"
#include "kernel/driver/power.h"
#include "kernel/arm.h"
#include "kernel/driver/io.h"
#include "kernel/fault.h"
#include "kernel/pump.h"
#include "kernel/match.h"
//#include "disco/elevator.h"
#include "control.h"
#include "disco/star/star.h"

#define CONTROL_STACK_SIZE       350

static struct control_usb_data control_usb_data;

static void control_task(void* arg);

static int control_module_init()
{
	portBASE_TYPE err = xTaskCreate(control_task, "control", CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	return 0;
}

module_init(control_module_init, INIT_CONTROL);

static void control_task(void* /*arg*/)
{
	uint32_t wake_time = 0;
	int xbeeCycleCount = 0;

	while(1)
	{
		// mise a jour adc
		adc_update();

		// mise a jour du can
		//canopen_update(wake_time + 3);
		can_mip_update(wake_time + 5);

		// mise a jour de la loc et calcul asservissement
		motion.compute();

		control_usb_data.pumpState = pump_update();

		motion.updateUsbData(&control_usb_data);
		control_usb_data.current_time = systick_get_time();
		control_usb_data.pos = location.getPosition();
//		control_usb_data.raw_data_gyro = gyro_get_raw_data();
//		control_usb_data.omega_gyro = gyro_get_omega();
//		control_usb_data.pos_theta_gyro_euler = gyro_get_theta_euler();
//		control_usb_data.pos_theta_gyro_simpson = gyro_get_theta_simpson();
		control_usb_data.vBat = adc_filtered_data.vBat;
		control_usb_data.iPwm[0] = adc_filtered_data.i[0];
		control_usb_data.iPwm[1] = adc_filtered_data.i[1];
		control_usb_data.iPwm[2] = adc_filtered_data.i[2];
		control_usb_data.iPwm[3] = adc_filtered_data.i[3];
		control_usb_data.encoder[ENCODER_1] = encoder_ab_get(ENCODER_1);
		control_usb_data.encoder[ENCODER_2] = encoder_ab_get(ENCODER_2);
		control_usb_data.encoder[ENCODER_3] = encoder_ab_get(ENCODER_3);
		control_usb_data.gpio = gpio_get_state();
		control_usb_data.power_state = power_get();
		control_usb_data.color = match_get_color();
//		control_usb_data.elevatorHeight = elevator_get_position();
//		arm_get_matrix(&control_usb_data.arm_matrix);

		ax12.updateUsbData(&control_usb_data.ax12);
		//rx24.updateUsbData(&control_usb_data.rx24);

		usb_add(USB_CONTROL, &control_usb_data, sizeof(control_usb_data));

		// en xbee, on diminue la frequence pour la bande passante
		xbeeCycleCount++;
		if( xbeeCycleCount > 100)
		{
			struct control_usb_data_light* data = &control_usb_data;
			xbee_add(USB_CONTROL_LIGHT, data, sizeof(*data));
			xbeeCycleCount = 0;
		}

		vTaskDelayUntil(&wake_time, CONTROL_PERIOD);
	}
}
