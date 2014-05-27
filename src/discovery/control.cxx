#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/can_motor.h"
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "control.h"
#include "kernel/location/location.h"
#include "kernel/geometric_model/geometric_model.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/gyro.h"
#include "kernel/driver/adc.h"
#include "kernel/driver/power.h"
#include "kernel/arm.h"
#include "kernel/heartbeat.h"
#include "gpio.h"
#include "kernel/fault.h"
#include "kernel/pump.h"

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

	while(1)
	{
		// mise a jour adc
		adc_update();

		// mise a jour du can
		canopen_update(wake_time + 3);

		// mise a jour de la loc et calcul asservissement
		motion_compute();

		// mise a jour heartbeat
		heartbeat_update();

		control_usb_data.pumpState = pump_update();

		motion_update_usb_data(&control_usb_data);
		control_usb_data.current_time = systick_get_time();
		control_usb_data.pos = location_get_position();
		control_usb_data.raw_data_gyro = gyro_get_raw_data();
		control_usb_data.omega_gyro = gyro_get_omega();
		control_usb_data.pos_theta_gyro_euler = gyro_get_theta_euler();
		control_usb_data.pos_theta_gyro_simpson = gyro_get_theta_simpson();
		control_usb_data.vBat = adc_filtered_data.vBat;
		control_usb_data.iPwm[0] = adc_filtered_data.i[0];
		control_usb_data.iPwm[1] = adc_filtered_data.i[1];
		control_usb_data.iPwm[2] = adc_filtered_data.i[2];
		control_usb_data.iPwm[3] = adc_filtered_data.i[3];
		control_usb_data.encoder[ENCODER_1] = encoder_get(ENCODER_1);
		control_usb_data.encoder[ENCODER_2] = encoder_get(ENCODER_2);
		control_usb_data.encoder[ENCODER_3] = encoder_get(ENCODER_3);
		control_usb_data.gpio = gpio_get_state();
		control_usb_data.power_state = power_get();
		arm_get_matrix(&control_usb_data.arm_matrix);

		dynamixel_update_usb_data(&control_usb_data.dynamixel);

		usb_add(USB_CONTROL, &control_usb_data, sizeof(control_usb_data));


		vTaskDelayUntil(&wake_time, CONTROL_PERIOD);
	}
}
