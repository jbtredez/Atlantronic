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
#include "kernel/fault.h"

#define CONTROL_STACK_SIZE       350

static struct control_usb_data control_usb_data;
static Kinematics control_kinematics_mes[CAN_MOTOR_MAX];

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
		canopen_update(wake_time + 2);

		// mise a jour de la localisation
		int motorNotReady = 0;
		for(int i = 0; i < CAN_MOTOR_MAX; i++)
		{
			if( ! can_motor[i].is_op_enable() )
			{
				motorNotReady = 1;
			}

			control_kinematics_mes[i] = can_motor[i].kinematics;
		}

		// homing
		if( ! motorNotReady )
		{
			for(int i = 0; i < 3; i++)
			{
				if(can_motor[2*i+1].homingStatus != CAN_MOTOR_HOMING_DONE)
				{
					can_motor[2*i+1].update_homing(1);
				}
			}
		}

		if( ! motorNotReady )
		{
			// mise à jour de la position
			location_update(control_kinematics_mes, CAN_MOTOR_MAX, CONTROL_DT);
		}

		if( ! motorNotReady )
		{
			motion_compute();
		}

		motion_update_usb_data(&control_usb_data);
		control_usb_data.current_time = systick_get_time();
		control_usb_data.pos = location_get_position();
		control_usb_data.raw_data_gyro = gyro_get_raw_data();
		control_usb_data.omega_gyro = gyro_get_omega();
		control_usb_data.pos_theta_gyro_euler = gyro_get_theta_euler();
		control_usb_data.pos_theta_gyro_simpson = gyro_get_theta_simpson();
		control_usb_data.mes_v1 = control_kinematics_mes[CAN_MOTOR_DRIVING1].v;
		control_usb_data.mes_v2 = control_kinematics_mes[CAN_MOTOR_DRIVING2].v;
		control_usb_data.mes_v3 = control_kinematics_mes[CAN_MOTOR_DRIVING3].v;
		control_usb_data.mes_theta1 = control_kinematics_mes[CAN_MOTOR_STEERING1].pos;
		control_usb_data.mes_theta2 = control_kinematics_mes[CAN_MOTOR_STEERING2].pos;
		control_usb_data.mes_theta3 = control_kinematics_mes[CAN_MOTOR_STEERING3].pos;
		control_usb_data.vBat = adc_filtered_data.vBat;
		control_usb_data.iPwm[0] = adc_filtered_data.i[0];
		control_usb_data.iPwm[1] = adc_filtered_data.i[1];
		control_usb_data.iPwm[2] = adc_filtered_data.i[2];
		control_usb_data.iPwm[3] = adc_filtered_data.i[3];

		usb_add(USB_CONTROL, &control_usb_data, sizeof(control_usb_data));

		vTaskDelayUntil(&wake_time, CONTROL_PERIOD);
	}
}
