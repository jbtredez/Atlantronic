#include "spi.h"
#include "gyro.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/math/simpson_integrator.h"
#include "gpio.h"
#include "kernel/fault.h"
#include <math.h>

// interface gyro
#define ADXRS453_READ          0x80000000
#define ADXRS453_WRITE         0x40000000
#define ADXRS453_SENSOR_DATA   0x20000000

#define ADXRS453_REG_RATE            0x00
#define ADXRS453_REG_TEM             0x02
#define ADXRS453_REG_LOCST           0x04
#define ADXRS453_REG_HICST           0x06
#define ADXRS453_REG_QUAD            0x08
#define ADXRS453_REG_FAULT           0x0A
#define ADXRS453_REG_PID             0x0C
#define ADXRS453_REG_SN_HIGH         0x0E
#define ADXRS453_REG_SN_LOW          0x10

static int16_t gyro_v_lsb;            //!< vitesse brute du gyro (non corrigée)
static float gyro_v;                  //!< vitesse angulaire vue par le gyro
static float gyro_v_nonoise;          //!< vitesse angulaire nettoyée du bruit
static float gyro_theta_euler;        //!< angle intégré par un schéma Euler explicite
static float gyro_theta_simpson;      //!< angle intégré par un schéma Simpson
static float gyro_bias;               //!< correction deviation du gyro
static float gyro_scale;              //!< gain du gyro
static float gyro_dead_zone;          //!< permet de créer une zone morte sur le gyro pour avoir une vraie immobilité
static uint32_t gyro_dev_count;       //!< nombre de donnees utilisées pour le calcul de la correction
static int gyro_calib_mode;
static enum GyroState gyro_state;
static int gyro_init_step;            //!< nombre de cycles (1 cycle = 1ms) depuis le debut de la phase d'init
static unsigned char gyro_tx_buffer[8];
static unsigned char gyro_rx_buffer[8];
static Simpson gyro_simpson;

static void init_gyro();
static void gyro_calibration_cmd(void* arg);
static void gyro_set_position_cmd(void* arg);
static void gyro_set_calibration_values_cmd(void* arg);
static void gyro_spi_callback();
static int gyro_update(float dt);

int gyro_module_init()
{
	gyro_state = GYRO_STATE_DISCONNECTED;

	spi_register_callback(SPI_DEVICE_GYRO, gyro_spi_callback);

	usb_add_cmd(USB_CMD_GYRO_CALIB, &gyro_calibration_cmd);
	usb_add_cmd(USB_CMD_GYRO_SET_POSITION, &gyro_set_position_cmd);
	usb_add_cmd(USB_CMD_GYRO_SET_CALIBRATION_VALUES, &gyro_set_calibration_values_cmd);

	gyro_scale = 0.000218166156f;  // from datasheet

	return 0;
}

module_init(gyro_module_init, INIT_GYRO);

static void gyro_spi_callback()
{
	if( gyro_state == GYRO_STATE_RUNNING)
	{
		gyro_update(0.001f);
	}
	else if( gyro_state == GYRO_STATE_DISCONNECTED )
	{
		init_gyro();
	}
}

static int gyro_send_data(uint32_t data, uint32_t* rxdata)
{
	gyro_tx_buffer[0] = (data >> 24) & 0xff;
	gyro_tx_buffer[1] = (data >> 16) & 0xff;
	gyro_tx_buffer[2] = (data >> 8) & 0xff;
	gyro_tx_buffer[3] = data & 0xff;

	int res = spi_transaction(SPI_DEVICE_GYRO, gyro_tx_buffer, gyro_rx_buffer, 4);

	*rxdata = ((gyro_rx_buffer[0] << 24) | (gyro_rx_buffer[1] << 16) | (gyro_rx_buffer[2] << 8) | (gyro_rx_buffer[3] << 0));
	return res;
}

static void init_gyro()
{
	int res;
	uint32_t data_gyro;

	// attente de 100 ms pour init du gyro apres mise sous tension
	if( gyro_init_step == 100)
	{
		res = gyro_send_data(0x20000003, &data_gyro);
		if( res )
		{
			gyro_init_step = 0;
			fault(FAULT_GYRO_DISCONNECTED, FAULT_ACTIVE);
		}
	}
	else if( gyro_init_step == 150 || gyro_init_step == 200)
	{
		res = gyro_send_data(ADXRS453_SENSOR_DATA, &data_gyro);
		if( res )
		{
			gyro_init_step = 0;
			fault(FAULT_GYRO_DISCONNECTED, FAULT_ACTIVE);
		}
	}

	if( gyro_init_step == 250)
	{
		res = gyro_send_data(ADXRS453_SENSOR_DATA, &data_gyro);
		if( res == 0 && (data_gyro & 0xe0000000) == 0 )
		{
			gyro_calib_mode = 1;
			gyro_state = GYRO_STATE_RUNNING;
			fault(FAULT_GYRO_DISCONNECTED, FAULT_CLEAR);
		}
		else
		{
			gyro_init_step = 0;
			fault(FAULT_GYRO_DISCONNECTED, FAULT_ACTIVE);
		}
	}

	gyro_init_step++;
}

static int gyro_update(float dt)
{
	uint32_t data_gyro;
	int error = 0;
	int res = gyro_send_data(ADXRS453_SENSOR_DATA, &data_gyro);
	if( res == 0 && (data_gyro & 0xe0000000) == 0)
	{
		if( ((data_gyro & 0xC000000) == 0x4000000) && ((data_gyro & 0x04) != 0x04) )
		{
			portENTER_CRITICAL();
			gyro_v_lsb = (int16_t)((data_gyro >> 10) & 0xffff);
			portEXIT_CRITICAL();
			fault(FAULT_GYRO_ERROR, FAULT_CLEAR);
		}
		else
		{
			log_format(LOG_ERROR, "gyro error : PLL %d Q %d NVM %d POR %d PWR %d CST %d CHK %d",
				(int)(data_gyro >> 7) & 0x01,
				(int)(data_gyro >> 6) & 0x01,
				(int)(data_gyro >> 5) & 0x01,
				(int)(data_gyro >> 4) & 0x01,
				(int)(data_gyro >> 3) & 0x01,
				(int)(data_gyro >> 2) & 0x01,
				(int)(data_gyro >> 1) & 0x01);
			fault(FAULT_GYRO_ERROR, FAULT_ACTIVE);
			error = 1;
		}
	}
	else
	{
		// erreur
		fault(FAULT_GYRO_DISCONNECTED, FAULT_ACTIVE);
		gyro_state = GYRO_STATE_DISCONNECTED;
		gyro_init_step = 0;
		error = 1;
	}

	if( ! gyro_calib_mode )
	{
		// en cas d'erreur, on utilise l'ancienne valeur de vitesse
		portENTER_CRITICAL();
		gyro_v = ((float)gyro_v_lsb - gyro_bias) * gyro_scale;
		if(gyro_v < gyro_dead_zone && gyro_v > -gyro_dead_zone)
		{
			gyro_v_nonoise = 0;
		}
		else
		{
			gyro_v_nonoise = gyro_v;
		}
		gyro_theta_euler += gyro_v_nonoise * dt;
		gyro_simpson.set_derivative(dt, gyro_v_nonoise);
		gyro_simpson.compute();
		gyro_theta_simpson = gyro_simpson.get();
		portEXIT_CRITICAL();
	}
	else
	{
		// en cas d'erreur, on ne fait rien
		if( ! error )
		{
			portENTER_CRITICAL();
			gyro_bias = (gyro_dev_count * gyro_bias + gyro_v_lsb) / (gyro_dev_count + 1);
			gyro_v = ((float)gyro_v_lsb - gyro_bias) * gyro_scale;
			gyro_v_nonoise = gyro_v;
			gyro_theta_euler = 0;
			gyro_theta_simpson = 0;
			portEXIT_CRITICAL();
			gyro_dev_count++;
		}
	}

	//log_format(LOG_INFO, "GYRO %d mdeg", (int)(spi_gyro_theta * 180000 / M_PI));

	return res;
}

int16_t gyro_get_raw_data()
{
	int16_t data;

	portENTER_CRITICAL();
	data = gyro_v_lsb;
	portEXIT_CRITICAL();

	return data;
}

float gyro_get_omega()
{
	float data;

	portENTER_CRITICAL();
	data = gyro_v;
	portEXIT_CRITICAL();

	return data;
}

float gyro_get_theta_euler()
{
	float data;

	portENTER_CRITICAL();
	data = gyro_theta_euler;
	portEXIT_CRITICAL();

	return data;
}

float gyro_get_theta_simpson()
{
	float data;

	portENTER_CRITICAL();
	data = gyro_simpson.get();
	portEXIT_CRITICAL();

	return data;
}

void gyro_set_theta(float theta)
{
	portENTER_CRITICAL();
	gyro_theta_euler = theta;
	gyro_simpson.reset(theta);
	gyro_theta_simpson = gyro_simpson.get();
	portEXIT_CRITICAL();
	log_format(LOG_INFO, "gyro set theta to : %d mrad", (int)(1000*theta));
}

void gyro_calib(enum GyroCalibrationCmd cmd)
{
	float bias = 0.f;
	switch(cmd)
	{
		case GYRO_CALIBRATION_START:
			portENTER_CRITICAL();
			gyro_calib_mode = 1;
			gyro_dev_count = 0;
			gyro_bias = 0;
			portEXIT_CRITICAL();
			log(LOG_INFO, "gyro start calibration");
			break;
		case GYRO_CALIBRATION_STOP:
			gyro_calib_mode = 0;
			portENTER_CRITICAL();
			bias = gyro_bias;
			portEXIT_CRITICAL();
			log_format(LOG_INFO, "gyro stop calibration : %d mLSB/s", (int)(1000*bias));
			break;
		default:
			break;
	}
}

void gyro_calibration_cmd(void* arg)
{
	struct gyro_cmd_calibration_arg* cmd_arg = (struct gyro_cmd_calibration_arg*) arg;
	gyro_calib((enum GyroCalibrationCmd)cmd_arg->calib_cmd);
}

void gyro_set_position_cmd(void* arg)
{
	struct gyro_cmd_set_position_arg* cmd_arg = (struct gyro_cmd_set_position_arg*) arg;
	gyro_set_theta(cmd_arg->theta);
}

void gyro_set_calibration_values_cmd(void* arg)
{
	struct gyro_cmd_set_calibration_values_arg *param = (struct gyro_cmd_set_calibration_values_arg*)arg;
	portENTER_CRITICAL();
	gyro_scale = param->scale;
	gyro_bias = param->bias;
	gyro_dead_zone = param->dead_zone;
	portEXIT_CRITICAL();
}
