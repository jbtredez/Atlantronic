#include "stmpe811.h"
#include "kernel/driver/i2c.h"
#include "kernel/driver/exti.h"
#include "kernel/driver/gpio.h"
#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/log.h"

/* chip IDs */
#define STMPE811_ID                     0x0811

/* Identification registers & System Control */
#define STMPE811_REG_CHP_ID_LSB         0x00
#define STMPE811_REG_CHP_ID_MSB         0x01
#define STMPE811_REG_ID_VER             0x02

/* Global interrupt Enable bit */
#define STMPE811_GIT_EN                 0x01

/* IO expander functionalities */
#define STMPE811_ADC_FCT                0x01
#define STMPE811_TS_FCT                 0x02
#define STMPE811_IO_FCT                 0x04
#define STMPE811_TEMPSENS_FCT           0x08

/* Global Interrupts definitions */
#define STMPE811_GIT_IO                 0x80  /* IO interrupt                   */
#define STMPE811_GIT_ADC                0x40  /* ADC interrupt                  */
#define STMPE811_GIT_TEMP               0x20  /* Not implemented                */
#define STMPE811_GIT_FE                 0x10  /* FIFO empty interrupt           */
#define STMPE811_GIT_FF                 0x08  /* FIFO full interrupt            */
#define STMPE811_GIT_FOV                0x04  /* FIFO overflowed interrupt      */
#define STMPE811_GIT_FTH                0x02  /* FIFO above threshold interrupt */
#define STMPE811_GIT_TOUCH              0x01  /* Touch is detected interrupt    */
#define STMPE811_ALL_GIT                0x1F  /* All global interrupts          */
#define STMPE811_TS_IT                  (STMPE811_GIT_TOUCH | STMPE811_GIT_FTH |  STMPE811_GIT_FOV | STMPE811_GIT_FF | STMPE811_GIT_FE) /* Touch screen interrupts */

/* General Control Registers */
#define STMPE811_REG_SYS_CTRL1          0x03
#define STMPE811_REG_SYS_CTRL2          0x04
#define STMPE811_REG_SPI_CFG            0x08

/* Interrupt system registers */
#define STMPE811_REG_INT_CTRL           0x09
#define STMPE811_REG_INT_EN             0x0A
#define STMPE811_REG_INT_STA            0x0B
#define STMPE811_REG_IO_INT_EN          0x0C
#define STMPE811_REG_IO_INT_STA         0x0D

/* IO Registers */
#define STMPE811_REG_IO_SET_PIN         0x10
#define STMPE811_REG_IO_CLR_PIN         0x11
#define STMPE811_REG_IO_MP_STA          0x12
#define STMPE811_REG_IO_DIR             0x13
#define STMPE811_REG_IO_ED              0x14
#define STMPE811_REG_IO_RE              0x15
#define STMPE811_REG_IO_FE              0x16
#define STMPE811_REG_IO_AF              0x17

/* ADC Registers */
#define STMPE811_REG_ADC_INT_EN         0x0E
#define STMPE811_REG_ADC_INT_STA        0x0F
#define STMPE811_REG_ADC_CTRL1          0x20
#define STMPE811_REG_ADC_CTRL2          0x21
#define STMPE811_REG_ADC_CAPT           0x22
#define STMPE811_REG_ADC_DATA_CH0       0x30
#define STMPE811_REG_ADC_DATA_CH1       0x32
#define STMPE811_REG_ADC_DATA_CH2       0x34
#define STMPE811_REG_ADC_DATA_CH3       0x36
#define STMPE811_REG_ADC_DATA_CH4       0x38
#define STMPE811_REG_ADC_DATA_CH5       0x3A
#define STMPE811_REG_ADC_DATA_CH6       0x3B
#define STMPE811_REG_ADC_DATA_CH7       0x3C

/* Touch Screen Registers */
#define STMPE811_REG_TSC_CTRL           0x40
#define STMPE811_REG_TSC_CFG            0x41
#define STMPE811_REG_WDM_TR_X           0x42
#define STMPE811_REG_WDM_TR_Y           0x44
#define STMPE811_REG_WDM_BL_X           0x46
#define STMPE811_REG_WDM_BL_Y           0x48
#define STMPE811_REG_FIFO_TH            0x4A
#define STMPE811_REG_FIFO_STA           0x4B
#define STMPE811_REG_FIFO_SIZE          0x4C
#define STMPE811_REG_TSC_DATA_X         0x4D
#define STMPE811_REG_TSC_DATA_Y         0x4F
#define STMPE811_REG_TSC_DATA_Z         0x51
#define STMPE811_REG_TSC_DATA_XYZ       0x52
#define STMPE811_REG_TSC_FRACT_XYZ      0x56
#define STMPE811_REG_TSC_DATA_INC       0x57
#define STMPE811_REG_TSC_DATA_NON_INC   0xD7
#define STMPE811_REG_TSC_I_DRIVE        0x58
#define STMPE811_REG_TSC_SHIELD         0x59

/* IO Pins definition */
#define STMPE811_PIN_0                  0x01
#define STMPE811_PIN_1                  0x02
#define STMPE811_PIN_2                  0x04
#define STMPE811_PIN_3                  0x08
#define STMPE811_PIN_4                  0x10
#define STMPE811_PIN_5                  0x20
#define STMPE811_PIN_6                  0x40
#define STMPE811_PIN_7                  0x80
#define STMPE811_PIN_ALL                0xFF

/* Touch Screen Pins definition */
#define STMPE811_TOUCH_YD               STMPE811_PIN_1
#define STMPE811_TOUCH_XD               STMPE811_PIN_2
#define STMPE811_TOUCH_YU               STMPE811_PIN_3
#define STMPE811_TOUCH_XU               STMPE811_PIN_4
#define STMPE811_TOUCH_IO_ALL           (uint32_t)(STMPE811_PIN_1 | STMPE811_PIN_2 | STMPE811_PIN_3 | STMPE811_PIN_4)

/* IO Pins directions */
#define STMPE811_DIRECTION_IN           0x00
#define STMPE811_DIRECTION_OUT          0x01

/* IO IT types */
#define STMPE811_TYPE_LEVEL             0x00
#define STMPE811_TYPE_EDGE              0x02

/* IO IT polarity */
#define STMPE811_POLARITY_LOW           0x00
#define STMPE811_POLARITY_HIGH          0x04

/* IO Pin IT edge modes */
#define STMPE811_EDGE_FALLING           0x01
#define STMPE811_EDGE_RISING            0x02

/* TS registers masks */
#define STMPE811_TS_CTRL_ENABLE         0x01
#define STMPE811_TS_CTRL_STATUS         0x80

#define STMPE811_STACK_SIZE              300
#define STMPE811_I2C_ADDR               0x82

#define STMPE811_TIMEOUT                 500

static void stmpe811_task(void* arg);
static int stmpe811_init();
static int stmpe811_init_ts();
static I2c_error stmpe811_write_reg(uint8_t reg, uint8_t val);
static I2c_error stmpe811_read_reg(uint8_t reg, uint8_t* val);
static long stmpe811_isr();

static xSemaphoreHandle stmpe811_sem;

int stmpe811_module_init()
{
	// activation GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// pin d'IT du stmpe811
	gpio_pin_init(GPIOA, 15, GPIO_MODE_IN, GPIO_SPEED_25MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);

	// activation EXTI sur PA15, front descendant
	exti_register(EXTI_PA, 15, EXTI_TYPE_DOWN, stmpe811_isr);

	vSemaphoreCreateBinary(stmpe811_sem);
	xSemaphoreTake(stmpe811_sem, 0);

	xTaskCreate(stmpe811_task, "stmpe811", STMPE811_STACK_SIZE, NULL, PRIORITY_TASK_STMPE811, NULL);

	return 0;
}

module_init(stmpe811_module_init, INIT_STMPE811);

static void stmpe811_task(void* arg)
{
	(void) arg;

	stmpe811_init();

	while( 1 )
	{
		xSemaphoreTake(stmpe811_sem, portMAX_DELAY);
		uint8_t reg = STMPE811_REG_TSC_DATA_NON_INC;
		uint8_t data[4];

		i2c_transaction(STMPE811_I2C_ADDR, &reg, 1, data, sizeof(data), STMPE811_TIMEOUT);
		uint32_t tmp = (data[0] << 24)|(data[1] << 16)|(data[2] << 8)| data[3];
		uint16_t x = (tmp >> 20) & 0x00000FFF;
		uint16_t y = (tmp >> 8) & 0x00000FFF;
		// reset fifo
		stmpe811_write_reg(STMPE811_REG_FIFO_STA, 0x01);
		stmpe811_write_reg(STMPE811_REG_FIFO_STA, 0x00);

		// clear des flags d'IT
		// TODO lire les flags avant pour voir si on detecte un appui
		stmpe811_write_reg(STMPE811_REG_INT_STA, 0xff);

		log_format(LOG_INFO, "ts : %d %d", x, y);
	}
}

static I2c_error stmpe811_write_reg(uint8_t reg, uint8_t val)
{
	char tx_buffer[2];
	tx_buffer[0] = reg;
	tx_buffer[1] = val;
	I2c_error res = i2c_write_data(STMPE811_I2C_ADDR, tx_buffer, 2, STMPE811_TIMEOUT);
	if( res )
	{
		log_format(LOG_ERROR, "i2c_write_data(%x, %x) failed : %x", reg, val, res);
	}

	return res;
}

static I2c_error stmpe811_read_reg(uint8_t reg, uint8_t* val)
{
	return i2c_transaction(STMPE811_I2C_ADDR, &reg, 1, &val, 1, STMPE811_TIMEOUT);
}

static int stmpe811_init()
{
	char tx_buffer[2];
	char rx_buffer[2];

	tx_buffer[0] = STMPE811_REG_CHP_ID_LSB;
	I2c_error res = i2c_transaction(STMPE811_I2C_ADDR, tx_buffer, 1, rx_buffer, 2, STMPE811_TIMEOUT);

	if( res )
	{
		log_format(LOG_ERROR, "stmpe811 not found : %d", res);
		return -1;
	}

	uint16_t id = (((uint16_t)rx_buffer[0]) << 8) + rx_buffer[1];

	if( id != STMPE811_ID )
	{
		log_format(LOG_ERROR, "stmpe811 not found, wrong id : %x", id );
		return -1;
	}

	log(LOG_INFO, "stmpe811 found" );

	// reset
	tx_buffer[0] = STMPE811_REG_SYS_CTRL1;
	tx_buffer[1] = 2;
	i2c_write_data(STMPE811_I2C_ADDR, tx_buffer, 2, STMPE811_TIMEOUT);

	// attente reset
	vTaskDelay(10);

	// power on
	tx_buffer[0] = STMPE811_REG_SYS_CTRL1;
	tx_buffer[1] = 0;
	i2c_write_data(STMPE811_I2C_ADDR, tx_buffer, 2, STMPE811_TIMEOUT);

	// attente reinitialisation
	vTaskDelay(2);

	return stmpe811_init_ts();
}

static int stmpe811_init_ts()
{
	uint8_t val;

	// sequence d'initialisation du TS
	stmpe811_read_reg(STMPE811_REG_SYS_CTRL2, &val);
	val &= ~(STMPE811_TS_FCT | STMPE811_ADC_FCT);
	stmpe811_write_reg(STMPE811_REG_SYS_CTRL2, val);

	stmpe811_write_reg(STMPE811_REG_SYS_CTRL1, 0x49);
	vTaskDelay(2);


	// activation IT TS
	stmpe811_read_reg(STMPE811_REG_INT_EN, &val);
	val |= STMPE811_TS_IT;
	stmpe811_write_reg(STMPE811_REG_INT_EN, val);

	// selection ADC_CLK: 3.25 MHz
	stmpe811_write_reg(STMPE811_REG_ADC_CTRL2, 0x01);

	// selection pin TSC
	stmpe811_read_reg(STMPE811_REG_IO_AF, &val);
	val &= ~STMPE811_TOUCH_IO_ALL;
	stmpe811_write_reg(STMPE811_REG_IO_AF, val);

	// selection filtre 2 nF
	// conf
    // - Touch average control    : 4 samples
    // - Touch delay time         : 500 uS
    // - Panel driver setting time: 500 uS
	stmpe811_write_reg(STMPE811_REG_TSC_CFG, 0x9A);

	// conf FIFO
	stmpe811_write_reg(STMPE811_REG_FIFO_TH, 0x01);

	// on efface le contenu de la fifo et on la reactive
	stmpe811_write_reg(STMPE811_REG_FIFO_STA, 0x01);
	stmpe811_write_reg(STMPE811_REG_FIFO_STA, 0x00);

	// Set the range and accuracy pf the pressure measurement (Z)
	// - Fractional part :7
	// - Whole part      :1
	stmpe811_write_reg(STMPE811_REG_TSC_FRACT_XYZ, 0x07);

	// Set the driving capability (limit) of the device for TSC pins: 50mA
	stmpe811_write_reg(STMPE811_REG_TSC_I_DRIVE, 0x01);

	// Touch screen control configuration (enable TSC):
	// - No window tracking index
	// - XYZ acquisition mode
	stmpe811_write_reg(STMPE811_REG_TSC_CTRL, 0x01);

	// Clear all the status pending bits if any
	stmpe811_write_reg(STMPE811_REG_INT_STA, 0xff);

	// activation IT generale
	stmpe811_read_reg(STMPE811_REG_INT_CTRL, &val);
	val |= STMPE811_GIT_EN;
	stmpe811_write_reg(STMPE811_REG_INT_CTRL, val);

	vTaskDelay(2);

	return 0;
}

long stmpe811_isr(void)
{
	long xHigherPriorityTaskWoken = 0;
	xSemaphoreGiveFromISR(stmpe811_sem, &xHigherPriorityTaskWoken);
	return xHigherPriorityTaskWoken;
}
