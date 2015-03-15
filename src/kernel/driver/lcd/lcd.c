#include "lcd.h"
#include "kernel/driver/spi.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/log.h"
#include "priority.h"
#include "kernel/driver/gpio.h"
#include "kernel/driver/sdram.h"

#define LCD_STACK_SIZE        300

static void lcd_task(void* arg);

static int lcd_module_init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

#if defined(__disco__)
	gpio_pin_init(GPIOD, 13, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DCX
#else
#error unknown card
#endif

	xTaskCreate(lcd_task, "lcd", LCD_STACK_SIZE, NULL, PRIORITY_TASK_LCD, NULL);

	return 0;
}

module_init(lcd_module_init, INIT_LCD);

static void lcd_write_reg(uint8_t reg)
{
	gpio_reset_pin(GPIOD, 13);
	spi_write(SPI_DEVICE_LCD, &reg, 1);
}

static void lcd_write8_data(uint8_t data)
{
	gpio_set_pin(GPIOD, 13);
	spi_write(SPI_DEVICE_LCD, &data, 1);
}

static void lcd_init()
{
	log(LOG_INFO, "initialisation lcd");
	lcd_write_reg(0xCA);
	lcd_write8_data(0xC3);
	lcd_write8_data(0x08);
	lcd_write8_data(0x50);

	lcd_write_reg(LCD_POWERB);
	lcd_write8_data(0x00);
	lcd_write8_data(0xC1);
	lcd_write8_data(0x30);

	lcd_write_reg(LCD_POWER_SEQ);
	lcd_write8_data(0x64);
	lcd_write8_data(0x03);
	lcd_write8_data(0x12);
	lcd_write8_data(0x81);

	lcd_write_reg(LCD_DTCA);
	lcd_write8_data(0x85);
	lcd_write8_data(0x00);
	lcd_write8_data(0x78);

	lcd_write_reg(LCD_POWERA);
	lcd_write8_data(0x39);
	lcd_write8_data(0x2C);
	lcd_write8_data(0x00);
	lcd_write8_data(0x34);
	lcd_write8_data(0x02);

	lcd_write_reg(LCD_PRC);
	lcd_write8_data(0x20);

	lcd_write_reg(LCD_DTCB);
	lcd_write8_data(0x00);
	lcd_write8_data(0x00);

	lcd_write_reg(LCD_FRMCTR1);
	lcd_write8_data(0x00);
	lcd_write8_data(0x1B);

	lcd_write_reg(LCD_DFC);
	lcd_write8_data(0x0A);
	lcd_write8_data(0xA2);

	lcd_write_reg(LCD_POWER1);
	lcd_write8_data(0x10);          // GVDD = 3.65V
	lcd_write_reg(LCD_POWER2);
	lcd_write8_data(0x10);         // DDVDH = 2 VCI ; VGH = 7 CVI ; VGL = -4 VCI
	lcd_write_reg(LCD_VCOM1);
	lcd_write8_data(0x45);        // VCOMH = 4.425 V
	lcd_write8_data(0x15);        // VCOML = -1.975 V
	lcd_write_reg(LCD_VCOM2);
	lcd_write8_data(0x90);

	lcd_write_reg(LCD_PIXEL_FORMAT);
	lcd_write8_data(0x55); // 16 bit
	lcd_write_reg(LCD_MAC);
	lcd_write8_data(0x08);
	lcd_write_reg(LCD_3GAMMA_EN);
	lcd_write8_data(0x00);

#if 0
	lcd_write_reg(LCD_RGB_INTERFACE);
	lcd_write8_data(0xC2);
#endif
	lcd_write_reg(LCD_DFC);
	lcd_write8_data(0x0A);
	lcd_write8_data(0xA7);
	lcd_write8_data(0x27);
	lcd_write8_data(0x04);

	lcd_write_reg(LCD_COLUMN_ADDR);
	lcd_write8_data(0x00);
	lcd_write8_data(0x00);
	lcd_write8_data(0x00);
	lcd_write8_data(0xEF);
	lcd_write_reg(LCD_PAGE_ADDR);
	lcd_write8_data(0x00);
	lcd_write8_data(0x00);
	lcd_write8_data(0x01);
	lcd_write8_data(0x3F);

/*	lcd_write_reg(LCD_INTERFACE);
	lcd_write8_data(0x01);
	lcd_write8_data(0x00); // TODO voir MDT
//	lcd_write8_data(0x06); // TODO RM=0
	lcd_write8_data(0x00); // TODO RM=0
*/
	lcd_write_reg(LCD_GRAM);
	vTaskDelay(200);

	lcd_write_reg(LCD_GAMMA);
	lcd_write8_data(0x01);

	lcd_write_reg(LCD_PGAMMA);
	lcd_write8_data(0x0F);
	lcd_write8_data(0x29);
	lcd_write8_data(0x24);
	lcd_write8_data(0x0C);
	lcd_write8_data(0x0E);
	lcd_write8_data(0x09);
	lcd_write8_data(0x4E);
	lcd_write8_data(0x78);
	lcd_write8_data(0x3C);
	lcd_write8_data(0x09);
	lcd_write8_data(0x13);
	lcd_write8_data(0x05);
	lcd_write8_data(0x17);
	lcd_write8_data(0x11);
	lcd_write8_data(0x00);
	lcd_write_reg(LCD_NGAMMA);
	lcd_write8_data(0x00);
	lcd_write8_data(0x16);
	lcd_write8_data(0x1B);
	lcd_write8_data(0x04);
	lcd_write8_data(0x11);
	lcd_write8_data(0x07);
	lcd_write8_data(0x31);
	lcd_write8_data(0x33);
	lcd_write8_data(0x42);
	lcd_write8_data(0x05);
	lcd_write8_data(0x0C);
	lcd_write8_data(0x0A);
	lcd_write8_data(0x28);
	lcd_write8_data(0x2F);
	lcd_write8_data(0x0F);

	lcd_write_reg(LCD_SLEEP_OUT);
	vTaskDelay(200);
	lcd_write_reg(LCD_DISPLAY_ON);

	lcd_write_reg(LCD_GRAM);
	log(LOG_INFO, "lcd initialise");
}

static void lcd_task(void* arg)
{
	(void) arg;

	vTaskDelay(100);

	lcd_init();

	int i;
	uint16_t* lcd_buffer = (uint16_t*)SDRAM_DEVICE_ADDR;
	for(i = 0; i < 320*240; i++)
	{
		lcd_buffer[i] = 0xf800;
		if(i > 320*120)
		{
			lcd_buffer[i] = 0x00f8;
		}
	}
	lcd_write_reg(LCD_GRAM);

	for(i = 0; i < 240; i++)
	{
		gpio_set_pin(GPIOD, 13);
		spi_write(SPI_DEVICE_LCD, &lcd_buffer[320*i], 320*2);
	}

	while(1)
	{
		vTaskDelay(1000);
	}
}
