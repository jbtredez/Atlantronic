#include "kernel/module.h"
#include "usb_core.h"
#include "usb_init.h"
#include "usb_pwr.h"
#include "usb_desc.h"
#include "usb_istr.h"

__IO uint8_t PrevXferComplete = 1;

static int usb_module_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->AHBENR |= RCC_AHBENR_OTGFSEN; // USB OTG FS clock enable

	// TODO priorite
	NVIC_EnableIRQ(OTG_FS_IRQn);

	USB_Init();

	return 0;
}

module_init(usb_module_init, INIT_USB);

void Enter_LowPowerMode(void)
{
	// Set the device state to suspend
	bDeviceState = SUSPENDED;
}

void Leave_LowPowerMode(void)
{
	DEVICE_INFO *pInfo = &Device_Info;

	// Set the device state to the correct state
	if (pInfo->Current_Configuration != 0)
	{
		// Device configured
		bDeviceState = CONFIGURED;
	}
	else 
	{
		bDeviceState = ATTACHED;
	}
}

//! Convert Hex 32Bits value into char
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
	uint8_t idx = 0;

	for( idx = 0 ; idx < len ; idx ++)
	{
		if( ((value >> 28)) < 0xA )
		{
			pbuf[ 2* idx] = (value >> 28) + '0';
		}
		else
		{
			pbuf[2* idx] = (value >> 28) + 'A' - 10; 
		}

		value = value << 4;

		pbuf[ 2* idx + 1] = 0;
	}
}

// Create the serial number string descriptor.
void Get_SerialNum(void)
{
	uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

	Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
	Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
	Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

	Device_Serial0 += Device_Serial2;

	if (Device_Serial0 != 0)
	{
		IntToUnicode (Device_Serial0, &usb_string_serial[2] , 8);
		IntToUnicode (Device_Serial1, &usb_string_serial[18], 4);
	}
}

//!< Petite attente active
void USB_OTG_BSP_uDelay ( uint32_t usec)
{
	usec *= 36;
	for( ; usec-- ; )
	{
		nop();
	}
}

void isr_otg_fs(void)
{
	STM32_PCD_OTG_ISR_Handler(); 
}