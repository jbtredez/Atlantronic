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