#ifndef __USB_BSP__H__
#define __USB_BSP__H__

#include "usb_core.h"
#include "kernel/rcc.h"

#define USB_OTG_BSP_uDelay(usec)  wait_active(us_to_systick(usec))
#define USB_OTG_BSP_mDelay(usec)  wait_active(ms_to_systick(usec))

//void USB_OTG_BSP_uDelay (const uint32_t usec);
//void USB_OTG_BSP_mDelay (const uint32_t msec);

#ifdef USE_HOST_MODE
void USB_OTG_BSP_ConfigVBUS(USB_OTG_CORE_HANDLE *pdev);
void USB_OTG_BSP_DriveVBUS(USB_OTG_CORE_HANDLE *pdev,uint8_t state);
#endif

#endif //__USB_BSP__H__
