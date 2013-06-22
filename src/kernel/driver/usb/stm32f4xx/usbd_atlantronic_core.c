#include "usbd_atlantronic_core.h"
#include "usbd_desc.h"
#include "usbd_req.h"

static uint8_t usbd_atlantronic_init(void* pdev , uint8_t cfgidx);
static uint8_t usbd_atlantronic_deinit(void* pdev , uint8_t cfgidx);
static uint8_t usbd_atlantronic_setup(void* pdev , USB_SETUP_REQ* req);
static uint8_t usbd_atlantronic_data_in(void* pdev , uint8_t epnum);
static uint8_t usbd_atlantronic_data_out(void* pdev , uint8_t epnum);
static uint8_t* usbd_atlantronic_get_config(uint8_t speed, uint16_t *length);
void EP1_IN_Callback(void) __attribute__((weak, alias("nop_function") ));
void EP2_OUT_Callback(void) __attribute__((weak, alias("nop_function") ));

USBD_Class_cb_TypeDef USBD_atlantronic_cb =
{
	usbd_atlantronic_init,
	usbd_atlantronic_deinit,
	usbd_atlantronic_setup,
	0,
	0,
	usbd_atlantronic_data_in,
	usbd_atlantronic_data_out,
	0,
	0,
	0,
	usbd_atlantronic_get_config,
};

static uint8_t usbd_atlantronic_init(void* pdev , uint8_t cfgidx)
{
	(void) cfgidx;
	DCD_EP_Open(pdev, 0x81, 0x40, USB_OTG_EP_BULK);
	DCD_EP_Open(pdev, 0x02, 0x40, USB_OTG_EP_BULK);

	return USBD_OK;
}

static uint8_t usbd_atlantronic_deinit(void* pdev , uint8_t cfgidx)
{
	(void) cfgidx;
	DCD_EP_Close(pdev, 0x81);
	DCD_EP_Close(pdev, 0x02);

	return USBD_OK;
}

static uint8_t usbd_atlantronic_setup(void* pdev , USB_SETUP_REQ* req)
{
	(void) pdev;
	(void) req;
	return USBD_OK;
}

static uint8_t usbd_atlantronic_data_in(void* pdev , uint8_t epnum)
{
	(void) pdev;
	if( epnum == 0x01)
	{
		EP1_IN_Callback();
	}
	return USBD_OK;
}

static uint8_t usbd_atlantronic_data_out(void* pdev , uint8_t epnum)
{
	(void) pdev;
	if( epnum == 0x02)
	{
		EP2_OUT_Callback();
	}	return USBD_OK;
}

static uint8_t* usbd_atlantronic_get_config(uint8_t speed, uint16_t *length)
{
	(void) speed;
	*length = sizeof(usb_config_descriptor);
	return (uint8_t*) usb_config_descriptor;
}
