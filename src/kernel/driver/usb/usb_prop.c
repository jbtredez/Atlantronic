//! @file usb_prop.c
//! @brief USB property
//! @author Atlantronic

#include "kernel/driver/usb/usb_lib.h"
#include "kernel/driver/usb/usb_conf.h"
#include "kernel/driver/usb/usb_desc.h"
#include "kernel/driver/usb/usb_pwr.h"
#include "kernel/serial_number.h"
#include "kernel/utf8.h"

static void usb_prop_init(void);
static uint8_t *usb_prop_get_string_descriptor(uint16_t);
static void usb_status_in(void);
static void usb_status_out(void);
static void usb_prop_reset(void);
static RESULT usb_prop_data_setup(uint8_t RequestNo);
static void usb_prop_set_device_address(void);
static RESULT usb_prop_no_data_setup(uint8_t RequestNo);
static void usb_prop_set_configuration(void);
static RESULT usb_prop_get_interface_setting(uint8_t Interface, uint8_t AlternateSetting);
static uint8_t *usb_prop_get_device_descriptor(uint16_t Length);
static uint8_t *usb_prop_get_config_descriptor(uint16_t Length);

DEVICE Device_Table =
{
	EP_NUM,
	1
};

DEVICE_PROP Device_Property =
{
	usb_prop_init,
	usb_prop_reset,
	usb_status_in,
	usb_status_out,
	usb_prop_data_setup,
	usb_prop_no_data_setup,
	usb_prop_get_interface_setting,
	usb_prop_get_device_descriptor,
	usb_prop_get_config_descriptor,
	usb_prop_get_string_descriptor,
	0,
	0x40 // MAX PACKET SIZE
};

USER_STANDARD_REQUESTS User_Standard_Requests =
{
	NOP_Process,
	usb_prop_set_configuration,
	NOP_Process,
	NOP_Process,
	NOP_Process,
	NOP_Process,
	NOP_Process,
	NOP_Process,
	usb_prop_set_device_address
};

ONE_DESCRIPTOR Device_Descriptor =
{
	(uint8_t*)usb_device_descriptor,
	USB_DEVICE_DESCRIPTOR_SIZE
};

ONE_DESCRIPTOR Config_Descriptor =
{
	(uint8_t*)usb_config_descriptor,
	USB_CONFIG_DESCRIPTOR_SIZE
};

ONE_DESCRIPTOR String_Descriptor[4] =
{
	{(uint8_t*)usb_string_langID, USB_STRING_LANG_ID_SIZE},
	{(uint8_t*)usb_string_vendor, USB_STRING_VENDOR_SIZE},
	{(uint8_t*)usb_string_product, USB_STRING_PRODUCT_SIZE},
	{(uint8_t*)usb_string_serial, USB_STRING_SERIAL_SIZE}
};

static void usb_prop_init(void)
{
	uint_to_hex_utf8(SERIAL_NUMBER_2, &usb_string_serial[2] , 8);
	uint_to_hex_utf8(SERIAL_NUMBER_1, &usb_string_serial[18], 8);
	uint_to_hex_utf8(SERIAL_NUMBER_0, &usb_string_serial[34], 8);

	pInformation->Current_Configuration = 0;

	// Connect the device
	PowerOn();

	// Perform basic device initialization operations
	USB_SIL_Init();

	bDeviceState = UNCONNECTED;
}

static void usb_prop_reset(void)
{
	// Set device as not configured
	pInformation->Current_Configuration = 0;

	// Current Feature initialization
	pInformation->Current_Feature = usb_config_descriptor[7];

	// Set the default Interface
	pInformation->Current_Interface = 0;

	/* EP0 is already configured by USB_SIL_Init() function */

	// Init EP1 IN as Interrupt endpoint
	OTG_DEV_EP_Init(EP1_IN, OTG_DEV_EP_TYPE_INT, 0x40);

	// Init EP1 IN as Bulk endpoint
	OTG_DEV_EP_Init(EP2_IN, OTG_DEV_EP_TYPE_BULK, 0x40);

	bDeviceState = ATTACHED;
}

//! Update the device state to configured.
static void usb_prop_set_configuration(void)
{
	DEVICE_INFO *pInfo = &Device_Info;

	if (pInfo->Current_Configuration != 0)
	{
		// Device configured
		bDeviceState = CONFIGURED;
	}
}

//! Update the device state to addressed.
static void usb_prop_set_device_address(void)
{
	bDeviceState = ADDRESSED;
}

static void usb_status_in(void)
{

}

static void usb_status_out(void)
{

}

//! Handle the data class specific requests
//! @return USB_UNSUPPORT or USB_SUCCESS
static RESULT usb_prop_data_setup(uint8_t RequestNo)
{
	(void) RequestNo;
	return USB_UNSUPPORT;
}

//! Handle the no data class specific requests
//! @return USB_UNSUPPORT or USB_SUCCESS
static RESULT usb_prop_no_data_setup(uint8_t RequestNo)
{
	(void) RequestNo;
	return USB_UNSUPPORT;
}

//! Gets the device descriptor.
//! @return address of the device descriptor.
static uint8_t *usb_prop_get_device_descriptor(uint16_t Length)
{
	return Standard_GetDescriptorData(Length, &Device_Descriptor);
}

//! Get the configuration descriptor.
//! @return address of the configuration descriptor.
static uint8_t *usb_prop_get_config_descriptor(uint16_t Length)
{
	return Standard_GetDescriptorData(Length, &Config_Descriptor);
}

//!< Gets the string descriptors according to the needed index
static uint8_t *usb_prop_get_string_descriptor(uint16_t Length)
{
	uint8_t wValue0 = pInformation->USBwValue0;
	if (wValue0 > 4)
	{
		return NULL;
	}
	else
	{
		return Standard_GetDescriptorData(Length, &String_Descriptor[wValue0]);
	}
}

//! test the interface and the alternate setting according to the supported one.
//! @Interface : interface number.
//! @AlternateSetting : Alternate Setting number.
//! @return The address of the string descriptors.
static RESULT usb_prop_get_interface_setting(uint8_t Interface, uint8_t AlternateSetting)
{
	if (AlternateSetting > 0)
	{
		return USB_UNSUPPORT;
	}
	else if (Interface > 1)
	{
		return USB_UNSUPPORT;
	}
	return USB_SUCCESS;
}
