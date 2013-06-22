//! @file usb_desc.c
//! @brief USB descriptors
//! @author Atlantronic
//!
//! 1 configuration
//!   3 interfaces
//!     - interface log
//!     - interface hokuyo
//!     - interface data

#include <stdint.h>
#include "usb_desc.h"

const uint8_t usb_string_vendor[USB_STRING_VENDOR_SIZE] =
{
	USB_STRING_VENDOR_SIZE,       // Size of Vendor string
	USB_STRING_DESCRIPTOR_TYPE,   // bDescriptorType
	'A', 0, 't', 0, 'l', 0, 'a', 0, 'n', 0, 't', 0, 'r', 0, 'o', 0,
	'n', 0, 'i', 0, 'c', 0
};

const uint8_t usb_string_product[USB_STRING_PRODUCT_SIZE] =
{
	USB_STRING_PRODUCT_SIZE,           // bLength
	USB_STRING_DESCRIPTOR_TYPE,        // bDescriptorType
#if defined( __foo__ )
	'F', 0, 'o', 0, 'o', 0
#elif defined( __bar__ )
	'B', 0, 'a', 0, 'r', 0
#else
#error unknown card
#endif
};

uint8_t usb_string_serial[USB_STRING_SERIAL_SIZE] =
{
	USB_STRING_SERIAL_SIZE,            // bLength
	USB_STRING_DESCRIPTOR_TYPE,        // bDescriptorType
	'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, '1', 0, '0', 0
};
