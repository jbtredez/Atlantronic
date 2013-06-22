#include <stdint.h>
#include "usb_descriptor.h"

// USB Standard Device Descriptor
const uint8_t usb_device_descriptor[USB_DEVICE_DESCRIPTOR_SIZE] __attribute__ ((aligned (4))) =
{
	USB_DEVICE_DESCRIPTOR_SIZE,   // bLength
	USB_DEVICE_DESCRIPTOR_TYPE,   // bDescriptorType
	0x00,
	0x02,   // bcdUSB = 2.00 (USB 2.0)
	0xff,   // bDeviceClass : 0xff = specifique constructeur (driver perso)
	0x00,   // bDeviceSubClass
	0x00,   // bDeviceProtocol
	0x40,   // bMaxPacketSize0
	0x18,
	0x18,   // idVendor = 0x1818
#if defined( __foo__ )
	0x01,
	0x00,   // idProduct = 0x0001
#elif defined( __bar__ )
	0x02,
	0x00,   // idProduct = 0x0002
#elif defined(__discovery__)
	0x03,                       // idProduct = 0x0003
	0x00,
#else
#error unknown card
#endif
	0x00,
	0x01,   // bcdDevice = 1.00
	1,      // Index of string descriptor describing manufacturer
	2,      // Index of string descriptor describing product
	3,      // Index of string descriptor describing the device's serial number
	0x01    // bNumConfigurations
};

// Configuration Descriptor
const uint8_t usb_config_descriptor[USB_CONFIG_DESCRIPTOR_SIZE] __attribute__ ((aligned (4))) =
{
	0x09,                                // bLength: Configuration Descriptor size
	USB_CONFIGURATION_DESCRIPTOR_TYPE,   // bDescriptorType: Configuration
	USB_CONFIG_DESCRIPTOR_SIZE,          // wTotalLength : no of returned bytes
	0x00,
	0x01,                                // bNumInterfaces: 1 interfaces
	0x01,                                // bConfigurationValue: Configuration value
	0x00,                                // iConfiguration: Index of string descriptor describing the configuration (pas de description)
	0xC0,                                // bmAttributes: 0xC0 = auto-alimenté
	0x32,                                // MaxPower : 0x32 * 2mA = 100 mA
// Interface Descriptor 0
	0x09,                           // bLength: Interface Descriptor size
	USB_INTERFACE_DESCRIPTOR_TYPE,  // bDescriptorType: Interface
	0x00,                           // bInterfaceNumber: Number of Interface (interface 0)
	0x00,                           // bAlternateSetting: Alternate setting
	0x02,                           // bNumEndpoints: 2 terminaisons utilisées
	0xff,                           // bInterfaceClass: specifique constructeur (driver perso)
	USB_DATA_SUBCLASS,              // bInterfaceSubClass
	0x00,                           // bInterfaceProtocol
	0x00,                           // iInterface: 0 => pas de description
// Endpoint Descriptor (IN1)
	0x07,                           // bLength: Endpoint Descriptor size
	USB_ENDPOINT_DESCRIPTOR_TYPE,   // bDescriptorType: Endpoint
	0x81,                           // bEndpointAddress: IN1
	0x02,                           // bmAttributes: terminaison de type bloc
	0x40,                           // wMaxPacketSize: 64 octets max
	0x00,
	0x00,                           // bInterval: ignorée pour les terminaisons de type bloc
// Endpoint Descriptor (OUT2)
	0x07,                           // bLength: Endpoint Descriptor size
	USB_ENDPOINT_DESCRIPTOR_TYPE,   // bDescriptorType: Endpoint
	0x02,                           // bEndpointAddress: OUT2
	0x02,                           // bmAttributes: terminaison de type bloc
	0x40,                           // wMaxPacketSize: 64 octets max
	0x00,
	0x00,                           // bInterval: ignorée pour les terminaisons de type bloc
};

// USB String Descriptors
const uint8_t usb_string_langID[USB_STRING_LANG_ID_SIZE] __attribute__ ((aligned (4))) =
{
	USB_STRING_LANG_ID_SIZE,
	USB_STRING_DESCRIPTOR_TYPE,
	0x0c,
	0x04 // LangID = 0x040c: Fr
};
