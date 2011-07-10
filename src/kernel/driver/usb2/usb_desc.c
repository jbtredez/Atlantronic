#include "usb_lib.h"
#include "usb_desc.h"

// USB Standard Device Descriptor
const uint8_t usb_device_descriptor[] =
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
	0x01,
	0x00,   // idProduct = 0x0001
	0x00,
	0x01,   // bcdDevice = 1.00
	1,      // Index of string descriptor describing manufacturer
	2,      // Index of string descriptor describing product
	3,      // Index of string descriptor describing the device's serial number
	0x01    // bNumConfigurations
};

// Configuration Descriptor
const uint8_t usb_config_descriptor[] =
{
	0x09,                                // bLength: Configuration Descriptor size
	USB_CONFIGURATION_DESCRIPTOR_TYPE,   // bDescriptorType: Configuration
	USB_CONFIG_DESCRIPTOR_SIZE,          // wTotalLength : no of returned bytes
	0x00,
	0x02,                                // bNumInterfaces: 2 interfaces
	0x01,                                // bConfigurationValue: Configuration value
	0x00,                                // iConfiguration: Index of string descriptor describing the configuration (pas de description)
	0xC0,                                // bmAttributes: 0xC0 = auto-alimenté
	0x32,                                // MaxPower : 0x32 * 2mA = 100 mA
// Interface Descriptor 0
	0x09,                           // bLength: Interface Descriptor size
	USB_INTERFACE_DESCRIPTOR_TYPE,  // bDescriptorType: Interface
	0x00,                           // bInterfaceNumber: Number of Interface (interface 0)
	0x00,                           // bAlternateSetting: Alternate setting
	0x01,                           // bNumEndpoints: 1 terminaison utilisée
	0xff,                           // bInterfaceClass: specifique constructeur (driver perso)
	USB_LOG_SUBCLASS,               // bInterfaceSubClass
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
// Interface descriptor 1
	0x09,                           // bLength: Endpoint Descriptor size
	USB_INTERFACE_DESCRIPTOR_TYPE,  // bDescriptorType: Interface
	0x01,                           // bInterfaceNumber: Number of Interface (interface 1)
	0x00,                           // bAlternateSetting: Alternate setting
	0x01,                           // bNumEndpoints: 1 terminaison utilisée
	0xff,                           // bInterfaceClass: specifique constructeur (driver perso)
	USB_HOKUYO_SUBCLASS,            // bInterfaceSubClass
	0x00,                           // bInterfaceProtocol
	0x00,                           // iInterface: 0 => pas de description
// Endpoint Descriptor (IN2)
	0x07,                           // bLength: Endpoint Descriptor size
	USB_ENDPOINT_DESCRIPTOR_TYPE,   // bDescriptorType: Endpoint
	0x82,                           // bEndpointAddress: IN2
	0x02,                           // bmAttributes: terminaison de type bloc
	0x40,                           // wMaxPacketSize: 64 octets max
	0x00,
	0x00,                           // bInterval: ignorée pour les terminaisons de type bloc
};

/* USB String Descriptors */
const uint8_t usb_string_langID[USB_STRING_LANG_ID_SIZE] =
{
	USB_STRING_LANG_ID_SIZE,
	USB_STRING_DESCRIPTOR_TYPE,
	0x0c,
	0x04 // LangID = 0x040c: Fr
};

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
	'B', 0, 'a', 0, 'r', 0
};

uint8_t usb_string_serial[USB_STRING_SERIAL_SIZE] =
{
	USB_STRING_SERIAL_SIZE,            // bLength
	USB_STRING_DESCRIPTOR_TYPE,        // bDescriptorType
	'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, '1', 0, '0', 0
};
