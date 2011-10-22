#ifndef USB_DESC_H
#define USB_DESC_H

//! @file usb_desc.h
//! @brief USB descriptors
//! @author Atlantronic

#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

#define USB_DATA_SUBCLASS                       0x00

#define USB_DEVICE_DESCRIPTOR_SIZE              0x12
#define USB_CONFIG_DESCRIPTOR_SIZE                25
#define USB_STRING_LANG_ID_SIZE                    4
#define USB_STRING_VENDOR_SIZE                    24
#define USB_STRING_PRODUCT_SIZE                    8
#define USB_STRING_SERIAL_SIZE                    50

#define STANDARD_ENDPOINT_DESC_SIZE             0x09

extern const uint8_t usb_device_descriptor[USB_DEVICE_DESCRIPTOR_SIZE];
extern const uint8_t usb_config_descriptor[USB_CONFIG_DESCRIPTOR_SIZE];
extern const uint8_t usb_string_langID[USB_STRING_LANG_ID_SIZE];
extern const uint8_t usb_string_vendor[USB_STRING_VENDOR_SIZE];
extern const uint8_t usb_string_product[USB_STRING_PRODUCT_SIZE];
extern uint8_t usb_string_serial[USB_STRING_SERIAL_SIZE];

#endif
