#ifndef USB_DESCRIPTOR_H
#define USB_DESCRIPTOR_H

#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

#define USB_DATA_SUBCLASS                       0x00

#define USB_DEVICE_DESCRIPTOR_SIZE              0x12
#define USB_CONFIG_DESCRIPTOR_SIZE                32
#define USB_STRING_LANG_ID_SIZE                    4

extern const uint8_t usb_device_descriptor[USB_DEVICE_DESCRIPTOR_SIZE];
extern const uint8_t usb_config_descriptor[USB_CONFIG_DESCRIPTOR_SIZE];
extern const uint8_t usb_string_langID[USB_STRING_LANG_ID_SIZE];

#endif
