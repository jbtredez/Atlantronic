#ifndef __USB_DESC_H
#define __USB_DESC_H

#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

#define USB_LOG_SUBCLASS                        0x00
#define USB_HOKUYO_SUBCLASS                     0x01

#define USB_DEVICE_DESCRIPTOR_SIZE              0x12
#define USB_CONFIG_DESCRIPTOR_SIZE              41


#define VIRTUAL_COM_PORT_SIZ_STRING_LANGID      4
#define STRING_VENDOR_SIZE      24
#define STRING_PRODUCT_SIZE     8
#define VIRTUAL_COM_PORT_SIZ_STRING_SERIAL      26

#define STANDARD_ENDPOINT_DESC_SIZE             0x09

extern const uint8_t usb_device_descriptor[USB_DEVICE_DESCRIPTOR_SIZE];
extern const uint8_t usb_config_descriptor[USB_CONFIG_DESCRIPTOR_SIZE];

extern const uint8_t Virtual_Com_Port_StringLangID[VIRTUAL_COM_PORT_SIZ_STRING_LANGID];
extern const uint8_t Virtual_Com_Port_StringVendor[STRING_VENDOR_SIZE];
extern const uint8_t Virtual_Com_Port_StringProduct[STRING_PRODUCT_SIZE];
extern uint8_t Virtual_Com_Port_StringSerial[VIRTUAL_COM_PORT_SIZ_STRING_SERIAL];

#endif /* __USB_DESC_H */

