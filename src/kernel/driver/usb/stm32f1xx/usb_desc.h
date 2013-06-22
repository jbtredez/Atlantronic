#ifndef USB_DESC_H
#define USB_DESC_H

//! @file usb_desc.h
//! @brief USB descriptors
//! @author Atlantronic
#include "kernel/driver/usb/usb_descriptor.h"

#define USB_STRING_VENDOR_SIZE                    24
#define USB_STRING_PRODUCT_SIZE                    8
#define USB_STRING_SERIAL_SIZE                    50

extern const uint8_t usb_string_vendor[USB_STRING_VENDOR_SIZE];
extern const uint8_t usb_string_product[USB_STRING_PRODUCT_SIZE];
extern uint8_t usb_string_serial[USB_STRING_SERIAL_SIZE];

#endif
