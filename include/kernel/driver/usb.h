#ifndef USB_H
#define USB_H

//! @file usb.h
//! @brief USB
//! @author Atlantronic

#define USB_LOG                  1
#define USB_HOKUYO               2

void usb_add(uint16_t type, unsigned char* msg, uint16_t size);

#endif