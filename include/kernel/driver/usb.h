#ifndef USB_H
#define USB_H

//! @file usb.h
//! @brief USB
//! @author Atlantronic

enum
{
	USB_LOG = 1,
	USB_HOKUYO,
	USB_CONTROL
};

enum
{
	USB_CMD_GOTO_NEAR,
	USB_CMD_NUM       //!< nombre de commandes, laisser en dernier
};

void usb_add(uint16_t type, void* msg, uint16_t size);

void usb_add_cmd(int id, void (*cmd)(void*));

#endif