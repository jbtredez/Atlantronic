#ifndef USB_H
#define USB_H

//! @file usb.h
//! @brief USB
//! @author Atlantronic

enum
{
	USB_LOG = 1,
	USB_ERR,
	USB_HOKUYO_FOO,
	USB_HOKUYO_BAR,
	USB_HOKUYO_FOO_SEG,
	USB_CONTROL
};

enum usb_cmd
{
	USB_CMD_TRAJECTORY = 0,
	USB_CMD_CONTROL_PARAM,
	USB_CMD_CONTROL_PRINT_PARAM,
	USB_CMD_CONTROL_MAX_SPEED,
	USB_CMD_PINCE,
	USB_CMD_AX12,
	USB_CMD_NUM       //!< nombre de commandes, laisser en dernier
};

void usb_add(uint16_t type, void* msg, uint16_t size);

void usb_add_cmd(enum usb_cmd id, void (*cmd)(void*));

#endif