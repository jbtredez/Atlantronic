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
	USB_CMD_LOCATION_SET_POSITION,
	USB_CMD_PINCE,
	USB_CMD_AX12,
	USB_CMD_RECALAGE,
	USB_CMD_GO,
	USB_CMD_MATCH_TIME,
	USB_CMD_COLOR,
	USB_CMD_ARM,
	USB_CMD_ARM_BRIDGE,
	USB_CMD_NUM       //!< nombre de commandes, laisser en dernier
};

//!< ajout de log
//!< le module usb doit être initialisé
void usb_add(uint16_t type, void* msg, uint16_t size);

//!< ajout d'une commande
//!< peut être appelée avant l'initialisation du module usb
void usb_add_cmd(enum usb_cmd id, void (*cmd)(void*));

#endif