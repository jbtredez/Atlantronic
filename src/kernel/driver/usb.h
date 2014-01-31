#ifndef USB_H
#define USB_H

//! @file usb.h
//! @brief USB
//! @author Atlantronic

#ifdef __cplusplus
extern "C" {
#endif

enum
{
	USB_LOG = 1,
	USB_ERR,
	USB_HOKUYO1,
	USB_HOKUYO2,
	USB_HOKUYO_FOO_SEG,
	USB_CONTROL,
	USB_GO,
	USB_DETECTION_DYNAMIC_OBJECT_SIZE,
	USB_DETECTION_DYNAMIC_OBJECT,
	USB_CAN_TRACE,
};

enum usb_cmd
{
	USB_CMD_TRAJECTORY = 0,
	USB_CMD_CONTROL_PARAM,
	USB_CMD_CONTROL_PRINT_PARAM,
	USB_CMD_CONTROL_MAX_SPEED,
	USB_CMD_CONTROL_GOTO,
	USB_CMD_CONTROL_SET_SPEED,
	USB_CMD_CONTROL_FREE,
	USB_CMD_LOCATION_SET_POSITION,
	USB_CMD_PINCE,
	USB_CMD_DYNAMIXEL,
	USB_CMD_RECALAGE,
	USB_CMD_GO,
	USB_CMD_MATCH_TIME,
	USB_CMD_COLOR,
	USB_CMD_ARM_GOTO,
	USB_CMD_ARM_BRIDGE,
	USB_CMD_STRAT,
	USB_CMD_CAN_SET_BAUDRATE,
	USB_CMD_CAN_WRITE,
	USB_CMD_GYRO_CALIB,
	USB_CMD_NUM       //!< nombre de commandes, laisser en dernier
};

//!< ajout de log
//!< le module usb doit être initialisé
void usb_add(uint16_t type, void* msg, uint16_t size);

//!< ajout d'une commande
//!< peut être appelée avant l'initialisation du module usb
void usb_add_cmd(enum usb_cmd id, void (*cmd)(void*));

#ifdef __cplusplus
}
#endif

#endif
