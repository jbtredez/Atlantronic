#ifndef USB_H
#define USB_H

//! @file usb.h
//! @brief USB
//! @author Atlantronic

#ifdef __cplusplus
extern "C" {
#endif

#include "kernel/asm/asm_base_func.h"

#ifndef WEAK_USB
#define WEAK_USB __attribute__((weak, alias("nop_function") ))
#endif

enum
{
	USB_LOG = 1,
	USB_ERR,
	USB_HOKUYO,
	USB_HOKUYO_SEG,
	USB_CONTROL,
	USB_GO,
	USB_DETECTION_DYNAMIC_OBJECT_SIZE1,
	USB_DETECTION_DYNAMIC_OBJECT_SIZE2,
	USB_DETECTION_DYNAMIC_OBJECT_POLYLINE,
	USB_DETECTION_DYNAMIC_OBJECT1,
	USB_DETECTION_DYNAMIC_OBJECT2,
	USB_CAN_TRACE,
	USB_DATA_MAX,     //!< nombre d'id, laisser en dernier
};

enum usb_cmd
{
	USB_CMD_GET_VERSION = 0, // laisser en 0 pour la compatibilite
// ordres OS
	USB_CMD_PTASK,
	USB_CMD_REBOOT,
// ordres bas niveau
	USB_CMD_PWM,
// ordres haut niveau
	USB_CMD_TRAJECTORY,
	USB_CMD_MOTION_PARAM,
	USB_CMD_MOTION_PRINT_PARAM,
	USB_CMD_MOTION_MAX_SPEED,
	USB_CMD_MOTION_GOTO,
	USB_CMD_MOTION_SET_SPEED,
	USB_CMD_MOTION_SET_MAX_CURRENT,
	USB_CMD_MOTION_SET_ACTUATOR_KINEMATICS,
	USB_CMD_MOTION_ENABLE,
	USB_CMD_LOCATION_SET_POSITION,
	USB_CMD_WING,
	USB_CMD_DYNAMIXEL,
	USB_CMD_RECALAGE,
	USB_CMD_GO,
	USB_CMD_MATCH_TIME,
	USB_CMD_COLOR,
	USB_CMD_ARM,
	USB_CMD_STRAT,
	USB_CMD_CAN_SET_BAUDRATE,
	USB_CMD_CAN_WRITE,
	USB_CMD_GYRO_CALIB,
	USB_CMD_GYRO_SET_POSITION,
	USB_CMD_GYRO_SET_CALIBRATION_VALUES,
	USB_CMD_PUMP,
	USB_CMD_XBEE,
	USB_CMD_POWER,
	USB_CMD_HEARTBEAT,
	USB_CMD_ELEVATOR,
	USB_CMD_FINGER,
	USB_CMD_NUM       //!< nombre de commandes, laisser en dernier
};

struct usb_header
{
	uint16_t type;
	uint16_t size;
} __attribute__((packed));

//!< ajout de log
//!< le module usb doit être initialisé
void usb_add(uint16_t type, void* msg, uint16_t size) WEAK_USB;

void usb_add_log(unsigned char level, const char* func, uint16_t line, const char* msg) WEAK_USB;

//!< ajout d'une commande
//!< peut être appelée avant l'initialisation du module usb
void usb_add_cmd(enum usb_cmd id, void (*cmd)(void*));

static inline unsigned char usb_is_get_version_done()
{
	extern unsigned char usb_get_version_done;
	return usb_get_version_done;
}

#ifdef __cplusplus
}
#endif

#endif
