#ifndef ERROR_CODES_H
#define ERROR_CODES_H

//! @file error_codes.h
//! @brief Error codes
//! @author Atlantronic

//! @enum fault
//! defauts
//! ne pas oublier de mettre a jour la description dans foo_interface
enum fault
{
	ERR_SUCCESS,
	// CAN
	ERR_CAN_READ_QUEUE_FULL,
	ERR_CAN_READ_FIFO_OVERFLOW,
	ERR_CAN_FILTER_LIST_FULL,

	// USART
	ERR_USART_UNKNOWN_DEVICE,

	// HOKUYO
	FAULT_HOKUYO_DISCONNECTED,
	FAULT_HOKUYO_DATA_CORRUPTION, // erreur FE/NE/ORE sur usart, check_cmd, unknown status, checksum

	// math
	ERR_LINEAR_REG_NULL_DET,

	ERR_MAX,
};

// codes pour les led (erreur grave, pas de log par usb)
// code 0x00 réservé : fin du match
#define ERR_NMI                 0x200
#define ERR_ARU                 0x201      //!< 2 led rouges
#define ERR_HARD_FAULT          0x202
#define ERR_MPU_FAULT           0x203
#define ERR_BUS_FAULT           0x204
#define ERR_USAGE_FAULT         0x205
#define ERR_UNEXPECTED_ISR      0x206
#define ERR_DEBUG_MONITOR       0x207

#define ERR_SYSTICK             0x208
#define ERR_INIT_USB            0x209
#define ERR_INIT_AX12           0x20a
#define ERR_INIT_CONTROL        0x20b
#define ERR_INIT_DETECTION      0x20c
#define ERR_INIT_END            0x20d
#define ERR_INIT_HOKUYO         0x20e
#define ERR_INIT_CAN            0x20f
#define ERR_INIT_TRAJECTORY     0x210
#define ERR_INIT_CAN_US         0x211
#define ERR_INIT_ERROR          0x212
#define ERR_INIT_ODOMETRY       0x213
#define ERR_INIT_TEST           0x214

#endif
