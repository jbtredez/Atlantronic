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
	// CAN
	ERR_CAN_READ_QUEUE_FULL,
	ERR_CAN_READ_FIFO_OVERFLOW,
	ERR_CAN_FILTER_LIST_FULL,

	// USART
	ERR_USART_UNKNOWN_DEVICE,

	// AX12
	ERR_AX12_DISCONNECTED,
	ERR_AX12_USART_FE,
	ERR_AX12_USART_NE,
	ERR_AX12_USART_ORE,
	ERR_AX12_SEND_CHECK,
	ERR_AX12_PROTO,
	ERR_AX12_CHECKSUM,
	ERR_AX12_INTERNAL_ERROR,
	ERR_AX12_INTERNAL_ERROR_INPUT_VOLTAGE,
	ERR_AX12_INTERNAL_ERROR_ANGLE_LIMIT,
	ERR_AX12_INTERNAL_ERROR_OVERHEATING,
	ERR_AX12_INTERNAL_ERROR_RANGE,
	ERR_AX12_INTERNAL_ERROR_CHECKSUM,
	ERR_AX12_INTERNAL_ERROR_OVERLOAD,
	ERR_AX12_INTERNAL_ERROR_INSTRUCTION,

	// HOKUYO
	ERR_HOKUYO_DISCONNECTED,
	ERR_HOKUYO_USART_FE,
	ERR_HOKUYO_USART_NE,
	ERR_HOKUYO_USART_ORE,
	ERR_HOKUYO_CHECK_CMD,
	ERR_HOKUYO_UNKNOWN_STATUS,
	ERR_HOKUYO_CHECKSUM,
	ERR_HOKUYO_BAUD_RATE,
	ERR_HOKUYO_LASER_MALFUNCTION,
	ERR_HOKUYO_SCAN_SIZE,
	ERR_HOKUYO_DISTANCE_BUFFER,

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
#define ERR_INIT_TEST           0x20c
#define ERR_INIT_END            0x20d
#define ERR_INIT_HOKUYO         0x20e
#define ERR_INIT_CAN            0x20f
#define ERR_INIT_CONTROL_PINCE  0x210
#define ERR_INIT_CAN_US         0x211
#define ERR_INIT_ERROR          0x212

#endif
