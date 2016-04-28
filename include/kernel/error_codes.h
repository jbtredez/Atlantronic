#ifndef ERROR_CODES_H
#define ERROR_CODES_H

//! @file error_codes.h
//! @brief Error codes
//! @author Atlantronic

//! @enum fault
//! defauts
//! ne pas oublier de mettre a jour la description dans robot_interface
enum fault
{
	// erreurs OS
	FAULT_UNALIGNED,
	FAULT_DIVBY0,

	// HOKUYO
	FAULT_HOKUYO_DISCONNECTED,
	FAULT_HOKUYO_DATA_CORRUPTION, // erreur FE/NE/ORE sur usart, check_cmd, unknown status, checksum

	// RPLIDAR
	FAULT_RPLIDAR_DISCONNECTED,
	FAULT_RPLIDAR_DATA_CORRUPTION, // erreur FE/NE/ORE sur usart

	// CAN
	FAULT_CAN_NOT_CONNECTED,
	FAULT_CAN_READ_QUEUE_FULL,
	FAULT_CAN_READ_FIFO_OVERFLOW,

	FAULT_CAN_MOTOR_DISCONNECTED_0,
	FAULT_CAN_MOTOR_DISCONNECTED_1,

	// GYRO
	FAULT_GYRO_DISCONNECTED,
	FAULT_GYRO_ERROR,

	FAULT_MAX,
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
#define ERR_INIT_DYNAMIXEL      0x20a
#define ERR_INIT_CONTROL        0x20b
#define ERR_INIT_DETECTION      0x20c
#define ERR_INIT_END            0x20d
#define ERR_INIT_HOKUYO         0x20e
#define ERR_INIT_CAN            0x20f
#define ERR_INIT_TRAJECTORY     0x210
#define ERR_INIT_CAN_US         0x211
#define ERR_INIT_ERROR          0x212
#define ERR_INIT_ODOMETRY       0x213
#define ERR_INIT_FAULT          0x214
#define ERR_INIT_STRAT          0x215
#define ERR_INIT_ARM            0x216
#define ERR_INIT_PINCE          0x217
#define ERR_INIT_ADC            0x218
#define ERR_INIT_XBEE           0x219
#define ERR_INIT_ESP8266        0x220
#define ERR_INIT_TEST           0x221

#endif
