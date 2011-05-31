#ifndef ERROR_H
#define ERROR_H

#include "gpio.h"


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
#define ERR_INIT_LOG            0x209
#define ERR_INIT_AX12           0x20a
#define ERR_INIT_CONTROL        0x20b
#define ERR_INIT_TEST           0x20c
#define ERR_INIT_END            0x20d
#define ERR_INIT_HOKUYO         0x20e
#define ERR_INIT_CAN            0x20f
#define ERR_INIT_CONTROL_PINCE  0x210
#define ERR_INIT_CAN_US         0x211

#define ERR_USART3_READ_SR_FE           0x220
#define ERR_USART3_READ_SR_NE           0x221
#define ERR_USART3_READ_SR_ORE          0x222
#define ERR_UART4_READ_SR_FE            0x223
#define ERR_UART4_READ_SR_NE            0x224
#define ERR_UART4_READ_SR_ORE           0x225

#define ERR_USART_TIMEOUT               0x226
#define ERR_USART_UNKNOWN_DEVICE        0x227
#define ERR_USART_UNKNOWN_FREQUENCY     0x228

#define ERR_AX12_SEND_CHECK             0x229

#define ERR_CAN_FILTER_LIST_FULL        0x22a
#define ERR_CAN_READ_QUEUE_FULL         0x22b

#define ERR_HOKUYO_DISCONNECTED         0x22c
#define ERR_HOKUYO_CHECK_CMD            0x22d
#define ERR_HOKUYO_UNKNOWN_STATUS       0x22e
#define ERR_HOKUYO_CHECKSUM             0x22f
#define ERR_HOKUYO_BAUD_RATE            0x230
#define ERR_HOKUYO_LASER_MALFUNCTION    0x231

#define ERR_US_UNKNOWN_US		0x233

void error_raise(uint16_t error_number);

#endif
