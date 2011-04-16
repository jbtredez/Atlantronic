#ifndef ERROR_H
#define ERROR_H

#include "io/gpio.h"


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

#define ERR_UART4_READ_SR_FE            0x20e
#define ERR_UART4_READ_SR_NE            0x20f
#define ERR_UART4_READ_SR_ORE           0x210

#define ERR_CAN_FILTER_LIST_FULL        0x214

#endif
