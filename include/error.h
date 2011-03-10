#ifndef ERROR_H
#define ERROR_H

#include "io/gpio.h"


// code 0x00 réservé : fin du match

#define ERR_NMI                 0x01
#define ERR_HARD_FAULT          0x02
#define ERR_MPU_FAULT           0x03
#define ERR_BUS_FAULT           0x04
#define ERR_USAGE_FAULT         0x05
#define ERR_UNEXPECTED_ISR      0x06
#define ERR_DEBUG_MONITOR       0x07

#define ERR_SYSTICK             0x08
#define ERR_INIT_LOG            0x09
#define ERR_INIT_AX12           0x0a
#define ERR_INIT_CONTROL        0x0b
#define ERR_INIT_TEST           0x0c
#define ERR_INIT_END            0x0d

#define ERR_ARU                0x201      //!< 2 led rouges

#endif
