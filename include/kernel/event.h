#ifndef EVENT_H
#define EVENT_H

//! @file event.h
//! @brief Event
//! @author Atlantronic

#define EVENT_GO                          0x01
#define EVENT_CONTROL_READY               0x02
#define EVENT_AX12_READ_COMPLETE          0x04
#define EVENT_END                         0x08
#define EVENT_DMA2_3_TC                   0x10
#define EVENT_UART4_ERROR                 0x20
#define EVENT_DMA1_3_TC                   0x40
#define EVENT_USART3_ERROR                0x80
#define EVENT_CAN_TX_END                 0x100
#define EVENT_ADC_READY                  0x200
#define EVENT_CONTROL_PINCE_READY        0x400
#define EVENT_CONTROL_COLSISION          0x800
#define EVENT_CONTROL_PINCE_COLISION    0x1000
#define EVENT_CONTROL_TIMEOUT           0x2000
#define EVENT_HOKUYO_READY              0x4000

#endif
