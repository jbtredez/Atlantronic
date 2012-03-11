#ifndef EVENT_H
#define EVENT_H

//! @file event.h
//! @brief Event
//! @author Atlantronic

#define EVENT_GO                          0x01    //!< évènement go
#define EVENT_END                         0x02    //!< évènement fin de match
#define EVENT_CONTROL_TARGET_REACHED      0x04    //!< évènement levé lors de l'arrivée au point voulu
#define EVENT_CONTROL_TARGET_NOT_REACHED  0x08    //!< évènement levé lorsque l'asservissement n'arrive pas à atteindre l'objectif
#define EVENT_CONTROL_COLSISION           0x10
#define EVENT_UART4_TC                    0x20
#define EVENT_USART3_TC                   0x40
#define EVENT_LOCAL_HOKUYO_UPDATE         0x80
#define EVENT_TRAJECTORY_UPDATE          0x100    //!< évènement de mise à jour de la trajectoire (usage interne à trajectory)
#define EVENT_CAN_TX_END                 0x200
#define EVENT_ADC_READY                  0x400
#define EVENT_AX12_SEND_COMPLETE         0x800
#define EVENT_DETECTION_UPDATE          x01000
#define EVENT_CONTROL_TIMEOUT           0x2000
#define EVENT_USB_READ                  0x4000
#define EVENT_USB_WRITE                 0x8000
#define EVENT_DETECTION_UPDATED        0x10000    //!< la tache detection a mis à jour les objets vus
#define EVENT_TRAJECTORY_END           0x20000    //!< évènement de fin de trajectoire

#endif
