#ifndef EVENT_H
#define EVENT_H

//! @file event.h
//! @brief Event
//! @author Atlantronic

#define EVENT_CONTROL_TARGET_REACHED      0x04    //!< évènement levé lors de l'arrivée au point voulu
#define EVENT_CONTROL_TARGET_NOT_REACHED  0x08    //!< évènement levé lorsque l'asservissement n'arrive pas à atteindre l'objectif
#define EVENT_CONTROL_COLSISION           0x10
#define EVENT_LOCAL_HOKUYO_UPDATE         0x80
#define EVENT_TRAJECTORY_UPDATE          0x100    //!< évènement de mise à jour de la trajectoire (usage interne à trajectory)

#define EVENT_AX12_SEND_COMPLETE         0x800
#define EVENT_DETECTION_UPDATE          0x1000
#define EVENT_CONTROL_TIMEOUT           0x2000
#define EVENT_DETECTION_UPDATED        0x10000    //!< la tache detection a mis à jour les objets vus
#define EVENT_TRAJECTORY_END           0x20000    //!< évènement de fin de trajectoire
#define EVENT_SICK                     0x40000    //!< évènement de modification de l'état des sick

#endif
