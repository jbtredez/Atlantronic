#ifndef POWER_H
#define POWER_H

//! @file power.h
//! @brief Gestion puissance
//! @author Atlantronic

#define POWER_ON                 0x00
#define POWER_OFF                0x01    //!< extinction de la puissance par la strategie
#define POWER_OFF_UNDERVOLTAGE   0x02    //!< extinction de la puissance a cause d'une sous tension batterie
#define POWER_OFF_END_MATCH      0x04    //!< extinction de la puissance a cause de la fin du match

void power_set(int powerEventMask);

void power_clear(int powerEventMask);

#endif
