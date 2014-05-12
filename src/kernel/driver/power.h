#ifndef POWER_H
#define POWER_H

//! @file power.h
//! @brief Gestion puissance
//! @author Atlantronic

#define POWER_ON                 0x00
#define POWER_OFF                0x01    //!< extinction de la puissance par la strategie
#define POWER_OFF_UNDERVOLTAGE   0x02    //!< extinction de la puissance a cause d'une sous tension batterie
#define POWER_OFF_END_MATCH      0x04    //!< extinction de la puissance a cause de la fin du match
#define POWER_OFF_AU             0x08    //!< extinction de la puissance a cause d'un AU

void power_set(int powerEventMask);

void power_clear(int powerEventMask);

//------------------ interface usb -------------------
struct power_cmd_arg
{
	int power_off;         //!< 0 pour laisser la puissance, !=0 sinon
} __attribute__((packed));

//------------------ fonctions inline------------------

//! @return 0 si on a la puissance, masque POWER_OFF_* sinon
static inline int power_get()
{
	extern int power_state;
	return power_state;
}

#endif
