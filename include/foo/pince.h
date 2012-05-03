#ifndef PINCE_H
#define PINCE_H

//! @file pince.h
//! @brief Gestion des pinces
//! @author Atlantronic

#include <stdint.h>

//!< configuration des pinces
void pince_configure();

//!< ouverture des pincces
void pince_open();

//!< fermeture des pinces
void pince_close();

enum pince_cmd_type
{
	PINCE_CONFIGURE,
	PINCE_OPEN,
	PINCE_CLOSE,
};

struct pince_cmd_arg
{
	uint32_t type;
};

#endif
