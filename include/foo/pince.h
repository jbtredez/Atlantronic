#ifndef PINCE_H
#define PINCE_H

//! @file pince.h
//! @brief Gestion des pinces
//! @author Atlantronic

#include <stdint.h>

//!< ouverture des pincces
void pince_open();

//!< fermeture des pinces
void pince_close();

enum pince_cmd_type
{
	PINCE_CLOSE,
	PINCE_OPEN,
};

struct pince_cmd_arg
{
	uint32_t type;
};

#endif
