#ifndef STRAT_H
#define STRAT_H

//! @file strat.h
//! @brief Stratégie
//! @author Atlantronic

enum
{
	STRAT_BOUTEILLE,
};

struct strat_cmd_arg
{
	uint8_t type;
	int32_t arg1;
};

#endif