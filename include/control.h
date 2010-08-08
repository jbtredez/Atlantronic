#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Jean-Baptiste Trédez

////////! @todo asservissement - choix ECN/ESEO
enum control_state
{
	READY,          //!< no trajectory ongoing
	ROTATE,         //!< rotate
	STRAIGHT,       //!< go straight
	ARC,            //!< arc
};
////////

#endif
