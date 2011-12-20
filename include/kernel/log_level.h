#ifndef LOG_LEVEL_H
#define LOG_LEVEL_H

//! @file log_level.h
//! @brief Log level
//! @author Atlantronic

enum log_level
{
	LOG_ERROR,
	LOG_INFO,
	LOG_DEBUG1,
	LOG_DEBUG2,
	LOG_MAX,
};

//! niveau de log compile
#define LOG_LEVEL        LOG_INFO


#endif
