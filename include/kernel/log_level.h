#ifndef LOG_LEVEL_H
#define LOG_LEVEL_H

//! @file log_level.h
//! @brief Log level
//! @author Atlantronic

#ifdef __cplusplus
extern "C" {
#endif

enum log_level
{
	LOG_ERROR,      //!< logs d'erreur
	LOG_INFO,       //!< logs d'info
	LOG_DEBUG1,     //!< logs de debug (pas trop verbeux)
	LOG_DEBUG2,     //!< logs de debug (logs verbeux, ou periodiques 100ms)
	LOG_DEBUG3,     //!< logs de debug (logs verbeux, ou periodiques  < 100ms)
	LOG_MAX,
};

//! niveau de log compile
#define LOG_LEVEL        LOG_INFO

#ifdef __cplusplus
}
#endif

#endif
