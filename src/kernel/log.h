#ifndef LOG_H
#define LOG_H

//! @file log.h
//! @brief Log task, log avec differents niveaux, formatés ou non (prend beaucoup plus de stack pour les logs formatés)
//! @author Atlantronic

#include "kernel/systick.h"
#include "kernel/portmacro.h"
#include "kernel/asm/asm_base_func.h"
#include "kernel/rcc.h"
#include "kernel/log_level.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>

// taille max d'un log (espace qui doit être dispo sur la stack)
#define LOG_SIZE             100

#ifndef NO_WEAK_LOG
void log_format_and_add(unsigned char level, const char* func, uint16_t line, const char* msg, ...) __attribute__(( format(printf, 4, 5) )) __attribute__((weak, alias("nop_function") ));
void log_add(unsigned char level, const char* func, uint16_t line, const char* msg) __attribute__((weak, alias("nop_function") ));
#else
void log_format_and_add(unsigned char level, const char* func, uint16_t line, const char* msg, ...) __attribute__(( format(printf, 4, 5) ));
void log_add(unsigned char level, const char* func, uint16_t line, const char* msg);
#endif

#define log(level, msg) \
	do \
	{ \
		if(level <= LOG_LEVEL) \
		{ \
			log_add(level, __FUNCTION__, __LINE__, msg);\
		} \
	}while(0)

#define log_format(level, msg, arg ...) \
	do \
	{ \
		if(level <= LOG_LEVEL) \
		{ \
			log_format_and_add(level, __FUNCTION__, __LINE__, msg, ##arg);\
		} \
	}while(0)

#endif
