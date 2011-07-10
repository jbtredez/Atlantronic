#ifndef LOG_H
#define LOG_H

//! @file log.h
//! @brief Log task
//! @author Atlantronic

#include "kernel/systick.h"
#include "kernel/portmacro.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>

// niveau de log à compiler
#define _erreur_           1
#define _info_             2


// taille max d'un log (espace qui doit être dispo sur la stack) : taille de la terminaison usb
#define LOG_SIZE         64

#if 0

//! Macro de log formatés. Permet de faire des log par type avec plusieurs niveaux de log
//! C'est une macro pour permettre d'avoir __FILE__, __FUNCTION__ et __LINE__ valide
//!
//! remarque : le while(0) permet de ne pas avoir de pb avec les if() meslog else ...
//! permet d'obliger de mettre le ; comme toute instruction
//! bloc viré (optimisation du compilo activé) selon le type et le niveau de log activé
# define meslog(type, niveau, msg, arg ...) do{	\
	if(type >= niveau){				\
		log_add( "%li\t%s:%i\t%s: "msg"\n",systick_get_match_time(),#type,niveau,__FUNCTION__,##arg);	\
	}				\
	else if(type < 0) { \
		log_add( LOG_ROUGE("%li\t%s:%i\t%s:%s:%i "msg"\n"),systick_get_match_time(),#type,niveau,__FILE__,__FUNCTION__,__LINE__,##arg);\
	} \
}while(0)

# define meslogFromISR(type, niveau, msg, arg ...) do{	\
	if(type >= niveau){				\
		log_add_from_isr( "%li\t%s:%i\t%s: "msg"\n",systick_get_match_time(),#type,niveau,__FUNCTION__,##arg);	\
	}				\
	else if(type < 0) { \
		log_add_from_isr( LOG_ROUGE("%li\t%s:%i\t%s:%s:%i "msg"\n"),systick_get_match_time(),#type,niveau,__FILE__,__FUNCTION__,__LINE__,##arg);\
	} \
}while(0)

# define meslogc(couleur, type, niveau, msg, arg ...) do{	\
	if(type >= niveau){				\
		log_add( LOG_COULEUR(couleur, "%li\t%s:%i\t%s: "msg"\n" ),systick_get_match_time(),#type,niveau,__FUNCTION__,##arg);	\
	}				\
	else if(type < 0) { \
		log_add( LOG_ROUGE("%li\t%s:%i\t%s:%s:%i "msg"\n"),systick_get_match_time(),#type,niveau,__FILE__,__FUNCTION__,__LINE__,##arg);\
	} \
}while(0)

# define logerror(msg,arg ...)	do{	\
	log_add( LOG_ROUGE("%li\t_erreur_\t%s:%s:%i\t"msg": %s""\n"),systick_get_match_time(),__FILE__,__FUNCTION__,__LINE__,##arg,strerror(errno));	\
}while(0) //!< Macro de log d'erreur formate (récupère l'erreur dans ERRNO)

#else

# define meslog(type, niveau, msg, arg ...)

# define meslogFromISR(type, niveau, msg, arg ...)

# define meslogc(couleur, type, niveau, msg, arg ...)

# define logerror(msg,arg ...)

#endif

#endif
