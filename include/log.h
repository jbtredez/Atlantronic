//! @file log.h
//! @brief Log task
//! @author Jean-Baptiste Trédez

#ifndef LOG_H
#define LOG_H

#include "time2.h"
#include "portmacro.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>

# ifndef DCOULEUR
# define DCOULEUR	1 //!< erreur affichées en rouge par défaut.
# endif

# if DCOULEUR
#    define LOG_COULEUR(i, arg...) "\033[01;3"#i"m" arg "\033[00m" //!< macro permettant de choisir la couleur i du terminal
# else
#    define LOG_COULEUR(i, arg...) (arg)
# endif

# define LOG_ROUGE(arg ...)			LOG_COULEUR(1, arg) //!<couleur du message rouge
# define LOG_VERT(arg ...)			LOG_COULEUR(2, arg) //!<couleur du message vert
# define LOG_JAUNE(arg ...)			LOG_COULEUR(3, arg) //!<couleur du message jaune
# define LOG_BLEU(arg ...)			LOG_COULEUR(4, arg) //!<couleur du message bleu

// niveau de log à compiler
// < 0 : compiler les log de ce type comme log d'erreur (sur stderr) en rouge (si DCOULEUR)
#define _info_             1
#define _erreur_          -1

#define LOG_SIZE         128

#define LOG_QUEUE_SIZE   10

int log_add(const char* msg, ...);
portBASE_TYPE log_add_from_isr(const char* msg, ...);

//! Macro de log formatés. Permet de faire des log par type avec plusieurs niveaux de log
//! C'est une macro pour permettre d'avoir __FILE__, __FUNCTION__ et __LINE__ valide
//!
//! remarque : le while(0) permet de ne pas avoir de pb avec les if() meslog else ...
//! permet d'obliger de mettre le ; comme toute instruction
//! bloc viré (optimisation du compilo activé) selon le type et le niveau de log activé
# define meslog(type, niveau, msg, arg ...) do{	\
	if(type >= niveau){				\
		log_add( "%li\t%s:%i\t%s: "msg"\n",time_match(),#type,niveau,__FUNCTION__,##arg);	\
	}				\
	else if(type < 0) { \
		log_add( LOG_ROUGE("%li\t%s:%i\t%s:%s:%i "msg"\n"),time_match(),#type,niveau,__FILE__,__FUNCTION__,__LINE__,##arg);\
	} \
}while(0)

# define meslogFromISR(type, niveau, msg, arg ...) do{	\
	if(type >= niveau){				\
		log_add_from_isr( "%li\t%s:%i\t%s: "msg"\n",time_match(),#type,niveau,__FUNCTION__,##arg);	\
	}				\
	else if(type < 0) { \
		log_add_from_isr( LOG_ROUGE("%li\t%s:%i\t%s:%s:%i "msg"\n"),time_match(),#type,niveau,__FILE__,__FUNCTION__,__LINE__,##arg);\
	} \
}while(0)

# define meslogc(couleur, type, niveau, msg, arg ...) do{	\
	if(type >= niveau){				\
		log_add( LOG_COULEUR(couleur, "%li\t%s:%i\t%s: "msg"\n" ),time_match(),#type,niveau,__FUNCTION__,##arg);	\
	}				\
	else if(type < 0) { \
		log_add( LOG_ROUGE("%li\t%s:%i\t%s:%s:%i "msg"\n"),time_match(),#type,niveau,__FILE__,__FUNCTION__,__LINE__,##arg);\
	} \
}while(0)

# define logerror(msg,arg ...)	do{	\
	log_add( LOG_ROUGE("%li\t_erreur_\t%s:%s:%i\t"msg": %s""\n"),time_match(),__FILE__,__FUNCTION__,__LINE__,##arg,strerror(errno));	\
}while(0) //!< Macro de log d'erreur formate (récupère l'erreur dans ERRNO)

#endif
