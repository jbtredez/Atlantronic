//! @file log.h
//! @brief Tache de log
//! @author Jean-Baptiste Trédez

#include "temps.h"
#include <stdio.h>
#include <errno.h>

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

//! taille maximale d'un log
#define TAILLE_MESSAGE_MAX	128

//! nombre maximum de log en attente d'écriture
#define NB_ELEMENT_QUEUE_LOG   10

extern int ajouterLog(const char* msg, ...);

extern int logInitialise();

//! Macro de log formatés. Permet de faire des log par type avec plusieurs niveaux de log
//! C'est une macro pour permettre d'avoir __FILE__, __FUNCTION__ et __LINE__ valide
//!
//! remarque : le while(0) permet de ne pas avoir de pb avec les if() meslog else ...
//! permet d'obliger de mettre le ; comme toute instruction
//! bloc viré (optimisation du compilo activé) selon le type et le niveau de log activé
# define meslog(type, niveau, msg, arg ...) do{	\
	if(type >= niveau){				\
		ajouterLog( "%li\t%s:%i\t%s: "msg"\n",tempsMatch(),#type,niveau,__FUNCTION__,##arg);	\
	}				\
	else if(type < 0) { \
		ajouterLog( LOG_ROUGE("%li\t%s:%i\t%s:%s:%i "msg"\n"),tempsMatch(),#type,niveau,__FILE__,__FUNCTION__,__LINE__,##arg);\
	} \
}while(0)

# define meslogc(couleur, type, niveau, msg, arg ...) do{	\
	if(type >= niveau){				\
		ajouterLog( LOG_COULEUR(couleur, "%li\t%s:%i\t%s: "msg"\n" ),tempsMatch(),#type,niveau,__FUNCTION__,##arg);	\
	}				\
	else if(type < 0) { \
		ajouterLog( LOG_ROUGE("%li\t%s:%i\t%s:%s:%i "msg"\n"),tempsMatch(),#type,niveau,__FILE__,__FUNCTION__,__LINE__,##arg);\
	} \
}while(0)

# define logerror(msg,arg ...)	do{	\
	ajouterLog( LOG_ROUGE("%li\t_erreur_\t%s:%s:%i\t"msg": %s""\n"),tempsMatch(),__FILE__,__FUNCTION__,__LINE__,##arg,strerror(errno));	\
}while(0) //!< Macro de log d'erreur formate (récupère l'erreur dans ERRNO)

