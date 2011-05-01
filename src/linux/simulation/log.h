#ifndef _LOG_H_
#define _LOG_H_

//! @file log.h
//! @brief Permet de faire des log formates

#include <ctime>
#include <cstdio>
#include <cerrno>
#include <cstring>

# ifndef DCOULEUR
# define DCOULEUR	1 //!< erreures affichées en rouge par défaut.
# endif

# if DCOULEUR
# define LOG_COULEUR(i)		"\033[01;3"#i"m" //!< macro permettant de choisir la couleur i du terminal
# define LOG_ROUGE			LOG_COULEUR(1) //!<couleur du message rouge
# define LOG_VERT			LOG_COULEUR(2) //!<couleur du message vert
# define LOG_JAUNE			LOG_COULEUR(3) //!<couleur du message jaune
# define LOG_BLEU			LOG_COULEUR(4) //!<couleur du message bleu
# define LOG_FINCOULEUR		"\033[00m" //!< macro indiquant la fin du message coloré
# else
# define LOG_ROUGE	//!<couleur du message rouge
# define LOG_VERT	//!<couleur du message vert
# define LOG_BLEU	//!<couleur du message bleu
# define LOG_JAUNE	//!<couleur du message jaune
# define LOG_FINCOULEUR //!< macro indiquant la fin du message coloré
# endif

// 0: ne pas compiler les log de ce type
// > 0 : compiler les log de ce type comme log d'info (sur stdout)
// < 0 : compiler les log de ce type comme log d'erreur (sur stderr) en rouge (si DCOULEUR)
# define _erreur_ 		-1					//!< Indique que le log est une erreur, actif par defaut
# define _info_          1

extern int initLog();
extern timespec debutProgramme; //!< Infos temporelles sur le debut du programme
extern FILE* _logStdout; //!< fichier vers lequel on envoi les log envoyés par défaut sur "stdout"
extern FILE* _logStderr; //!< fichier vers lequel on envoi les log d'erreur envoyés par défaut sur "stderr"

//! nota: le while(0) permet de ne pas avoir de pb avec les if() meslog else ...
//! permet d'obliger de mettre le ; comme toute instruction
//! c'est viré par le préprocesseur avant la compilation donc pas de pb
//! de même le if(type) est viré avant la compilation vu que type est connu (constante)
//! d'ou un bloc non compilé si n'en veut pas
# define meslog(type,msg, arg ...) do{	\
 if(type){				\
	struct timespec _logTps;	\
	clock_gettime(0,&_logTps);	\
	_logTps.tv_sec -= debutProgramme.tv_sec;	\
	_logTps.tv_nsec -= debutProgramme.tv_nsec;	\
	double _logDeltat = (double) _logTps.tv_sec + ((double) _logTps.tv_nsec) / 1000000000;	\
	if(type > 0) fprintf(_logStdout, "%.6f\t%s\t%s: "msg"\n",_logDeltat,#type,__FUNCTION__,##arg);	\
	else fprintf(_logStderr, LOG_ROUGE"%.6f\t%s\t%s:%s:%i "msg"\n"LOG_FINCOULEUR,_logDeltat,#type,__FILE__,__FUNCTION__,__LINE__,##arg);\
 }					\
 }while(0) //!< Macro de log d'info/erreur formate

# define logerror(msg,arg ...)	do{	\
	struct timespec _logTps;	\
	clock_gettime(0,&_logTps);	\
	_logTps.tv_sec -= debutProgramme.tv_sec;	\
	_logTps.tv_nsec -= debutProgramme.tv_nsec;	\
	double _logDeltat = (double) _logTps.tv_sec + ((double) _logTps.tv_nsec) / 1000000000;	\
	fprintf(_logStderr, LOG_ROUGE"%.6f\t_erreur_\t%s:%s:%i\t"msg": %s""\n"LOG_FINCOULEUR,_logDeltat,__FILE__,__FUNCTION__,__LINE__,##arg,strerror(errno));	\
 }while(0) //!< Macro de log d'erreur formate (récupère l'erreur dans ERRNO)

#endif
