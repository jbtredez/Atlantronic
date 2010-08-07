#ifndef MODULE_H
#define MODULE_H

//! @file module.h
//! @brief fournit les fonctions d'enregistrement des points d'entrée et de sortie des modules
//! @author Jean-Baptiste Trédez

#include "init.h"

typedef int (*initcall_t)();
typedef int (*exitcall_t)();

#ifdef __GCC_PIC32__
	// microchip a déjà défini __section__ pour autre chose ...
	#define module_init(fn, lv) \
		static const initcall_t __initcall_##fn __attribute__((used)) \
		__attribute__((section(".initcall.init." lv))) = fn

	#define module_exit(fn, lv) \
		static const exitcall_t __exitcall_##fn __attribute__((used)) \
		__attribute__((section(".exitcall.exit." lv))) = fn
#else
	#define module_init(fn, lv) \
		static const initcall_t __initcall_##fn __attribute__((used)) \
		__attribute__((__section__(".initcall.init." lv))) = fn

	#define module_exit(fn, lv) \
		static const exitcall_t __exitcall_##fn __attribute__((used)) \
		__attribute__((__section__(".exitcall.exit." lv))) = fn
#endif

//! Fonction d'initialisation des modules
//! Les modules sont initialisés dans l'ordre
//! Dés qu'une initialisation échoue, la fonction s'arrête. Ceci permet de
//! garantir aux fonctions d'initialisation que  leurs dépendances sont
//! initialisées avant.
//!
//! @return 0 si tout va bien, -1 sinon
static inline int initModules()
{
	extern initcall_t __initcall_start[], __initcall_end[];
	initcall_t* init;
	for(init = __initcall_start; init < __initcall_end; init++){
		if((*init)())
			return -1;
	}

	return 0;
}

static inline void exitModules()
{
	extern exitcall_t __exitcall_start[], __exitcall_end[];
	exitcall_t* exit;
	for(exit = __exitcall_end-1; exit >= __exitcall_start; exit--)
		(*exit)();
}
#endif
