#ifndef MODULE_H
#define MODULE_H

//! @file module.h
//! @brief fournit les fonctions d'enregistrement des points d'entrée et de sortie des modules
//! @author Atlantronic
//!
//! Prévu pour gcc
//! Fonctionnement :
//!    - un utilisateur créer une fonction "int mon_module_init()" qui retourne 0 si l'init s'est bien faite, un code d'erreur sinon
//!    - l'utilisateur enregistre la fonction avec : monule_init(mon_module_init, INIT_MON_MODULE) ou INIT_MON_MODULE est la priorité du module (cf init.h)
//!    - (fonctionnement interne) la macro module_init enregistre un pointeur vers la fonction mon_module_init dans la section .initcall.init.INIT_MON_MODULE
//!    - (fonctionnement interne) à l'édition des liens, on utilise un script (scripts/elcPC.ld par exemple) qui permet de trier les sections .initcall.init.* et on range le tout dans la section .init.data. Les pointeurs __initcall_start et __initcall_end sont définis au début de la section .init.data et à la fin. On peut voir l'ensemble comme un tableau de pointeur de fonction débutant à __initcall_start et se terminant à __initcall_end
//!    - l'utilisateur utilise la fonction initModules dans le programme principal, ce qui a pour effet de lire les pointeurs de fonctions enregistrés dans la section .init.data dans le bon ordre grâce à __initcall_start et __initcall_end

#include "init.h"

// macro pour optimiser les fonctions d'init en fonction de la taille
#define __OPTIMIZE_SIZE__ __attribute__((optimize("-Os")))
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x)   __builtin_expect(!!(x), 0)

typedef int (*initcall_t)();
typedef void (*exitcall_t)();

#define module_init(fn, lv) \
	static const initcall_t __initcall_##fn __attribute__((used)) \
	__attribute__((__section__(".initcall.init." lv))) = fn

#define module_exit(fn, lv) \
	static const exitcall_t __exitcall_##fn __attribute__((used)) \
	__attribute__((__section__(".exitcall.exit." lv))) = fn

//! Fonction d'initialisation des modules
//! Les modules sont initialisés dans l'ordre
//! Dés qu'une initialisation échoue, la fonction s'arrête. Ceci permet de
//! garantir aux fonctions d'initialisation que  leurs dépendances sont
//! initialisées avant.
//!
//! @return 0 si tout va bien, le code d'erreur de la fonction d'initialisation sinon
static inline int initModules()
{
	extern initcall_t __initcall_start[], __initcall_end[];
	initcall_t* init;
	int err;
	for(init = __initcall_start; init < __initcall_end; init++)
	{
		err = (*init)();
		if(err)
			return err;
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
