#ifndef _TEMPS_H_
#define _TEMPS_H_

//! @file temps.c
//! @brief Gestion du temps et de la durée du match
//! @author Jean-Baptiste Trédez

//! temps système
//!
//! ATTENTION : ne pas utiliser cette version depuis une interruption
//!
//! @return temps écoulé en tick depuis le lancement du pic (depuis l'appel à vTaskStartScheduler())
extern unsigned long tempsSysteme();

//! temps depuis le début du match
//!
//! ATTENTION : ne pas utiliser cette version depuis une interruption
//!
//! @return temps écoulé en tick depuis le début du match
extern unsigned long tempsMatch();

//! temps système, appel depuis une interruption
//!
//! @return temps écoulé en tick depuis le lancement du pic (depuis l'appel à vTaskStartScheduler())
extern unsigned long tempsSystemeFromISR();

//! temps depuis le début du match, appel depuis une interruption
//!
//! @return temps écoulé en tick depuis le début du match
extern unsigned long tempsMatchFromISR();

#endif
