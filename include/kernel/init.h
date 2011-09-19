#ifndef INIT_H
#define INIT_H

//! @file init.h
//! @brief Contient l'ordre d'initialisation des modules s'ils sont présent
//! @author Atlantronic
//!
//! il s'agit d'un tri par ordre alphabétique (donc "01" < "02" < "1" < "10" < "2" )
//! réalisé à l'édition des liens.
//! pour faire simple et lisible, on n'utilise que les chiffres et
//! on préfixe de "0" : "00" < "01" < "09" < "10" < "19" < "20".
//! en cas d'égalité, l'ordre d'initialisation entre ces fonctions est
//! "la première trouvée par l'éditeur de liens sera la première exécutée".
//!
//! les #define sont ordonnés pour indiquer le sens d'exécution (croissant pour l'init)

#include "kernel/error.h"

// init frequence cpu, bus internes...
#define INIT_RCC                      "00"

// init gpio pour afficher les erreurs sur les led
#define INIT_GPIO                     "01"

// init des log (buffer circulaire)
#define INIT_LOG                      "02"

// init de la communication
#define INIT_USB                      "03"
#define INIT_USART                    "04"
#define INIT_CAN                      "05"

// init adc
#define INIT_ADC                      "06"

#define INIT_MODEL                    "06"
#define INIT_ENCODERS                 "07"
#define INIT_CURRENT                  "08"
#define INIT_PWM                      "09"
#define INIT_AX12                     "10"
#define INIT_ODOMETRY                 "11"
#define INIT_CAN_US                   "11"
#define INIT_DETECTION                "12"
#define INIT_BEACON                   "13"
#define INIT_LOCATION                 "14"
//#define INIT_VFS                      "02"
#define INIT_END                      "15"
#define INIT_CONTROL                  "16"
#define INIT_CONTROL_PINCE            "17"
#define INIT_STRATEGY                 "20"
#define INIT_TEST                     "30"
#define INIT_TEST_PINCE               "30"
#define INIT_TEST_DEPLACEMENT         "30"
#define INIT_TEST_TASK1               "31"
#define INIT_TEST_TASK2               "32"

// init de l'ordonanceur en DERNIER !
#define INIT_SYSTICK                  "51"

#define EXIT_CONTROL_PINCE            "17"
#define EXIT_CONTROL                  "16"
#define EXIT_AX12                     "10"

#endif
