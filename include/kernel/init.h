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
#include "kernel/error_codes.h"

// init frequence cpu, bus internes...
#define INIT_RCC                      "00"

// init gpio, usb et error pour afficher les erreurs sur les led et envoyer des log
#define INIT_GPIO                     "01"
#define INIT_USB                      "02"
#define INIT_FAULT                    "03"
#define INIT_SPI                      "04"
#define INIT_GYRO                     "05"
#define INIT_ACCELERO                 "06"

// init de la communication
#define INIT_CAN                      "07"
#define INIT_CAN_MOTOR                "08"

// init adc
#define INIT_ADC                      "09"
#define INIT_XBEE                     "10"

#define INIT_ENCODERS                 "10"
#define INIT_PWM                      "11"
#define INIT_DYNAMIXEL                "12"
#define INIT_CAN_US                   "13"
#define INIT_HOKUYO                   "14"
#define INIT_DETECTION                "15"
#define INIT_BEACON                   "16"
#define INIT_LOCATION                 "17"
#define INIT_END                      "18"
#define INIT_MOTION                   "19"
#define INIT_CONTROL                  "20"
#define INIT_PINCE                    "21"
#define INIT_ARM                      "22"
#define INIT_TRAJECTORY               "23"
#define INIT_PUMP                     "24"

#define INIT_STRATEGY                 "40"

#define INIT_TEST                     "50"
#define INIT_TEST_PINCE               "50"
#define INIT_TEST_DEPLACEMENT         "50"
#define INIT_TEST_TASK1               "51"
#define INIT_TEST_TASK2               "52"

// init de l'ordonanceur en DERNIER !
#define INIT_SYSTICK                  "99"

#define EXIT_PWM                      "1"

#endif
