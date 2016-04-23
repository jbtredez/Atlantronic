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
#define INIT_USB                      "03"
#define INIT_XBEE                     "04"
#define INIT_FAULT                    "04"
#define INIT_POWER                    "05"
#define INIT_SPI                      "06"
#define INIT_GYRO                     "07"
#define INIT_ACCELERO                 "08"
#define INIT_ESP8266                  "09"
// init de la communication
#define INIT_CAN                      "10"

#define INIT_SDRAM                    "12"
#define INIT_I2C                      "13"
#define INIT_STMPE811                 "14"
#define INIT_LCD                      "15"

// init adc
#define INIT_ADC                      "20"

#define INIT_ENCODERS                 "31"
#define INIT_PWM                      "32"
#define INIT_END                      "39"
#define INIT_MAIN_ROBOT               "40"
#define INIT_CONTROL                  "41"
#define INIT_WING                     "42"
//#define INIT_ARM                      "43"

#define INIT_PUMP                     "45"
#define INIT_ELEVATOR                 "46"
#define INIT_FINGER                   "47"
#define INIT_CARPET                   "48"

#define INIT_STRATEGY                 "60"

#define INIT_TEST                     "80"
#define INIT_TEST_PINCE               "80"
#define INIT_TEST_DEPLACEMENT         "80"
#define INIT_TEST_TASK1               "81"
#define INIT_TEST_TASK2               "82"

// init de l'ordonanceur en DERNIER !
#define INIT_SYSTICK                  "99"

#define EXIT_PWM                      "1"
#define EXIT_FINGER                   "2"
#define EXIT_POWER                   "10"

#endif
