//! @file model.h
//! @brief Model of the robot
//! @author Jean-Baptiste Trédez

#include <stdint.h>

//! Mise à jour d'une pwm du robot
//! Met le modèle à jour automatiquement avant la mise à jour de la pwm
//!
//! @param num numéro de la pwm
//! @param val valeur
//! @param dir direction
void model_pwm_set(unsigned int num, uint32_t val, int dir);

//! Lecture d'un codeur du robot
//! Met automatiquement le modèle à jour avant la lecture du codeur
//!
//! @param num numéro du codeur
//! @return valeur du codeur
uint16_t model_encoders_get(unsigned int num);

uint32_t model_current_get(unsigned int num);

//! Mise à jour du modèle
//!
void model_update();
