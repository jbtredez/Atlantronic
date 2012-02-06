#ifndef DETECTION_H
#define DETECTION_H

//! @file detection.h
//! @brief Detection
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"

//!< Calcule en fonction de la position (actuelle ou non) pos du robot le segment [a b] (repère table) tel
//!< que [a b] soit la projection - sur l'axe Y du repère robot - de l'obstacle le plus proche dans l'axe du robot.
//!< => dans le repère robot, a.x = b.x = distance avant collision (valeur retournée) et [a.y b.y] largeur de
//!< l'objet vue par le robot. Le robot ne doit pas dépasser le segment [a b]
//!<
//!< attention, prise de mutex sur l'ensemble des segments
//!<
//!< @return distance maximale d'avance avant collision
int32_t detection_compute_front_object(struct fx_vect_pos* pos, struct fx_vect2* a, struct fx_vect2* b);

#endif