#ifndef REGRESSION_H
#define REGRESSION_H

#include "kernel/vect_pos.h"

//! @file regression.h
//! @brief Calcul d'une régression linéaire
//! @author Atlantronic

// TODO a mettre en virgule fixe si on l'utilise
#ifdef LINUX
//! calcul la droite y = a*x + b par la méthode des moindres carrés avec la pondération w
//! si wi = 0, le couple (xi, yi) n'est pas pris en compte. Plus wi est grand, plus la mesure est importante
int regression_linear(float* x, float* y, float* w, int size, float* a, float* b);
#endif

int regression_poly(struct vect2* pt, int size, int seuil, struct vect2* regression_pt, int reg_size);

#endif
