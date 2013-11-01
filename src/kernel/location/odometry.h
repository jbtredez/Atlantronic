#ifndef ODOMETRY_H
#define ODOMETRY_H

//! @file odometry.h
//! @brief Odometry
//! @author Atlantronic

#include "kernel/math/vect_plan.h"

VectPlan odometry2turret(const VectPlan &cp, const VectPlan &A, const VectPlan &B, const VectPlan &v1, const VectPlan &v2, float* slippageSpeed);

#endif
