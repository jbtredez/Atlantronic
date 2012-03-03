#ifndef POLY7_H
#define POLY7_H

#include "kernel/math/kinematics.h"

void poly7_simple(struct kinematics* A, struct kinematics* B, int32_t d, int32_t* a, int32_t* b);

void poly7f_full(float x1, float y1, float alpha1, float v1, float w1, float x2, float y2, float alpha2, float v2, float w2, float* a, float* b, float* u);

#endif