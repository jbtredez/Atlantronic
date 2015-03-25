#ifndef GLPLOT_H
#define GLPLOT_H

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>

void plot_pave(float x, float y, float z, float dx, float dy, float dz);

void plot_boundingBox(float x1, float y1, float z1, float x2, float y2, float z2);

void draw_plus(float x, float y, float rx, float ry);

#endif

