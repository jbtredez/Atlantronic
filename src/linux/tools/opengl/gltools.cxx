#include "gltools.h"

void plot_pave(float x, float y, float z, float dx, float dy, float dz)
{
	glTranslatef(x, y, z);
	dx /= 2;
	dy /= 2;
	dz /= 2;

	glBegin(GL_QUADS);
	glNormal3f(-1, 0, 0);
	glVertex3f(-dx, -dy, -dz);
	glVertex3f(-dx,  dy, -dz);
	glVertex3f(-dx,  dy,  dz);
	glVertex3f(-dx, -dy,  dz);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(1, 0, 0);
	glVertex3f( dx, -dy, -dz);
	glVertex3f( dx,  dy, -dz);
	glVertex3f( dx,  dy,  dz);
	glVertex3f( dx, -dy,  dz);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(0, -1, 0);
	glVertex3f(-dx, -dy, -dz);
	glVertex3f(-dx, -dy,  dz);
	glVertex3f( dx, -dy,  dz);
	glVertex3f( dx, -dy, -dz);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(0, 1, 0);
	glVertex3f(-dx,  dy, -dz);
	glVertex3f(-dx,  dy,  dz);
	glVertex3f( dx,  dy,  dz);
	glVertex3f( dx,  dy, -dz);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(0, 0, -1);
	glVertex3f(-dx, -dy, -dz);
	glVertex3f( dx, -dy, -dz);
	glVertex3f( dx,  dy, -dz);
	glVertex3f(-dx,  dy, -dz);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(0, 0, 1);
	glVertex3f(-dx, -dy, dz);
	glVertex3f( dx, -dy, dz);
	glVertex3f( dx,  dy, dz);
	glVertex3f(-dx,  dy, dz);
	glEnd();

	glTranslatef(-x, -y, -z);
}

void draw_plus(float x, float y, float rx, float ry)
{
	glBegin(GL_LINES);
	glVertex2f(x-rx, y);
	glVertex2f(x+rx, y);
	glVertex2f(x, y-ry);
	glVertex2f(x, y+ry);
	glEnd();
}
