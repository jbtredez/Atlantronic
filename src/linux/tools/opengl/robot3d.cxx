#include "robot3d.h"

bool Robot3d::init()
{
	bool res = mobileBase.init("media/robot2015.obj");
	res &= wing.init("media/wing.obj");

	rightWingTheta = 0;
	leftWingTheta = 0;

	return res;
}

void Robot3d::draw()
{
	// garde au sol de 5mm
	glTranslatef(0, 0, 5);

	glPushMatrix();
	glRotatef(90, 0, 0, 1);
	mobileBase.draw();
	glPopMatrix();

	drawWing(false);
	drawWing(true);
}

void Robot3d::drawWing(bool right)
{
	int side = 1;
	float theta = -90 + leftWingTheta * 180 / M_PI;
	if(right)
	{
		side = -1;
		theta = -90 + rightWingTheta * 180 / M_PI;
	}
	glPushMatrix();
	glTranslatef(34.5, side * 135, 95);
	glRotatef(theta,0,0,1);
	wing.draw();
	glPopMatrix();
}
