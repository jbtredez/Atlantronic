#include "robot3d.h"

bool Robot3d::init()
{
	bool res = mobileBase.init("media/robot2015.obj");
	res &= clapArm.init("media/wing.obj");

	rightClapArmTheta = 0;
	leftClapArmTheta = 0;

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

	drawClapArm(false);
	drawClapArm(true);
}

void Robot3d::drawClapArm(bool right)
{
	int side = 1;
	float theta = -90 + leftClapArmTheta * 180 / M_PI;
	if(right)
	{
		side = -1;
		theta = -90 + rightClapArmTheta * 180 / M_PI;
	}
	glPushMatrix();
	glTranslatef(34.5, side * 135, 95);
	glRotatef(theta,0,0,1);
	clapArm.draw();
	glPopMatrix();
}
