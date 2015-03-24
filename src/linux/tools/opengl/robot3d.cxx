#include "robot3d.h"

bool Robot3d::init()
{
	bool res = m_mobileBase.init("media/robot2015.obj");
	res &= m_wing.init("media/wing.obj");
	res &= m_elevator.init("media/elevator.obj");
	res &= m_finger.init("media/finger.obj");

	rightWingTheta = 0;
	leftWingTheta = 0;
	elevatorHeight = 0;
	highFingerTheta = 0;
	lowFingerTheta = 0;

	return res;
}

void Robot3d::draw()
{
	// garde au sol de 5mm
	glTranslatef(0, 0, 5);

	drawMobileBase();

	drawElevator();

	drawWing(false);
	drawWing(true);
}

void Robot3d::drawMobileBase()
{
	glPushMatrix();
	glRotatef(90, 0, 0, 1);
	m_mobileBase.draw();
	glPopMatrix();
}

void Robot3d::drawElevator()
{
	glPushMatrix();
	glTranslatef(49, 0, elevatorHeight + 32);
	m_elevator.draw();

	glPushMatrix();
	glTranslatef(45.5, 50, 31);
	glRotatef(highFingerTheta * 180 / M_PI, 0, 0, 1);
	m_finger.draw();
	glPopMatrix();

	glTranslatef(45.5, -50, 39);
	glRotatef(180, 1, 0, 0);
	glRotatef(lowFingerTheta * 180 / M_PI, 0, 0, 1);
	m_finger.draw();

	glPopMatrix();
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
	glTranslatef(34.5, side * 135, 100);
	glRotatef(theta,0,0,1);
	m_wing.draw();
	glPopMatrix();
}
