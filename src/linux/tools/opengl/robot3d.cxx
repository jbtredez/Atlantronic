#include "robot3d.h"

bool Robot3d::init(MainShader* shader)
{
	m_shader = shader;
	bool res = m_mobileBase.init("media/robot2015.obj", shader);
	res &= m_wing.init("media/wing.obj", shader);
	res &= m_elevator.init("media/elevator.obj", shader);
	res &= m_finger.init("media/finger.obj", shader);
	res &= m_carpet.init("media/brasTapis.obj", shader);

	rightWingTheta = 0;
	leftWingTheta = 0;
	elevatorHeight = 0;
	highFingerTheta = 0;
	lowFingerTheta = 0;
	rightCarpetTheta = 0;
	leftCarpetTheta = 0;

	return res;
}

void Robot3d::draw()
{
	glm::mat4 oldModelView = m_shader->getModelView();

	// garde au sol de 5mm
	glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(0, 0, 5));
	m_shader->setModelView(modelView);

	drawMobileBase();

	drawElevator();

	drawWing(false);
	drawWing(true);
	drawCarpet(false);
	drawCarpet(true);
	m_shader->setModelView(oldModelView);
}

void Robot3d::drawMobileBase()
{
	glm::mat4 oldModelView = m_shader->getModelView();
	glm::mat4 modelView = glm::rotate(oldModelView, (float)M_PI/2, glm::vec3(0, 0, 1));
	m_shader->setModelView(modelView);
	m_mobileBase.draw();
	m_shader->setModelView(oldModelView);
}

void Robot3d::drawElevator()
{
	glm::mat4 oldModelView = m_shader->getModelView();
	glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(49, 0, elevatorHeight + 32));
	m_shader->setModelView(modelView);
	m_elevator.draw();

	glm::mat4 oldModelView2 = m_shader->getModelView();
	modelView = glm::translate(oldModelView2, glm::vec3(45.5, 50, 31));
	modelView = glm::rotate(modelView, highFingerTheta, glm::vec3(0, 0, 1));
	m_shader->setModelView(modelView);
	m_finger.draw();
	m_shader->setModelView(oldModelView2);

	modelView = glm::translate(oldModelView2, glm::vec3(45.5, -50, 39));
	modelView = glm::rotate(modelView, (float)M_PI, glm::vec3(1, 0, 0));
	modelView = glm::rotate(modelView, -lowFingerTheta, glm::vec3(0, 0, 1));
	m_shader->setModelView(modelView);
	m_finger.draw();

	m_shader->setModelView(oldModelView);
}

void Robot3d::drawWing(bool right)
{
	int side = 1;
	float theta = -M_PI/2 + leftWingTheta;
	if(right)
	{
		side = -1;
		theta = -M_PI/2 + rightWingTheta;
	}
	glm::mat4 oldModelView = m_shader->getModelView();
	glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(34.5, side * 135, 95));
	modelView = glm::rotate(modelView, theta, glm::vec3(0, 0, 1));
	m_shader->setModelView(modelView);
	m_wing.draw();
	m_shader->setModelView(oldModelView);
}

void Robot3d::drawCarpet(bool right)
{
	int side = 1;
	float theta = leftCarpetTheta;
	if(right)
	{
		theta = -rightCarpetTheta;
		side = -1;
	}
	glm::mat4 oldModelView = m_shader->getModelView();
	glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(-100.5, side * 60, 77.4));
	modelView = glm::rotate(modelView, (float)M_PI/2, glm::vec3(0, 0, 1));
	modelView = glm::rotate(modelView, theta, glm::vec3(1, 0, 0));
	m_shader->setModelView(modelView);
	m_carpet.draw();
	m_shader->setModelView(oldModelView);
}
