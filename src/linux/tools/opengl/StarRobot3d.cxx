#include "StarRobot3d.h"

bool StarRobot3d::init(MainShader* shader)
{
	m_shader = shader;
	bool res = m_mobileBase.init("media/2016/StarRobot.obj", shader);
	res &= m_leftFishWing.init("media/2016/leftFishWing.obj", shader);
	res &= m_rightFishWing.init("media/2016/rightFishWing.obj", shader);
	res &= m_fishRemover.init("media/2016/fishRemover.obj", shader);

	leftFishWingTheta = 0;
	rightFishWingTheta = 0;
	leftFishRemoverTheta = 0;
	rightFishRemoverTheta = 0;

	return res;
}

void StarRobot3d::draw()
{
	glm::mat4 oldModelView = m_shader->getModelView();

	// garde au sol de 5mm
	glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(0, 0, 5));
	m_shader->setModelView(modelView);

	drawMobileBase();
	drawFishWings();

	m_shader->setModelView(oldModelView);
}

void StarRobot3d::drawMobileBase()
{
	glm::mat4 oldModelView = m_shader->getModelView();
	glm::mat4 modelView = glm::rotate(oldModelView, (float)M_PI/2, glm::vec3(0, 0, 1));
	m_shader->setModelView(modelView);
	m_mobileBase.draw();
	m_shader->setModelView(oldModelView);
}

void StarRobot3d::drawFishWings()
{
	glm::mat4 oldModelView = m_shader->getModelView();
	glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(20, 90, 83));
	modelView = glm::rotate(modelView, leftFishWingTheta, glm::vec3(-1, 0, 0));
	modelView = glm::rotate(modelView, (float)M_PI/2, glm::vec3(0, 0, 1));
	m_shader->setModelView(modelView);
	m_leftFishWing.draw();

	modelView = glm::rotate(modelView, (float)-M_PI/2, glm::vec3(0, 0, 1));
	modelView = glm::translate(modelView, glm::vec3(-19.75, 16, 29.5));
	modelView = glm::rotate(modelView, leftFishRemoverTheta, glm::vec3(0, 1, 0));
	modelView = glm::rotate(modelView, (float)M_PI/2, glm::vec3(1, 0, 0));
	modelView = glm::rotate(modelView, (float)-M_PI/2, glm::vec3(0, 0, 1));
	m_shader->setModelView(modelView);
	m_fishRemover.draw();
	m_shader->setModelView(oldModelView);


	modelView = glm::translate(oldModelView, glm::vec3(20, -90, 83));
	modelView = glm::rotate(modelView, rightFishWingTheta, glm::vec3(-1, 0, 0));
	modelView = glm::rotate(modelView, (float)M_PI/2, glm::vec3(0, 0, 1));
	m_shader->setModelView(modelView);
	m_rightFishWing.draw();
	modelView = glm::rotate(modelView, (float)M_PI/2, glm::vec3(0, 0, 1));
	modelView = glm::translate(modelView, glm::vec3(19.75, 16, 29.5));
	modelView = glm::rotate(modelView, rightFishRemoverTheta, glm::vec3(0, 1, 0));
	modelView = glm::rotate(modelView, (float)M_PI/2, glm::vec3(1, 0, 0));
	modelView = glm::rotate(modelView, (float)-M_PI/2, glm::vec3(0, 0, 1));
	m_shader->setModelView(modelView);
	m_fishRemover.draw();
	m_shader->setModelView(oldModelView);
}
