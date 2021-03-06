#include "GateRobot3d.h"

bool GateRobot3d::init(MainShader* shader)
{
	m_shader = shader;
	bool res = m_mobileBase.init("media/2016/GateRobot.obj", shader);

	return res;
}

void GateRobot3d::draw()
{
	glm::mat4 oldModelView = m_shader->getModelView();

	// garde au sol de 5mm
	glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(0, 0, 5));
	m_shader->setModelView(modelView);

	drawMobileBase();

	m_shader->setModelView(oldModelView);
}

void GateRobot3d::drawMobileBase()
{
	glm::mat4 oldModelView = m_shader->getModelView();
	glm::mat4 modelView = glm::rotate(oldModelView, (float)M_PI/2, glm::vec3(0, 0, 1));
	m_shader->setModelView(modelView);
	m_mobileBase.draw();
	m_shader->setModelView(oldModelView);
}
