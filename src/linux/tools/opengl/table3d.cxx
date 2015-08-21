#include "table3d.h"
#include "linux/tools/opengl/gltools.h"

bool Table3d::init(int _glSelectFeetName[16], int _glSelectGlassName[5], MainShader* shader)
{
	showTable = true;
	showElements = true;
	m_shader = shader;

	bool res = table.init("media/table2015.obj", shader);
	res &= dispenser.init("media/distributeur.obj", shader);
	res &= clapYellow.init("media/clap_jaune.obj", shader);
	res &= clapGreen.init("media/clap_vert.obj", shader);
	res &= feetYellow.init("media/pied_jaune.obj", shader);
	res &= feetGreen.init("media/pied_vert.obj", shader);
	res &= glass.init("media/verre.obj", shader);

	feetPosition[0] = aiVector3D(-1410, -850, 0);
	feetPosition[1] = aiVector3D(-1410, -750, 0);
	feetPosition[2] = aiVector3D(-1410,  800, 0);
	feetPosition[3] = aiVector3D(- 650,  800, 0);
	feetPosition[4] = aiVector3D(- 650,  900, 0);
	feetPosition[5] = aiVector3D(- 630, -355, 0);
	feetPosition[6] = aiVector3D(- 400, -750, 0);
	feetPosition[7] = aiVector3D(- 200, -400, 0);

	feetPosition[8]  = aiVector3D(1410, -850, 0);
	feetPosition[9]  = aiVector3D(1410, -750, 0);
	feetPosition[10] = aiVector3D(1410,  800, 0);
	feetPosition[11] = aiVector3D( 650,  800, 0);
	feetPosition[12] = aiVector3D( 650,  900, 0);
	feetPosition[13] = aiVector3D( 630, -355, 0);
	feetPosition[14] = aiVector3D( 400, -750, 0);
	feetPosition[15] = aiVector3D( 200, -400, 0);

	glassPosition[0] = aiVector3D(-1250, -750, 0);
	glassPosition[1] = aiVector3D(- 590,  170, 0);
	glassPosition[2] = aiVector3D(    0, -650, 0);
	glassPosition[3] = aiVector3D(  590,  170, 0);
	glassPosition[4] = aiVector3D( 1250, -750, 0);

	for(unsigned int i = 0; i < sizeof(feetSelected) / sizeof(feetSelected[0]); i++)
	{
		feetSelected[i] = false;
		glSelectFeetName[i] = _glSelectFeetName[i];
	}

	for(unsigned int i = 0; i < sizeof(glassSelected) / sizeof(glassSelected[0]); i++)
	{
		glassSelected[i] = false;
		glSelectGlassName[i] = _glSelectGlassName[i];
	}

	return res;
}

void Table3d::selectFeet(unsigned int id)
{
	if( id < sizeof(feetSelected) / sizeof(feetSelected[0]))
	{
		feetSelected[id] = true;
	}
}

void Table3d::selectGlass(unsigned int id)
{
	if( id < sizeof(glassSelected) / sizeof(glassSelected[0]))
	{
		glassSelected[id] = true;
	}
}

void Table3d::unselectAll()
{
	memset(feetSelected, 0, sizeof(feetSelected));
	memset(glassSelected, 0, sizeof(glassSelected));
}

void Table3d::moveSelected(float dx, float dy)
{
	for(unsigned int i = 0; i < sizeof(feetSelected) / sizeof(feetSelected[0]); i++)
	{
		if( feetSelected[i] )
		{
			feetPosition[i].x += dx;
			feetPosition[i].y += dy;
		}
	}

	for(unsigned int i = 0; i < sizeof(glassSelected) / sizeof(glassSelected[0]); i++)
	{
		if( glassSelected[i] )
		{
			glassPosition[i].x += dx;
			glassPosition[i].y += dy;
		}
	}

}

void Table3d::draw(GLenum mode)
{
	if(mode != GL_SELECT && showTable)
	{
		glm::mat4 oldModelView = m_shader->getModelView();
		glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(-table.sceneCenter.x, -table.sceneCenter.y, -table.sceneMin.z-22));
		m_shader->setModelView(modelView);
		table.draw();
		m_shader->setModelView(oldModelView);

		drawDispenser(-1200);
		drawDispenser(- 900);
		drawDispenser(  900);
		drawDispenser( 1200);

		drawClap(-1100, true);
		drawClap(- 800, false);
		drawClap(- 500, true);

		drawClap(  500, false);
		drawClap(  800, true);
		drawClap( 1100, false);
	}

	if( showElements )
	{
		drawFeets(mode);
		drawGlass(mode);
	}
}

void Table3d::drawDispenser(float x)
{
	glm::mat4 oldModelView = m_shader->getModelView();
	glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(x, 1022, 0));
	modelView = glm::rotate(modelView, (float)M_PI, glm::vec3(0, 0, 1));
	modelView = glm::translate(modelView, glm::vec3(-dispenser.sceneCenter.x, -dispenser.sceneMin.y, -dispenser.sceneMin.z));
	m_shader->setModelView(modelView);
	dispenser.draw();
	m_shader->setModelView(oldModelView);
}

void Table3d::drawClap(float x, bool yellow)
{
	Object3d* clap = &clapYellow;
	if( ! yellow )
	{
		clap = &clapGreen;
	}
	glm::mat4 oldModelView = m_shader->getModelView();
	glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(x, -1000, 78));
	if( x > 0)
	{
		modelView = glm::rotate(modelView, (float)M_PI, glm::vec3(0, 0, 1));
	}
	if( x < 0)
	{
		modelView = glm::translate(modelView, glm::vec3(-clap->sceneMax.x, -clap->sceneMax.y, -clap->sceneMin.z));
	}
	else
	{
		modelView = glm::translate(modelView, glm::vec3(-clap->sceneMax.x, -clap->sceneMin.y, -clap->sceneMin.z ));
	}
	m_shader->setModelView(modelView);
	clap->draw();
	m_shader->setModelView(oldModelView);
}

void Table3d::drawFeets(GLenum mode)
{
	Object3d* feet = &feetYellow;
	for(unsigned int i = 0; i < sizeof(feetPosition) / sizeof(feetPosition[0]); i++)
	{
		if( i >= sizeof(feetPosition) / (2*sizeof(feetPosition[0])) )
		{
			feet = &feetGreen;
		}
		glPushName(glSelectFeetName[i]);
		glm::mat4 oldModelView = m_shader->getModelView();
		glm::mat4 modelView = glm::translate(oldModelView, glm::vec3( -feet->sceneCenter.x + feetPosition[i].x, -feet->sceneCenter.y + feetPosition[i].y, -feet->sceneMin.z + feetPosition[i].z));
		m_shader->setModelView(modelView);
		feet->selected = feetSelected[i];
		if(mode != GL_SELECT)
		{
			feet->draw();
		}
		else
		{
			//plot_boundingBox(feet->sceneMin.x, feet->sceneMin.y, feet->sceneMin.z, feet->sceneMax.x, feet->sceneMax.y, feet->sceneMax.z);
		}
		m_shader->setModelView(oldModelView);
		glPopName();
	}
}

void Table3d::drawGlass(GLenum mode)
{
	for(unsigned int i = 0; i < sizeof(glassPosition) / sizeof(glassPosition[0]); i++)
	{
		glPushName(glSelectGlassName[i]);
		glm::mat4 oldModelView = m_shader->getModelView();
		glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(-glass.sceneCenter.x + glassPosition[i].x, -glass.sceneCenter.y + glassPosition[i].y, -glass.sceneMin.z + glassPosition[i].z));
		m_shader->setModelView(modelView);
		glass.selected = glassSelected[i];
		if(mode != GL_SELECT)
		{
			glass.draw();
		}
		else
		{
			//plot_boundingBox(glass.sceneMin.x, glass.sceneMin.y, glass.sceneMin.z, glass.sceneMax.x, glass.sceneMax.y, glass.sceneMax.z);
		}
		m_shader->setModelView(oldModelView);
		glPopName();
	}
}
