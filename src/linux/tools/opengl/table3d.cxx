#include "table3d.h"

#define SEA_SHELL_R         38.1f
#define CYLINDER_R          29.0f

Vect2 cubePt[] =
{
	Vect2( -29, -29),
	Vect2(  29, -29),
	Vect2(  29,  29),
	Vect2( -29,  29),
	Vect2( -29, -29),
};

Vect2 seaShellPt[] =
{
	Vect2( SEA_SHELL_R, 0),
	Vect2( SEA_SHELL_R * 0.707106781, SEA_SHELL_R * 0.707106781),
	Vect2( 0, SEA_SHELL_R),
	Vect2( -SEA_SHELL_R * 0.707106781, SEA_SHELL_R * 0.707106781),
	Vect2( -SEA_SHELL_R, 0),
	Vect2( -SEA_SHELL_R * 0.707106781, -SEA_SHELL_R * 0.707106781),
	Vect2( 0, -SEA_SHELL_R),
	Vect2( SEA_SHELL_R * 0.707106781, -SEA_SHELL_R * 0.707106781),
	Vect2( SEA_SHELL_R, 0),
};

Vect2 cylinderPt[] =
{
	Vect2( CYLINDER_R, 0),
	Vect2( CYLINDER_R * 0.707106781, CYLINDER_R * 0.707106781),
	Vect2( 0, CYLINDER_R),
	Vect2( -CYLINDER_R * 0.707106781, CYLINDER_R * 0.707106781),
	Vect2( -CYLINDER_R, 0),
	Vect2( -CYLINDER_R * 0.707106781, -CYLINDER_R * 0.707106781),
	Vect2( 0, -CYLINDER_R),
	Vect2( CYLINDER_R * 0.707106781, -CYLINDER_R * 0.707106781),
	Vect2( CYLINDER_R, 0),
};

struct polyline cube = { cubePt, sizeof(cubePt) / sizeof(cubePt[0]) };
struct polyline seaShell = { seaShellPt, sizeof(seaShellPt) / sizeof(seaShellPt[0]) };
struct polyline cylinder = { cylinderPt, sizeof(cylinderPt) / sizeof(cylinderPt[0]) };

bool Table3d::init(int glSelectBaseId, MainShader* shader, Qemu* qemu)
{
	showTable = true;
	showElements = true;
	m_shader = shader;
	m_qemu = qemu;

	bool res = m_glTable.init("media/2016/table2016.obj", shader);
	res &= m_glSandCone.init("media/2016/cone_sable.obj", shader);
	res &= m_glSandCube.init("media/2016/cube_sable.obj", shader);
	res &= m_glSandCylinder.init("media/2016/cylindre_sable.obj", shader);
	res &= m_glWhiteSeaShell.init("media/2016/coquillage_blanc.obj", shader);
	res &= m_glGreenSeaShell.init("media/2016/coquillage_vert.obj", shader);
	res &= m_glPurpleSeaShell.init("media/2016/coquillage_violet.obj", shader);
	res &= m_glGreenFish.init("media/2016/poisson_vert.obj", shader);
	res &= m_glPurpleFish.init("media/2016/poisson_violet.obj", shader);

	int selectId = glSelectBaseId;
	for(unsigned int i = TABLE_OBJ_SAND_CONE_START; i < TABLE_OBJ_SAND_CONE_START + NUM_SAND_CONE; i++)
	{
		res &= m_obj[i].init(&m_glSandCone, selectId++);
	}

	for(unsigned int i = TABLE_OBJ_SAND_CUBE_START; i < TABLE_OBJ_SAND_CUBE_START + NUM_SAND_CUBE; i++)
	{
		res &= m_obj[i].init(&m_glSandCube, selectId++);
	}

	for(unsigned int i = TABLE_OBJ_SAND_CYLINDER_START; i < TABLE_OBJ_SAND_CYLINDER_START + NUM_SAND_CYLINDER; i++)
	{
		res &= m_obj[i].init(&m_glSandCylinder, selectId++);
	}

	for(unsigned int i = TABLE_OBJ_WHITE_SEA_SHELL_START; i < TABLE_OBJ_WHITE_SEA_SHELL_START + NUM_WHITE_SEA_SHELL; i++)
	{
		res &= m_obj[i].init(&m_glWhiteSeaShell, selectId++);
	}

	for(unsigned int i = TABLE_OBJ_GREEN_SEA_SHELL_START; i < TABLE_OBJ_GREEN_SEA_SHELL_START + NUM_GREEN_SEA_SHELL; i++)
	{
		res &= m_obj[i].init(&m_glGreenSeaShell, selectId++);
	}

	for(unsigned int i = TABLE_OBJ_PURPLE_SEA_SHELL_START; i < TABLE_OBJ_PURPLE_SEA_SHELL_START + NUM_PURPLE_SEA_SHELL; i++)
	{
		res &= m_obj[i].init(&m_glPurpleSeaShell, selectId++);
	}

	for(unsigned int i = TABLE_OBJ_GREEN_FISH_START; i < TABLE_OBJ_GREEN_FISH_START + NUM_GREEN_FISH; i++)
	{
		res &= m_obj[i].init(&m_glGreenFish, selectId++);
	}

	for(unsigned int i = TABLE_OBJ_PURPLE_FISH_START; i < TABLE_OBJ_PURPLE_FISH_START + NUM_PURPLE_FISH; i++)
	{
		res &= m_obj[i].init(&m_glPurpleFish, selectId++);
	}

	initElementPosition(5);

	return res;
}

void Table3d::initElementPosition(int configuration)
{
	// sable
	m_obj[TABLE_OBJ_SAND_CUBE_START].setPosition(879, 129, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+1].setPosition(879, 71, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+2].setPosition(821, 129, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+3].setPosition(821, 71, 0);

	m_obj[TABLE_OBJ_SAND_CUBE_START+4].setPosition(649, 971, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+5].setPosition(649, 913, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+6].setPosition(591, 971, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+7].setPosition(591, 913, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+8].setPosition(649, 971, 58);
	m_obj[TABLE_OBJ_SAND_CUBE_START+9].setPosition(649, 913, 58);
	m_obj[TABLE_OBJ_SAND_CUBE_START+10].setPosition(591, 971, 58);
	m_obj[TABLE_OBJ_SAND_CUBE_START+11].setPosition(591, 913, 58);

	m_obj[TABLE_OBJ_SAND_CUBE_START+12].setPosition(232, 971, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+13].setPosition(174, 971, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+14].setPosition(116, 971, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+15].setPosition(58, 971, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+16].setPosition(58, 971, 58);
	m_obj[TABLE_OBJ_SAND_CUBE_START+17].setPosition(58, 913, 0);

	m_obj[TABLE_OBJ_SAND_CYLINDER_START].setPosition(850, 100, 58);
	m_obj[TABLE_OBJ_SAND_CYLINDER_START+1].setPosition(620, 942, 116);
	m_obj[TABLE_OBJ_SAND_CYLINDER_START+2].setPosition(174, 971, 58);
	m_obj[TABLE_OBJ_SAND_CYLINDER_START+3].setPosition(116, 971, 58);
	m_obj[TABLE_OBJ_SAND_CYLINDER_START+4].setPosition(116, 971, 116);
	m_obj[TABLE_OBJ_SAND_CYLINDER_START+5].setPosition(58, 971, 116);
	m_obj[TABLE_OBJ_SAND_CYLINDER_START+6].setPosition(58, 971, 174);
	m_obj[TABLE_OBJ_SAND_CYLINDER_START+7].setPosition(58, 913, 58);

	m_obj[TABLE_OBJ_SAND_CONE_START].setPosition(850, 100, 116);
	m_obj[TABLE_OBJ_SAND_CONE_START+1].setPosition(620, 942, 174);
	m_obj[TABLE_OBJ_SAND_CONE_START+2].setPosition(116, 971, 174);
	m_obj[TABLE_OBJ_SAND_CONE_START+3].setPosition(58, 971, 232);

	for(int i = 0; i < 18; i++)
	{
		m_obj[TABLE_OBJ_SAND_CUBE_START + 18 + i].position = m_obj[TABLE_OBJ_SAND_CUBE_START + i].position;
		m_obj[TABLE_OBJ_SAND_CUBE_START + 18 + i].position.x *= -1;
	}
	for(int i = 0; i < 8; i++)
	{
		m_obj[TABLE_OBJ_SAND_CYLINDER_START + 8 + i].position = m_obj[TABLE_OBJ_SAND_CYLINDER_START + i].position;
		m_obj[TABLE_OBJ_SAND_CYLINDER_START + 8 + i].position.x *= -1;
	}
	for(int i = 0; i < 4; i++)
	{
		m_obj[TABLE_OBJ_SAND_CONE_START + 4 + i].position = m_obj[TABLE_OBJ_SAND_CONE_START + i].position;
		m_obj[TABLE_OBJ_SAND_CONE_START + 4 + i].position.x *= -1;
	}

	m_obj[TABLE_OBJ_SAND_CUBE_START+36].setPosition(0, 971, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+37].setPosition(0, 913, 0);
	m_obj[TABLE_OBJ_SAND_CUBE_START+38].setPosition(0, 971, 58);
	m_obj[TABLE_OBJ_SAND_CUBE_START+39].setPosition(0, 913, 58);

	m_obj[TABLE_OBJ_SAND_CYLINDER_START+16].setPosition(0, 971, 116);
	m_obj[TABLE_OBJ_SAND_CYLINDER_START+17].setPosition(0, 971, 174);
	m_obj[TABLE_OBJ_SAND_CYLINDER_START+18].setPosition(0, 913, 116);
	m_obj[TABLE_OBJ_SAND_CYLINDER_START+19].setPosition(0, 855, 0);

	m_obj[TABLE_OBJ_SAND_CONE_START+8].setPosition(0, 971, 232);

	// coquillages
	switch( configuration )
	{
		default:
		case 1:
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START].setPosition(-1300, -250, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+1].setPosition(1300, -250, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+2].setPosition(-1300, -550, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+3].setPosition(1300, -550, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+4].setPosition(0, -550, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+5].setPosition(0, -850, 0);

			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START].setPosition(-600, -450, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+1].setPosition(300, -650, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+2].setPosition(-1425, -925, 66);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+3].setPosition(-1425, -800, 44);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+4].setPosition(-1300, -925, 44);
			break;
		case 2:
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START].setPosition(-1425, -925, 66);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+1].setPosition(1425, -925, 66);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+2].setPosition(-1300, -550, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+3].setPosition(1300, -550, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+4].setPosition(0, -550, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+5].setPosition(0, -850, 0);

			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START].setPosition(-1300, -250, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+1].setPosition(-600, -450, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+2].setPosition(-300, -650, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+3].setPosition(-1425, -800, 44);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+4].setPosition(-1300, -925, 44);
			break;
		case 3:
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START].setPosition(-1425, -925, 66);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+1].setPosition(1425, -925, 66);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+2].setPosition(-1300, -550, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+3].setPosition(1300, -550, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+4].setPosition(-800, -550, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+5].setPosition(800, -550, 0);

			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START].setPosition(-1300, -250, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+1].setPosition(-800, -250, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+2].setPosition(-300, -650, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+3].setPosition(-1425, -800, 44);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+4].setPosition(-1300, -925, 44);
			break;
		case 4:
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START].setPosition(-1425, -925, 66);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+1].setPosition(1425, -925, 66);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+2].setPosition(-800, -550, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+3].setPosition(800, -550, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+4].setPosition(-300, -650, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+5].setPosition(300, -650, 0);

			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START].setPosition(-1300, -250, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+1].setPosition(-1300, -550, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+2].setPosition(-800, -250, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+3].setPosition(1425, -800, 44);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+4].setPosition(1300, -925, 44);
			break;
		case 5:
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START].setPosition(-1425, -925, 66);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+1].setPosition(1425, -925, 66);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+2].setPosition(-1300, -925, 44);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+3].setPosition(1300, -925, 44);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+4].setPosition(-800, -850, 0);
			m_obj[TABLE_OBJ_WHITE_SEA_SHELL_START+5].setPosition(800, -850, 0);

			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START].setPosition(-1300, -250, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+1].setPosition(-1300, -550, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+2].setPosition(-800, -250, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+3].setPosition(800, -550, 0);
			m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START+4].setPosition(-1425, -800, 44);
			break;
	}

	for(int i = 0; i < NUM_PURPLE_SEA_SHELL; i++)
	{
		m_obj[TABLE_OBJ_GREEN_SEA_SHELL_START + i].position = m_obj[TABLE_OBJ_PURPLE_SEA_SHELL_START + i].position;
		m_obj[TABLE_OBJ_GREEN_SEA_SHELL_START + i].position.x *= -1;
	}

	// poissons
	m_obj[TABLE_OBJ_GREEN_FISH_START].setPosition(-700, -1150, 70);
	m_obj[TABLE_OBJ_GREEN_FISH_START+1].setPosition(-775, -1110, 70);
	m_obj[TABLE_OBJ_GREEN_FISH_START+2].setPosition(-850, -1150, 70);
	m_obj[TABLE_OBJ_GREEN_FISH_START+3].setPosition(-925, -1100, 70);
	for(int i = 0; i < NUM_PURPLE_FISH; i++)
	{
		m_obj[TABLE_OBJ_PURPLE_FISH_START + i].position = m_obj[TABLE_OBJ_GREEN_FISH_START + i].position;
		m_obj[TABLE_OBJ_PURPLE_FISH_START + i].position.x *= -1;
		m_obj[TABLE_OBJ_PURPLE_FISH_START + i].theta = m_obj[TABLE_OBJ_GREEN_FISH_START + i].theta + M_PI;
	}
}

bool Table3d::initQemuObjects()
{
	if( ! m_qemu )
	{
		return false;
	}

	for(unsigned int i = TABLE_OBJ_SAND_CONE_START; i < TABLE_OBJ_SAND_CONE_START + NUM_SAND_CONE; i++)
	{
		m_qemu->add_object(OBJECT_MOBILE | OBJECT_SEEN_BY_HOKUYO, cylinder, &m_qemuObjId[i]);
	}

	for(unsigned int i = TABLE_OBJ_SAND_CUBE_START; i < TABLE_OBJ_SAND_CUBE_START + NUM_SAND_CUBE; i++)
	{
		m_qemu->add_object(OBJECT_MOBILE | OBJECT_SEEN_BY_HOKUYO, cube, &m_qemuObjId[i]);
	}

	for(unsigned int i = TABLE_OBJ_SAND_CYLINDER_START; i < TABLE_OBJ_SAND_CYLINDER_START + NUM_SAND_CYLINDER; i++)
	{
		m_qemu->add_object(OBJECT_MOBILE | OBJECT_SEEN_BY_HOKUYO, cylinder, &m_qemuObjId[i]);
	}

	for(unsigned int i = TABLE_OBJ_WHITE_SEA_SHELL_START; i < TABLE_OBJ_WHITE_SEA_SHELL_START + NUM_WHITE_SEA_SHELL; i++)
	{
		m_qemu->add_object(OBJECT_MOBILE | OBJECT_SEEN_BY_HOKUYO, seaShell, &m_qemuObjId[i]);
	}

	for(unsigned int i = TABLE_OBJ_GREEN_SEA_SHELL_START; i < TABLE_OBJ_GREEN_SEA_SHELL_START + NUM_GREEN_SEA_SHELL; i++)
	{
		m_qemu->add_object(OBJECT_MOBILE | OBJECT_SEEN_BY_HOKUYO, seaShell, &m_qemuObjId[i]);
	}

	for(unsigned int i = TABLE_OBJ_PURPLE_SEA_SHELL_START; i < TABLE_OBJ_PURPLE_SEA_SHELL_START + NUM_PURPLE_SEA_SHELL; i++)
	{
		m_qemu->add_object(OBJECT_MOBILE | OBJECT_SEEN_BY_HOKUYO, seaShell, &m_qemuObjId[i]);
	}

	for(unsigned int i = TABLE_OBJ_GREEN_FISH_START; i < TABLE_OBJ_GREEN_FISH_START + NUM_GREEN_FISH; i++)
	{
		m_qemuObjId[i] = -1;
	}

	for(unsigned int i = TABLE_OBJ_PURPLE_FISH_START; i < TABLE_OBJ_PURPLE_FISH_START + NUM_PURPLE_FISH; i++)
	{
		m_qemuObjId[i] = -1;
	}

	for(unsigned int i = 0; i < TABLE_OBJ_MAX; i++)
	{
		m_qemu->move_object(m_qemuObjId[i], Vect2(), VectPlan(m_obj[i].position.x, m_obj[i].position.y, m_obj[i].theta));
	}

	return true;
}

void Table3d::select(unsigned int id)
{
	for(unsigned int i = 0; i < TABLE_OBJ_MAX; i++)
	{
		if( m_obj[i].selectionId == id )
		{
			m_obj[i].selected = true;
		}
	}
}

void Table3d::unselectAll()
{
	for(unsigned int i = 0; i < TABLE_OBJ_MAX; i++)
	{
		m_obj[i].selected = false;
	}
}

void Table3d::moveSelected(float dx, float dy)
{
	for(unsigned int i = 0; i < TABLE_OBJ_MAX; i++)
	{
		if( m_obj[i].selected )
		{
			if( m_qemu )
			{
				m_qemu->move_object(m_qemuObjId[i], Vect2(m_obj[i].position.x, m_obj[i].position.y), VectPlan(dx, dy, 0));
			}
			m_obj[i].position.x += dx;
			m_obj[i].position.y += dy;
		}
	}
}

void Table3d::draw()
{
	if(showTable)
	{
		// on baisse la table de 1mm pour eviter des pb d'affichage avec les points places en z=0
		glm::mat4 oldModelView = m_shader->getModelView();
		glm::mat4 modelView = glm::translate(oldModelView, glm::vec3(0, 0, -1));
		m_shader->setModelView(modelView);
		m_glTable.draw();
		m_shader->setModelView(oldModelView);
	}

	if( showElements )
	{
		for(unsigned int i = 0; i < TABLE_OBJ_MAX; i++)
		{
			m_obj[i].draw();
		}
	}
}
