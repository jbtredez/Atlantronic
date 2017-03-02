#include "table3d.h"

#define TITANIUM_ORE_R         60.0f
#define MOON_ROCK_R            40.0f
#define LUNAR_MODULE_R         63.0f

Vect2 titaniumOrePt[] =
{
	Vect2( TITANIUM_ORE_R, 0),
	Vect2( TITANIUM_ORE_R * 0.707106781, TITANIUM_ORE_R * 0.707106781),
	Vect2( 0, TITANIUM_ORE_R),
	Vect2( -TITANIUM_ORE_R * 0.707106781, TITANIUM_ORE_R * 0.707106781),
	Vect2( -TITANIUM_ORE_R, 0),
	Vect2( -TITANIUM_ORE_R * 0.707106781, -TITANIUM_ORE_R * 0.707106781),
	Vect2( 0, -TITANIUM_ORE_R),
	Vect2( TITANIUM_ORE_R * 0.707106781, -TITANIUM_ORE_R * 0.707106781),
	Vect2( TITANIUM_ORE_R, 0),
};

Vect2 moonRockPt[] =
{
	Vect2( MOON_ROCK_R, 0),
	Vect2( MOON_ROCK_R * 0.707106781, MOON_ROCK_R * 0.707106781),
	Vect2( 0, MOON_ROCK_R),
	Vect2( -MOON_ROCK_R * 0.707106781, MOON_ROCK_R * 0.707106781),
	Vect2( -MOON_ROCK_R, 0),
	Vect2( -MOON_ROCK_R * 0.707106781, -MOON_ROCK_R * 0.707106781),
	Vect2( 0, -MOON_ROCK_R),
	Vect2( MOON_ROCK_R * 0.707106781, -MOON_ROCK_R * 0.707106781),
	Vect2( MOON_ROCK_R, 0),
};

Vect2 lunarModulePt[] =
{
	Vect2( LUNAR_MODULE_R, 0),
	Vect2( LUNAR_MODULE_R * 0.707106781, LUNAR_MODULE_R * 0.707106781),
	Vect2( 0, LUNAR_MODULE_R),
	Vect2( -LUNAR_MODULE_R * 0.707106781, LUNAR_MODULE_R * 0.707106781),
	Vect2( -LUNAR_MODULE_R, 0),
	Vect2( -LUNAR_MODULE_R * 0.707106781, -LUNAR_MODULE_R * 0.707106781),
	Vect2( 0, -LUNAR_MODULE_R),
	Vect2( LUNAR_MODULE_R * 0.707106781, -LUNAR_MODULE_R * 0.707106781),
	Vect2( LUNAR_MODULE_R, 0),
};

struct polyline titaniumOre = { titaniumOrePt, sizeof(titaniumOrePt) / sizeof(titaniumOrePt[0]) };
struct polyline moonRock = { moonRockPt, sizeof(moonRockPt) / sizeof(moonRockPt[0]) };
struct polyline lunarModule = { lunarModulePt, sizeof(lunarModulePt) / sizeof(lunarModulePt[0]) };

bool Table3d::init(int glSelectBaseId, MainShader* shader, Robot* robot, int robotCount)
{
	showTable = true;
	showElements = true;
	m_shader = shader;
	m_robot = robot;
	m_robotCount = robotCount;

	bool res = m_glTable.init("media/2017/table2017.obj", shader);
	res &= m_titaniumOre.init("media/2017/titaniumOre.obj", shader);
	res &= m_moonRock.init("media/2017/moonRock.obj", shader);
	res &= m_blueLunarModule.init("media/2017/blueLunarModule.obj", shader);
	res &= m_yellowLunarModule.init("media/2017/yellowLunarModule.obj", shader);
	res &= m_biLunarModule.init("media/2017/biLunarModule.obj", shader);

	int selectId = glSelectBaseId;
	for(unsigned int i = TABLE_OBJ_TITANIUM_ORE_START; i < TABLE_OBJ_TITANIUM_ORE_START + NUM_TITANIUM_ORE; i++)
	{
		res &= m_obj[i].init(&m_titaniumOre, selectId++);
	}

	for(unsigned int i = TABLE_MOON_ROCK_START; i < TABLE_MOON_ROCK_START + NUM_MOON_ROCK; i++)
	{
		res &= m_obj[i].init(&m_moonRock, selectId++);
	}

	for(unsigned int i = TABLE_BLUE_LUNAR_MODULE_START; i < TABLE_BLUE_LUNAR_MODULE_START + NUM_BLUE_LUNAR_MODULE; i++)
	{
		res &= m_obj[i].init(&m_blueLunarModule, selectId++);
	}

	for(unsigned int i = TABLE_YELLOW_LUNAR_MODULE_START; i < TABLE_YELLOW_LUNAR_MODULE_START + NUM_YELLOW_LUNAR_MODULE; i++)
	{
		res &= m_obj[i].init(&m_yellowLunarModule, selectId++);
	}

	for(unsigned int i = TABLE_BI_LUNAR_MODULE_START; i < TABLE_BI_LUNAR_MODULE_START + NUM_BI_LUNAR_MODULE; i++)
	{
		res &= m_obj[i].init(&m_biLunarModule, selectId++);
	}
	initElementPosition(1);

	return res;
}

void Table3d::initElementPosition(int /*configuration*/)
{
	// titanium ore
	// TODO coordonnees a mettre
	m_obj[TABLE_OBJ_TITANIUM_ORE_START].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+1].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+2].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+3].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+4].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+5].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+6].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+7].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+8].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+9].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+10].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+11].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+12].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+13].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+14].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+15].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+16].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+17].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+18].setPosition(0,0,0,0);
	m_obj[TABLE_OBJ_TITANIUM_ORE_START+19].setPosition(0,0,0,0);
	for(int i = TABLE_OBJ_TITANIUM_ORE_START; i< TABLE_OBJ_TITANIUM_ORE_START+20; i++)
	{
		m_obj[i+20].position = m_obj[i].position;
		m_obj[i+20].position.x *= -1;
	}

	// moon rock
	// TODO coordonnees a mettre
	m_obj[TABLE_MOON_ROCK_START].setPosition(0,0,0,0);
	m_obj[TABLE_MOON_ROCK_START+1].setPosition(0,0,0,0);
	m_obj[TABLE_MOON_ROCK_START+2].setPosition(0,0,0,0);
	m_obj[TABLE_MOON_ROCK_START+3].setPosition(0,0,0,0);
	m_obj[TABLE_MOON_ROCK_START+4].setPosition(0,0,0,0);
	m_obj[TABLE_MOON_ROCK_START+5].setPosition(0,0,0,0);
	m_obj[TABLE_MOON_ROCK_START+6].setPosition(0,0,0,0);
	m_obj[TABLE_MOON_ROCK_START+7].setPosition(0,0,0,0);
	for(int i = TABLE_MOON_ROCK_START; i< TABLE_MOON_ROCK_START+8; i++)
	{
		m_obj[i+8].position = m_obj[i].position;
		m_obj[i+8].position.x *= -1;
	}

	// blue lunar module
	m_obj[TABLE_BLUE_LUNAR_MODULE_START].setPosition(   -350,  960,   0, 0);
	m_obj[TABLE_BLUE_LUNAR_MODULE_START+1].setPosition( -350,  960, 100, 0);
	m_obj[TABLE_BLUE_LUNAR_MODULE_START+2].setPosition( -350,  960, 200, 0);
	m_obj[TABLE_BLUE_LUNAR_MODULE_START+3].setPosition( -350,  960, 300, 0);
	m_obj[TABLE_BLUE_LUNAR_MODULE_START+4].setPosition( -550,  800,   0, 0);
	m_obj[TABLE_BLUE_LUNAR_MODULE_START+5].setPosition(-1300,  400,   0, 0);
	m_obj[TABLE_BLUE_LUNAR_MODULE_START+6].setPosition( -700, -850,   0, 0);

	// yellow lunar module (symetrie)
	for(int i = 0; i< NUM_BLUE_LUNAR_MODULE; i++)
	{
		m_obj[TABLE_YELLOW_LUNAR_MODULE_START+i].position = m_obj[TABLE_BLUE_LUNAR_MODULE_START+i].position;
		m_obj[TABLE_YELLOW_LUNAR_MODULE_START+i].position.x *= -1;
	}

	m_obj[TABLE_BI_LUNAR_MODULE_START].setPosition(   -1460, -350,   0, 0);
	m_obj[TABLE_BI_LUNAR_MODULE_START+1].setPosition( -1460, -350, 100, 0);
	m_obj[TABLE_BI_LUNAR_MODULE_START+2].setPosition( -1460, -350, 200, 0);
	m_obj[TABLE_BI_LUNAR_MODULE_START+3].setPosition( -1460, -350, 300, 0);
	m_obj[TABLE_BI_LUNAR_MODULE_START+4].setPosition(  -500,  400,   0, 0);
	m_obj[TABLE_BI_LUNAR_MODULE_START+5].setPosition( -1000, -100,   0, 0);
	m_obj[TABLE_BI_LUNAR_MODULE_START+6].setPosition(  -600, -400,   0, 0);

	for(int i = TABLE_BI_LUNAR_MODULE_START; i< TABLE_BI_LUNAR_MODULE_START+7; i++)
	{
		m_obj[i+7].position = m_obj[i].position;
		m_obj[i+7].position.x *= -1;
	}
}

bool Table3d::initQemuObjects()
{
	int objFlags = OBJECT_MOBILE;// | OBJECT_SEEN_BY_HOKUYO;
	for(int j = 0; j < m_robotCount; j++ )
	{
		if( m_robot[j].m_simulation )
		{
			for(unsigned int i = TABLE_OBJ_TITANIUM_ORE_START; i < TABLE_OBJ_TITANIUM_ORE_START + NUM_TITANIUM_ORE; i++)
			{
				m_robot[j].m_qemu.add_object(objFlags, titaniumOre, &m_qemuObjId[i]);
			}

			for(unsigned int i = TABLE_MOON_ROCK_START; i < TABLE_MOON_ROCK_START + NUM_MOON_ROCK; i++)
			{
				m_robot[j].m_qemu.add_object(objFlags, moonRock, &m_qemuObjId[i]);
			}

			for(unsigned int i = TABLE_BLUE_LUNAR_MODULE_START; i < TABLE_BLUE_LUNAR_MODULE_START + NUM_BLUE_LUNAR_MODULE; i++)
			{
				m_robot[j].m_qemu.add_object(objFlags, lunarModule, &m_qemuObjId[i]);
			}

			for(unsigned int i = TABLE_YELLOW_LUNAR_MODULE_START; i < TABLE_YELLOW_LUNAR_MODULE_START + NUM_YELLOW_LUNAR_MODULE; i++)
			{
				m_robot[j].m_qemu.add_object(objFlags, lunarModule, &m_qemuObjId[i]);
			}

			for(unsigned int i = TABLE_BI_LUNAR_MODULE_START; i < TABLE_BI_LUNAR_MODULE_START + NUM_BI_LUNAR_MODULE; i++)
			{
				m_robot[j].m_qemu.add_object(objFlags, lunarModule, &m_qemuObjId[i]);
			}

			for(unsigned int i = 0; i < TABLE_OBJ_MAX; i++)
			{
				m_robot[j].m_qemu.move_object(m_qemuObjId[i], Vect2(), VectPlan(m_obj[i].position.x, m_obj[i].position.y, m_obj[i].theta));
			}
		}
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
			for(int j = 0; j < m_robotCount; j++ )
			{
				if( m_robot[j].m_simulation )
				{
					m_robot[j].m_qemu.move_object(m_qemuObjId[i], Vect2(m_obj[i].position.x, m_obj[i].position.y), VectPlan(dx, dy, 0));
				}
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
