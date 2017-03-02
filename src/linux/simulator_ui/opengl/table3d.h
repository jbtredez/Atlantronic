#ifndef TABLE_3D_H
#define TABLE_3D_H

#include "linux/simulator_ui/opengl/object3d.h"
#include "linux/usb_interface_common/Robot.h"

#define NUM_TITANIUM_ORE           40
#define NUM_MOON_ROCK              16
#define NUM_BLUE_LUNAR_MODULE       7
#define NUM_YELLOW_LUNAR_MODULE     7
#define NUM_BI_LUNAR_MODULE        14
enum
{
	TABLE_OBJ_TITANIUM_ORE_START = 0,
	TABLE_MOON_ROCK_START = TABLE_OBJ_TITANIUM_ORE_START + NUM_TITANIUM_ORE,
	TABLE_BLUE_LUNAR_MODULE_START = TABLE_MOON_ROCK_START + NUM_MOON_ROCK,
	TABLE_YELLOW_LUNAR_MODULE_START = TABLE_BLUE_LUNAR_MODULE_START + NUM_BLUE_LUNAR_MODULE,
	TABLE_BI_LUNAR_MODULE_START = TABLE_YELLOW_LUNAR_MODULE_START + NUM_YELLOW_LUNAR_MODULE,
	TABLE_OBJ_MAX = TABLE_BI_LUNAR_MODULE_START + NUM_BI_LUNAR_MODULE,
};

class Table3d
{
	public:
		bool init(int glSelectBaseId, MainShader* shader, Robot* robot, int robotCount);
		bool initQemuObjects();
		void draw();

		void unselectAll();
		void select(unsigned int id);
		void moveSelected(float dx, float dy);

		bool showTable;
		bool showElements;
		void initElementPosition(int configuration);

	protected:
		GlObject m_glTable;
		GlObject m_titaniumOre;
		GlObject m_moonRock;
		GlObject m_blueLunarModule;
		GlObject m_yellowLunarModule;
		GlObject m_biLunarModule;

		Object3d m_obj[TABLE_OBJ_MAX];
		int m_qemuObjId[TABLE_OBJ_MAX];

		MainShader* m_shader;
		Robot* m_robot;
		int m_robotCount;
};

#endif
