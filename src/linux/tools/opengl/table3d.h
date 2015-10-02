#ifndef TABLE_3D_H
#define TABLE_3D_H

#include "linux/tools/opengl/object3d.h"

#define NUM_SAND_CONE               9
#define NUM_SAND_CUBE              40
#define NUM_SAND_CYLINDER          20
#define NUM_WHITE_SEA_SHELL         6
#define NUM_GREEN_SEA_SHELL         5
#define NUM_PURPLE_SEA_SHELL        5
#define NUM_GREEN_FISH              4
#define NUM_PURPLE_FISH             4

enum
{
	TABLE_OBJ_SAND_CONE_START = 0,
	TABLE_OBJ_SAND_CUBE_START = TABLE_OBJ_SAND_CONE_START + NUM_SAND_CONE,
	TABLE_OBJ_SAND_CYLINDER_START = TABLE_OBJ_SAND_CUBE_START + NUM_SAND_CUBE,
	TABLE_OBJ_WHITE_SEA_SHELL_START = TABLE_OBJ_SAND_CYLINDER_START + NUM_SAND_CYLINDER,
	TABLE_OBJ_GREEN_SEA_SHELL_START = TABLE_OBJ_WHITE_SEA_SHELL_START + NUM_WHITE_SEA_SHELL,
	TABLE_OBJ_PURPLE_SEA_SHELL_START = TABLE_OBJ_GREEN_SEA_SHELL_START + NUM_GREEN_SEA_SHELL,
	TABLE_OBJ_GREEN_FISH_START = TABLE_OBJ_PURPLE_SEA_SHELL_START + NUM_PURPLE_SEA_SHELL,
	TABLE_OBJ_PURPLE_FISH_START = TABLE_OBJ_GREEN_FISH_START + NUM_GREEN_FISH,
	TABLE_OBJ_MAX = TABLE_OBJ_PURPLE_FISH_START + NUM_PURPLE_FISH,
};

class Table3d
{
	public:
		bool init(int glSelectBaseId, MainShader* shader);
		void draw();

		void unselectAll();
		void select(unsigned int id);
		void moveSelected(float dx, float dy);

		bool showTable;
		bool showElements;
		void initElementPosition(int configuration);

	protected:
		GlObject m_glTable;
		GlObject m_glSandCone;
		GlObject m_glSandCube;
		GlObject m_glSandCylinder;
		GlObject m_glWhiteSeaShell;
		GlObject m_glGreenSeaShell;
		GlObject m_glPurpleSeaShell;
		GlObject m_glGreenFish;
		GlObject m_glPurpleFish;

		Object3d m_obj[TABLE_OBJ_MAX];
		MainShader* m_shader;
};

#endif
