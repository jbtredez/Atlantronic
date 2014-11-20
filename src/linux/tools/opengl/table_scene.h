#ifndef TABLE_SCENE_H
#define TABLE_SCENE_H

#include "linux/tools/opengl/table3d.h"
#include "linux/tools/opengl/gl_font.h"
#include "linux/tools/graphique.h"
#include "linux/tools/robot_interface.h"
#include "kernel/table.h"
#include "kernel/motion/graph.h"
#include "kernel/robot_parameters.h"
#include "kernel/math/vect_plan.h"

enum
{
	GL_NAME_NONE = 0,
	GL_NAME_ROBOT,
	GL_NAME_OPPONENT,
	GL_NAME_FEET_0,
	GL_NAME_FEET_1,
	GL_NAME_FEET_2,
	GL_NAME_FEET_3,
	GL_NAME_FEET_4,
	GL_NAME_FEET_5,
	GL_NAME_FEET_6,
	GL_NAME_FEET_7,
	GL_NAME_FEET_8,
	GL_NAME_FEET_9,
	GL_NAME_FEET_10,
	GL_NAME_FEET_11,
	GL_NAME_FEET_12,
	GL_NAME_FEET_13,
	GL_NAME_FEET_14,
	GL_NAME_FEET_15,
	GL_NAME_GLASS_0,
	GL_NAME_GLASS_1,
	GL_NAME_GLASS_2,
	GL_NAME_GLASS_3,
	GL_NAME_GLASS_4,
};

enum
{
	SUBGRAPH_TABLE_POS_ROBOT = 0,
	SUBGRAPH_TABLE_STATIC_ELM,
	SUBGRAPH_TABLE_TABLE,
	SUBGRAPH_TABLE_HOKUYO1,
	SUBGRAPH_TABLE_HOKUYO2,
	SUBGRAPH_TABLE_HOKUYO1_SEG,
	SUBGRAPH_TABLE_POS_CONS,
	SUBGRAPH_TABLE_POS_MES,
	SUBGRAPH_TABLE_GRAPH,
	SUBGRAPH_TABLE_GRAPH_LINK,
	SUBGRAPH_TABLE_NUM,
};

class TableScene
{
	public:
		TableScene();
		bool init(GlFont* glfont, RobotInterface* robotItf);
		void draw(GLenum mode, Graphique* graph);
		VectPlan projectMouseOnTable(int x, int y);
		void rotateView(float dx, float dy);
		void translateView(float dx, float dy, float dz);

		void mouseSelect(int x, int y, Graphique* graph);
		void mouseMoveSelection(int x, int y);
		inline VectPlan getOpponentPosition()
		{
			return opponentRobotPos;
		}

	protected:
		void draw_plus(float x, float y, float rx, float ry);
		void drawOpponentRobot();
		void drawRobot(Graphique* graph);
		void processHits(GLint hits, GLuint buffer[]);

		RobotInterface* robotItf;
		Table3d table3d;
		Object3d robot3d;
		Object3d opponentRobot3d;
		VectPlan opponentRobotPos;
		GlFont* glfont;
		float tableThetaZ;
		GLdouble tableModelview[16];
		GLdouble tableProjection[16];
		MatrixHomogeneous viewMatrix;
		int mouseX;
		int mouseY;
};

#endif
