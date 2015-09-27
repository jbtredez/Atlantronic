#ifndef TABLE_SCENE_H
#define TABLE_SCENE_H

#include "linux/tools/opengl/table3d.h"
#include "linux/tools/opengl/robot3d.h"
#include "linux/tools/opengl/gl_font.h"
#include "linux/tools/graphique.h"
#include "linux/tools/robot_interface.h"
#include "disco/table.h"
#include "middleware/motion/graph.h"
#include "kernel/robot_parameters.h"
#include "kernel/math/vect_plan.h"
#include "main_shader.h"

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
	SUBGRAPH_TABLE_ROBOT = 0,
	SUBGRAPH_TABLE_OPPONENT_ROBOT,
	SUBGRAPH_TABLE_STATIC_ELM,
	SUBGRAPH_TABLE_TABLE3D,
	SUBGRAPH_TABLE_ELM3D,
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
		bool init(GlFont* glfont, RobotInterface* robotItf, MainShader* shader);
		void draw(Graphique* graph);
		void printInfos(Graphique* graph);
		VectPlan projectMouseOnTable(int x, int y);
		void rotateView(float dx, float dy);
		void translateView(float dx, float dy, float dz);

		void mouseSelect(int x, int y, Graphique* graph);
		void mouseMoveSelection(int x, int y);
		inline VectPlan getOpponentPosition()
		{
			return m_opponentRobotPos;
		}

	protected:
		void drawOpponentRobot(Graphique* graph);
		void drawRobot(Graphique* graph);

		RobotInterface* m_robotItf;
		Table3d m_table3d;
		Robot3d m_robot3d;
		Object3d m_opponentRobot3d;
		VectPlan m_opponentRobotPos;
		GlFont* m_glfont;
		glm::mat4 m_tableModelview;
		glm::mat4 m_tableProjection;
		MatrixHomogeneous m_viewMatrix;
		int m_mouseX;
		int m_mouseY;
		MainShader* m_shader;
		Object3dBasic m_robot2d;
		Object3dBasic m_table2d[TABLE_OBJ_SIZE];
		Object3dBasic m_graphPointObject;
};

#endif
