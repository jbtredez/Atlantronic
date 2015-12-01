#ifndef TABLE_SCENE_H
#define TABLE_SCENE_H

#include "linux/tools/opengl/table3d.h"
#include "linux/tools/opengl/robot3d.h"
#include "linux/tools/opengl/gl_font.h"
#include "linux/tools/graphique.h"
#include "linux/tools/Robot.h"
#include "disco/table.h"
#include "middleware/trajectory/Graph.h"
#include "disco/robot_parameters.h"
#include "kernel/math/VectPlan.h"
#include "main_shader.h"

enum
{
	GL_NAME_NONE = 0,
	GL_NAME_ROBOT,
	GL_NAME_OPPONENT,
	GL_NAME_TABLE_ELEMENTS_0,
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
		bool init(GlFont* glfont, Robot* robot, int robotCount, MainShader* shader);
		bool initQemuObjects();

		void draw(Graphique* graph);
		void printInfos(Graphique* graph);
		VectPlan projectMouseOnTable(int x, int y);
		void rotateView(float dx, float dy);
		void translateView(float dx, float dy, float dz);

		void setColor(int color);

		void mouseSelect(int x, int y, Graphique* graph);
		void mouseMoveSelection(int x, int y);
		inline VectPlan getOpponentPosition()
		{
			return m_opponentRobotPos;
		}

		inline void selectConfiguration(int id)
		{
			m_table3d.initElementPosition(id);
		}
	protected:
		void drawOpponentRobot(Graphique* graph);
		void drawRobot(Graphique* graph);

		Robot* m_robot;
		int m_robotCount;
		int m_qemuObjectOpponentId;
		Table3d m_table3d;
		Robot3d m_mainRobot3d;
		Robot3d m_pmiRobot3d;
		GlObject m_opponentRobot3d;
		VectPlan m_opponentRobotPos;
		GlFont* m_glfont;
		glm::mat4 m_tableModelview;
		glm::mat4 m_tableProjection;
		MatrixHomogeneous m_viewMatrix;
		int m_mouseX;
		int m_mouseY;
		MainShader* m_shader;
		GlObjectBasic m_robot2d;
		GlObjectBasic m_table2d[TABLE_OBJ_SIZE];
		GlObjectBasic m_graphPointObject;
};

#endif
