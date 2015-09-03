#ifndef TABLE_3D_H
#define TABLE_3D_H

#include "linux/tools/opengl/object3d.h"

class Table3d
{
	public:
		bool init(int glSelectFeetName[16], int glSelectGlassName[5], MainShader* shader);
		void draw();

		void unselectAll();
		void selectFeet(unsigned int id);
		void selectGlass(unsigned int id);
		void moveSelected(float dx, float dy);

		bool showTable;
		bool showElements;

	protected:
		void drawDispenser(float x);
		void drawClap(float x, bool yellow);
		void drawFeets();
		void drawGlass();

		Object3d table;
		Object3d dispenser;
		Object3d clapYellow;
		Object3d clapGreen;
		Object3d feetYellow;
		Object3d feetGreen;
		Object3d glass;
		aiVector3D feetPosition[16];
		int glSelectFeetName[16];
		bool feetSelected[16];
		aiVector3D glassPosition[5];
		int glSelectGlassName[5];
		bool glassSelected[5];
		MainShader* m_shader;
};

#endif
