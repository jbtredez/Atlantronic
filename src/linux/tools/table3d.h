#ifndef TABLE_3D_H
#define TABLE_3D_H

#include "linux/tools/object3d.h"

class Table3d
{
	public:
		bool init();
		void draw();

	protected:
		void drawDispenser(float x);
		void drawClap(float x, bool yellow);
		void drawFeets();
		void drawGlass();

		Object3d table;
		Object3d dispenser;
		Object3d clapYellow;
		Object3d clapGreen;
		Object3d feet;
		Object3d glass;
		aiVector3D feetPosition[16];
		aiVector3D glassPosition[5];
};

#endif
