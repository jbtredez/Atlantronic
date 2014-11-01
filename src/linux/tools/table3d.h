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

		Object3d table;
		Object3d dispenser;
		Object3d clapYellow;
		Object3d clapGreen;
};

#endif
