#ifndef ROBOT_3D_H
#define ROBOT_3D_H

#include "linux/tools/opengl/object3d.h"

class Robot3d
{
	public:
		bool init();
		void draw();

		float rightWingTheta;
		float leftWingTheta;

	protected:
		void drawWing(bool right);

		Object3d mobileBase;
		Object3d wing;
};

#endif
