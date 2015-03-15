#ifndef ROBOT_3D_H
#define ROBOT_3D_H

#include "linux/tools/opengl/object3d.h"

class Robot3d
{
	public:
		bool init();
		void draw();

		float rightClapArmTheta;
		float leftClapArmTheta;

	protected:
		void drawClapArm(bool right);

		Object3d mobileBase;
		Object3d clapArm;
};

#endif
