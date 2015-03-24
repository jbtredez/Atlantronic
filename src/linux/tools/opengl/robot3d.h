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
		float elevatorHeight;
		float highFingerTheta;
		float lowFingerTheta;

	protected:
		void drawMobileBase();
		void drawElevator();
		void drawWing(bool right);

		Object3d m_mobileBase;
		Object3d m_wing;
		Object3d m_elevator;
		Object3d m_finger;
};

#endif
