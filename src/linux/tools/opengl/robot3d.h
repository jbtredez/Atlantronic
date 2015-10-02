#ifndef ROBOT_3D_H
#define ROBOT_3D_H

#include "linux/tools/opengl/object3d.h"

class Robot3d
{
	public:
		bool init(MainShader* shader);
		void draw();

		float rightWingTheta;
		float leftWingTheta;
		float elevatorHeight;
		float highFingerTheta;
		float lowFingerTheta;
		float rightCarpetTheta;
		float leftCarpetTheta;

	protected:
		void drawMobileBase();
		void drawElevator();
		void drawWing(bool right);
		void drawCarpet(bool right);

		GlObject m_mobileBase;
		GlObject m_wing;
		GlObject m_elevator;
		GlObject m_finger;
		GlObject m_carpet;
		MainShader* m_shader;
};

#endif
