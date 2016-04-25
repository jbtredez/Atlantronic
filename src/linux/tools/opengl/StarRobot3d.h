#ifndef STAR_ROBOT_3D_H
#define STAR_ROBOT_3D_H

#include "linux/tools/opengl/object3d.h"

class StarRobot3d
{
	public:
		bool init(MainShader* shader);
		void draw();

		float leftFishWingTheta;
		float rightFishWingTheta;
		float leftFishRemoverTheta;
		float rightFishRemoverTheta;

	protected:
		void drawMobileBase();
		void drawFishWings();

		GlObject m_mobileBase;
		GlObject m_leftFishWing;
		GlObject m_rightFishWing;
		GlObject m_fishRemover;
		MainShader* m_shader;
};

#endif
