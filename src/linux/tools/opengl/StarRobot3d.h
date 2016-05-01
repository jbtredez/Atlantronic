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
		float leftDoorTheta;
		float rightDoorTheta;

	protected:
		void drawMobileBase();
		void drawFishWings();
		void drawDoors();

		GlObject m_mobileBase;
		GlObject m_leftFishWing;
		GlObject m_rightFishWing;
		GlObject m_fishRemover;
		GlObject m_leftDoor;
		GlObject m_rightDoor;
		MainShader* m_shader;
};

#endif
