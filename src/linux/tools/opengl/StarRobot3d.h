#ifndef STAR_ROBOT_3D_H
#define STAR_ROBOT_3D_H

#include "linux/tools/opengl/object3d.h"

class StarRobot3d
{
	public:
		bool init(MainShader* shader);
		void draw();

	protected:
		void drawMobileBase();

		GlObject m_mobileBase;
		MainShader* m_shader;
};

#endif
