#ifndef LIDAR_LITE_H
#define LIDAR_LITE_H

#include <stdint.h>

class LidarLite
{
	public:
		LidarLite();

		void update();

	protected:
		int m_distance;    //!< distance en mm
};


#endif
