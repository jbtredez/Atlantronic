#ifndef ROBOT_H
#define ROBOT_H

#include "qemu.h"
#include "robot_interface.h"
#include "linux/usb_interface_common/com/com.h"

class Robot
{
	public:
		Robot();

		int init(const char* name,
				bool simu, const char* path, const char* progStm, int gdb_port,
				const char* ip,
				bool xbee,
				bool serverTcp,
				const char* file_stm,
				void (*_callback)(void*), void* arg);

		void destroy();

		RobotInterface m_robotItf;    //!< interface de com avec le robot
		Qemu m_qemu;                  //!< interface de com avec la simulation du robot (si simule)
		bool m_simulation;            //!< indique si l'interface de simulation est initialisee

	protected:
		Com* m_com;
};


#endif
