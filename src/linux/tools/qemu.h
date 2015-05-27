#ifndef QEMU_H
#define QEMU_H

#include "linux/tools/com/com_usb.h"
#include "kernel/math/polyline.h"
#include "kernel/math/vect_plan.h"

enum ObjectType
{
	OBJECT_FLOOR_FOOTPRINT,
	OBJECT_MOBILE_FLOOR_FOOTPRINT,
	OBJECT_BEACON_FOOTPRINT,
};

class Qemu
{
	public:
		Qemu();

		char m_file_qemu_read[64];
		char m_file_qemu_write[64];
		char m_file_board_read[64];
		char m_file_board_write[64];

		int init(const char* qemu_path, const char* prog_name, int gdb_port);
		void reboot();
		void destroy();
		int set_clock_factor(unsigned int factor, unsigned int icount);
		int add_object(ObjectType type, const struct polyline polyline);
		int move_object(int id, Vect2 origin, VectPlan delta);
		int manage_canopen_connexion(int nodeId, bool connected);
		int set_io(uint32_t id, bool val);
		int setPosition(VectPlan pos);

	protected:
		void startQemu();
		void stopQemu();

		unsigned int m_clock_factor;
		char m_qemu_path[2048];
		char m_prog_name[2048];
		int m_gdb_port;
		Com* m_com; //!< communication avec qemu
		pid_t m_pid; //!< pid de qemu
};

#endif
