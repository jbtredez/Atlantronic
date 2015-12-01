#ifndef QEMU_H
#define QEMU_H

#include "linux/tools/com/com_usb.h"
#include "kernel/math/polyline.h"
#include "kernel/math/VectPlan.h"

#define OBJECT_MOBILE                1
#define OBJECT_SEEN_BY_HOKUYO        2
#define OBJECT_SEEN_BY_OMRON         4

class Qemu
{
	public:
		Qemu();

		char m_file_qemu_read[64];
		char m_file_qemu_write[64];
		char m_file_board_read[64];
		char m_file_board_write[64];

		int init(const char* name, const char* qemu_path, const char* prog_name, int gdb_port);
		void reboot();
		void destroy();
		int add_object(int flags, const struct polyline polyline, int* objectId = NULL);
		int move_object(int id, Vect2 origin, VectPlan delta);
		int manage_canopen_connexion(int nodeId, bool connected);
		int setIo(uint32_t id, bool val);
		int setPosition(VectPlan pos);

	protected:
		void startQemu();
		void stopQemu();

		char m_qemu_path[2048];
		char m_prog_name[2048];
		int m_gdb_port;
		Com* m_com; //!< communication avec qemu
		pid_t m_pid; //!< pid de qemu
		int m_lastObjectId;
		char m_name[256];
};

#endif
