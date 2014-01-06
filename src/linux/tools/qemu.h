#ifndef QEMU_H
#define QEMU_H

#include "linux/tools/com.h"
#include "kernel/math/polyline.h"
#include "kernel/math/vect_plan.h"

class qemu
{
	public:
		char file_qemu_read[64];
		char file_qemu_write[64];
		char file_foo_read[64];
		char file_foo_write[64];
		Com com; //!< communication avec qemu
		pid_t pid; //!< pid de qemu

		int init(const char* prog_name, int gdb_port);
		void destroy();
		int set_clock_factor(unsigned int factor);
		int add_object(const struct polyline polyline);
		int move_object(int id, struct vect2 origin, VectPlan delta);
};

#endif
