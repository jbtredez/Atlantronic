#ifndef QEMU_H
#define QEMU_H

#include "linux/tools/com.h"
#include "foo/table.h"
#include "kernel/vect_pos.h"

struct qemu
{
	char file_qemu_read[64];
	char file_qemu_write[64];
	char file_foo_read[64];
	char file_foo_write[64];
	struct com com; //!< communication avec qemu
	pid_t pid; //!< pid de qemu
};

int qemu_init(struct qemu* qemu, const char* prog_name, int gdb_port);

void qemu_destroy(struct qemu* qemu);

int qemu_set_clock_factor(struct qemu* qemu, unsigned int factor);

int qemu_add_object(struct qemu* qemu, const struct polyline polyline);

int qemu_move_object(struct qemu* qemu, int id, struct fx_vect2 origin, struct fx_vect_pos delta);

#endif
