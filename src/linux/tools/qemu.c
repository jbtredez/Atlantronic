#include "qemu.h"
#include <stdlib.h>
#include <signal.h>
#include <sys/stat.h>
#include "linux/tools/cli.h"
#include <math.h>

#define  MIN(a, b)      (((a) < (b)) ? (a) : (b))

#define EVENT_CLOCK_FACTOR         1
#define EVENT_NEW_OBJECT           2
#define EVENT_MOVE_OBJECT          3

struct atlantronic_model_tx_event
{
	uint32_t type;        //!< type
	union
	{
		uint8_t data[256];    //!< données
		uint32_t data32[64];  //!< données
	};
};

int qemu_init(struct qemu* qemu, const char* prog_name, int gdb_port)
{
	pid_t pid = getpid();

	snprintf(qemu->file_qemu_read, sizeof(qemu->file_qemu_read), "/tmp/qemu-%i.out", pid);
	snprintf(qemu->file_qemu_write, sizeof(qemu->file_qemu_write), "/tmp/qemu-%i.in", pid);
	snprintf(qemu->file_foo_read, sizeof(qemu->file_foo_read), "/tmp/foo-%i.out", pid);
	snprintf(qemu->file_foo_write, sizeof(qemu->file_foo_write), "/tmp/foo-%i.in", pid);

	mkfifo(qemu->file_qemu_read, 0666);
	mkfifo(qemu->file_qemu_write, 0666);
	mkfifo(qemu->file_foo_read, 0666);
	mkfifo(qemu->file_foo_write, 0666);

	qemu->pid = fork();

	char pipe_usb[64];
	char pipe_model[64];
	snprintf(pipe_usb, sizeof(pipe_usb), "pipe,id=foo_usb,path=/tmp/foo-%i", pid);
	snprintf(pipe_model, sizeof(pipe_model), "pipe,id=foo_model,path=/tmp/qemu-%i", pid);

	if(qemu->pid == 0)
	{
		char* arg[15];
		char buf_tcp[64];

		arg[0] = (char*) "qemu/arm-softmmu/qemu-system-arm";
		arg[1] = (char*) "-M";
		arg[2] = (char*) "atlantronic-foo";
		arg[3] = (char*)"-nodefaults";
		arg[4] = (char*)"-nographic";
		arg[5] = (char*) "-chardev";
		arg[6] = (char*) pipe_usb;
		arg[7] = (char*) "-chardev";
		arg[8] = (char*) pipe_model;
		arg[9] = (char*) "-kernel";
		arg[10] = (char*) prog_name;
		if(gdb_port)
		{
			arg[11] = (char*) "-S";
			arg[12] = (char*) "-gdb";
			snprintf(buf_tcp, sizeof(buf_tcp), "tcp::%i", gdb_port);
			arg[13] = buf_tcp;
			arg[14] = NULL;
		}
		else
		{
			arg[11] = NULL;
		}

		execv(arg[0], arg);
		perror("execv");
		exit(-1);
	}

	if(qemu->pid < 0)
	{
		perror("fork");
		return -1;
	}

	com_init(&qemu->com, qemu->file_qemu_read, qemu->file_qemu_write);

	com_open_block(&qemu->com);

	return 0;
}

void qemu_destroy(struct qemu* qemu)
{
	// TODO un peu bourrin, faire mieux
	if(qemu->pid > 0)
	{
		kill(qemu->pid, SIGILL);
	}

	com_close(&qemu->com);
	com_destroy(&qemu->com);

	unlink(qemu->file_qemu_read);
	unlink(qemu->file_qemu_write);
	unlink(qemu->file_foo_read);
	unlink(qemu->file_foo_write);
}

int qemu_set_clock_factor(struct qemu* qemu, unsigned int factor)
{
	struct atlantronic_model_tx_event event;

	event.type = EVENT_CLOCK_FACTOR;
	event.data32[0] = factor;

	return com_write(&qemu->com, (void*) &event, sizeof(event));
}

int qemu_add_object(struct qemu* qemu, const struct polyline polyline)
{
	struct atlantronic_model_tx_event event;
	unsigned int i = 0;

	event.type = EVENT_NEW_OBJECT;

	event.data[0] = MIN((unsigned int)polyline.size, (sizeof(event.data) - 1)/8);
	float* f = (float*)(event.data+1);
	for(i = 0; i < event.data[0]; i++ )
	{
		f[0] = polyline.pt[i].x / 65536.0f;
		f[1] = polyline.pt[i].y / 65536.0f;
		f += 2;
	}

	return com_write(&qemu->com, (void*) &event, sizeof(event));
}

int qemu_move_object(struct qemu* qemu, int id, struct fx_vect2 origin, struct fx_vect_pos delta)
{
	struct atlantronic_model_tx_event event;

	event.type = EVENT_MOVE_OBJECT;

	event.data[0] = id;
	float* f = (float*)(event.data+1);
	f[0] = origin.x / 65536.0f;
	f[1] = origin.y / 65536.0f;
	f[2] = delta.x / 65536.0f;
	f[3] = delta.y / 65536.0f;
	f[4] = delta.alpha / ((float)(1<<26)) * 2 * M_PI;

	return com_write(&qemu->com, (void*) &event, sizeof(event));
}
