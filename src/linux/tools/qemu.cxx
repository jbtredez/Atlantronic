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

int qemu::init(const char* prog_name, int gdb_port)
{
	pid_t current_pid = getpid();

	snprintf(file_qemu_read, sizeof(file_qemu_read), "/tmp/qemu-%i.out", current_pid);
	snprintf(file_qemu_write, sizeof(file_qemu_write), "/tmp/qemu-%i.in", current_pid);
	snprintf(file_foo_read, sizeof(file_foo_read), "/tmp/foo-%i.out", current_pid);
	snprintf(file_foo_write, sizeof(file_foo_write), "/tmp/foo-%i.in", current_pid);

	mkfifo(file_qemu_read, 0666);
	mkfifo(file_qemu_write, 0666);
	mkfifo(file_foo_read, 0666);
	mkfifo(file_foo_write, 0666);

	pid = fork();

	char pipe_usb[64];
	char pipe_model[64];
	snprintf(pipe_usb, sizeof(pipe_usb), "pipe,id=foo_usb,path=/tmp/foo-%i", current_pid);
	snprintf(pipe_model, sizeof(pipe_model), "pipe,id=foo_model,path=/tmp/qemu-%i", current_pid);

	if(pid == 0)
	{
		char* arg[15];
		char buf_tcp[64];

		arg[0] = (char*) "qemu/arm-softmmu/qemu-system-arm";
		arg[1] = (char*) "-M";
		arg[2] = (char*) "atlantronic";
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

	if(pid < 0)
	{
		perror("fork");
		return -1;
	}

	com.init(file_qemu_read, file_qemu_write);

	com.open_block();

	return 0;
}

void qemu::destroy()
{
	// TODO un peu bourrin, faire mieux
	if(pid > 0)
	{
		kill(pid, SIGILL);
	}

	com.close();
	com.destroy();

	unlink(file_qemu_read);
	unlink(file_qemu_write);
	unlink(file_foo_read);
	unlink(file_foo_write);
}

int qemu::set_clock_factor(unsigned int factor)
{
	struct atlantronic_model_tx_event event;

	event.type = EVENT_CLOCK_FACTOR;
	event.data32[0] = factor;

	return com.write((void*) &event, sizeof(event));
}

int qemu::add_object(const struct polyline polyline)
{
	struct atlantronic_model_tx_event event;
	unsigned int i = 0;

	event.type = EVENT_NEW_OBJECT;

	event.data[0] = MIN((unsigned int)polyline.size, (sizeof(event.data) - 1)/8);
	float* f = (float*)(event.data+1);
	for(i = 0; i < event.data[0]; i++ )
	{
		f[0] = polyline.pt[i].x;
		f[1] = polyline.pt[i].y;
		f += 2;
	}

	return com.write((void*) &event, sizeof(event));
}

int qemu::move_object(int id, struct vect2 origin, VectPlan delta)
{
	struct atlantronic_model_tx_event event;

	event.type = EVENT_MOVE_OBJECT;

	event.data[0] = id;
	float* f = (float*)(event.data+1);
	f[0] = origin.x;
	f[1] = origin.y;
	f[2] = delta.x;
	f[3] = delta.y;
	f[4] = delta.theta;

	return com.write((void*) &event, sizeof(event));
}
