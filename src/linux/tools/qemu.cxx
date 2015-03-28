#include "qemu.h"
#include <stdlib.h>
#include <signal.h>
#include <sys/stat.h>
#include "linux/tools/cli.h"
#include <math.h>

#define  MIN(a, b)      (((a) < (b)) ? (a) : (b))

enum
{
	EVENT_CLOCK_FACTOR = 1,
	EVENT_NEW_OBJECT,
	EVENT_MOVE_OBJECT,
	EVENT_MANAGE_CAN_MOTOR,
	EVENT_SET_IO,
	EVENT_SET_POSITION,
};

enum
{
	EVENT_MANAGE_CAN_MOTOR_CONNECT,
	EVENT_MANAGE_CAN_MOTOR_DISCONNECT,
};

struct atlantronic_model_tx_event
{
	uint32_t type;        //!< type
	union
	{
		uint8_t data[256];    //!< données
		uint32_t data32[64];  //!< données
	};
};

int Qemu::init(const char* qemu_path, const char* prog_name, int gdb_port)
{
	pid_t current_pid = getpid();

	snprintf(file_qemu_read, sizeof(file_qemu_read), "/tmp/qemu-%i.out", current_pid);
	snprintf(file_qemu_write, sizeof(file_qemu_write), "/tmp/qemu-%i.in", current_pid);
	snprintf(file_board_read, sizeof(file_board_read), "/tmp/carte-%i.out", current_pid);
	snprintf(file_board_write, sizeof(file_board_write), "/tmp/carte-%i.in", current_pid);

	mkfifo(file_qemu_read, 0666);
	mkfifo(file_qemu_write, 0666);
	mkfifo(file_board_read, 0666);
	mkfifo(file_board_write, 0666);

	pid = fork();

	char pipe_usb[64];
	char pipe_model[64];
	snprintf(pipe_usb, sizeof(pipe_usb), "pipe,id=foo_usb,path=/tmp/carte-%i", current_pid);
	snprintf(pipe_model, sizeof(pipe_model), "pipe,id=foo_model,path=/tmp/qemu-%i", current_pid);

	if(pid == 0)
	{
		char* arg[15];
		char buf_tcp[64];

		arg[0] = (char*) qemu_path;
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

	com = new ComUsb(file_qemu_read, file_qemu_write);

	com->open_block();

	return 0;
}

void Qemu::destroy()
{
	// TODO un peu bourrin, faire mieux
	if(pid > 0)
	{
		kill(pid, SIGILL);
	}

	if( com )
	{
		com->close();
	}

	unlink(file_qemu_read);
	unlink(file_qemu_write);
	unlink(file_board_read);
	unlink(file_board_write);
}

int Qemu::set_clock_factor(unsigned int factor, unsigned int icount)
{
	struct atlantronic_model_tx_event event;

	event.type = EVENT_CLOCK_FACTOR;
	event.data32[0] = factor;
	event.data32[1] = icount;

	return com->write((void*) &event, sizeof(event));
}

int Qemu::add_object(const struct polyline polyline)
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

	return com->write((void*) &event, sizeof(event));
}

int Qemu::move_object(int id, Vect2 origin, VectPlan delta)
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

	return com->write((void*) &event, sizeof(event));
}

//! @param nodeId : nodeId ou 0 pour tous
int Qemu::manage_canopen_connexion(int nodeId, bool connected)
{
	struct atlantronic_model_tx_event event;

	event.type = EVENT_MANAGE_CAN_MOTOR;
	event.data32[0] = nodeId;
	if(connected)
	{
		event.data32[1] = EVENT_MANAGE_CAN_MOTOR_CONNECT;
	}
	else
	{
		event.data32[1] = EVENT_MANAGE_CAN_MOTOR_DISCONNECT;
	}

	return com->write((void*) &event, sizeof(event));
}

int Qemu::set_io(uint32_t id, bool val)
{
	struct atlantronic_model_tx_event event;
	event.type = EVENT_SET_IO;
	event.data32[0] = id;
	event.data32[1] = val?1:0;

	return com->write((void*) &event, sizeof(event));
}

int Qemu::setPosition(VectPlan pos)
{
	struct atlantronic_model_tx_event event;
	event.type = EVENT_SET_POSITION;
	memcpy(&event.data32[0], &pos, sizeof(pos));

	return com->write((void*) &event, sizeof(event));
}
