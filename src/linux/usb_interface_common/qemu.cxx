#include "qemu.h"
#include <stdlib.h>
#include <signal.h>
#include <sys/stat.h>
#include "linux/usb_interface_common/cli.h"
#include <math.h>

#define  MIN(a, b)      (((a) < (b)) ? (a) : (b))

Qemu::Qemu()
{
	m_com = NULL;
	m_lastObjectId = -1;
	m_name[0] = 0;
}

int Qemu::init(const char* name, const char* qemu_path, const char* prog_name, int gdb_port)
{
	pid_t current_pid = getpid();
	snprintf(m_name, sizeof(m_name), "%s-%i", name, current_pid);

	strncpy(m_qemu_path, qemu_path, sizeof(m_qemu_path));
	strncpy(m_prog_name, prog_name, sizeof(m_prog_name));
	m_gdb_port = gdb_port;

	snprintf(m_file_qemu_read, sizeof(m_file_qemu_read), "/tmp/qemu-%s.out", m_name);
	snprintf(m_file_qemu_write, sizeof(m_file_qemu_write), "/tmp/qemu-%s.in", m_name);
	snprintf(m_file_board_read, sizeof(m_file_board_read), "/tmp/carte-%s.out", m_name);
	snprintf(m_file_board_write, sizeof(m_file_board_write), "/tmp/carte-%s.in", m_name);

	m_com = new ComUsb(m_file_qemu_read, m_file_qemu_write);

	startQemu();

	return 0;
}

void Qemu::startQemu()
{
	m_com->close();
	m_lastObjectId = -1;

	mkfifo(m_file_qemu_read, 0666);
	mkfifo(m_file_qemu_write, 0666);
	mkfifo(m_file_board_read, 0666);
	mkfifo(m_file_board_write, 0666);

	// copie du nom avant le fork car m_name ne sera plus accessible
	char name[256];
	strncpy(name, m_name, sizeof(name));
	m_pid = fork();

	if(m_pid == 0)
	{
		char pipe_usb[64];
		char pipe_model[64];
		snprintf(pipe_usb, sizeof(pipe_usb), "pipe,id=foo_usb,path=/tmp/carte-%s", m_name);
		snprintf(pipe_model, sizeof(pipe_model), "pipe,id=foo_model,path=/tmp/qemu-%s", m_name);

		char* arg[15];
		char buf_tcp[64];

		arg[0] = (char*) m_qemu_path;
		arg[1] = (char*) "-M";
		arg[2] = (char*) "atlantronic";
		arg[3] = (char*)"-nodefaults";
		arg[4] = (char*)"-nographic";
		arg[5] = (char*) "-chardev";
		arg[6] = (char*) pipe_usb;
		arg[7] = (char*) "-chardev";
		arg[8] = (char*) pipe_model;
		arg[9] = (char*) "-kernel";
		arg[10] = (char*) m_prog_name;
		if(m_gdb_port)
		{
			arg[11] = (char*) "-S";
			arg[12] = (char*) "-gdb";
			snprintf(buf_tcp, sizeof(buf_tcp), "tcp::%i", m_gdb_port);
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

	if(m_pid < 0)
	{
		perror("fork");
	}

	m_com->open_block();
}

void Qemu::stopQemu()
{
	// TODO un peu bourrin, faire mieux
	if(m_pid > 0)
	{
		kill(m_pid, SIGILL);
	}
	m_pid = 0;

	if( m_com )
	{
		m_com->close();
	}

	unlink(m_file_qemu_read);
	unlink(m_file_qemu_write);
	unlink(m_file_board_read);
	unlink(m_file_board_write);
}

void Qemu::reboot()
{
	stopQemu();
	startQemu();
	setQemuRobotParameters(m_robotParam);
}

void Qemu::destroy()
{
	stopQemu();
	if( m_com )
	{
		delete m_com;
		m_com = NULL;
	}
}

int Qemu::add_object(int flags, const struct polyline polyline, int* objectId)
{
	QemuAtlantronicModelEvent event;
	unsigned int i = 0;

	event.type = EVENT_NEW_OBJECT;

	event.data[0] = flags;
	event.data[1] = MIN((unsigned int)polyline.size, (sizeof(event.data) - 2)/8);
	float* f = (float*)(event.data+2);
	for(i = 0; i < event.data[1]; i++ )
	{
		f[0] = polyline.pt[i].x;
		f[1] = polyline.pt[i].y;
		f += 2;
	}

	m_lastObjectId++;
	if( objectId )
	{
		*objectId = m_lastObjectId;
	}
	return m_com->write((void*) &event, sizeof(event));
}

int Qemu::move_object(int id, Vect2 origin, VectPlan delta)
{
	QemuAtlantronicModelEvent event;

	if( id < 0 )
	{
		return -1;
	}

	event.type = EVENT_MOVE_OBJECT;

	event.data[0] = id;
	float* f = (float*)(event.data+1);
	f[0] = origin.x;
	f[1] = origin.y;
	f[2] = delta.x;
	f[3] = delta.y;
	f[4] = delta.theta;

	return m_com->write((void*) &event, sizeof(event));
}

//! @param nodeId : nodeId ou 0 pour tous
int Qemu::manage_canopen_connexion(int nodeId, bool connected)
{
	QemuAtlantronicModelEvent event;

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

	return m_com->write((void*) &event, sizeof(event));
}

int Qemu::setIo(uint32_t id, bool val)
{
	QemuAtlantronicModelEvent event;
	event.type = EVENT_SET_IO;
	event.data32[0] = id;
	event.data32[1] = val?1:0;

	return m_com->write((void*) &event, sizeof(event));
}

int Qemu::setPosition(VectPlan pos)
{
	QemuAtlantronicModelEvent event;
	event.type = EVENT_SET_POSITION;
	memcpy(&event.data32[0], &pos, sizeof(pos));

	return m_com->write((void*) &event, sizeof(event));
}

int Qemu::setMaxCycleCount(uint32_t maxCycleCount)
{
	QemuAtlantronicModelEvent event;
	event.type = EVENT_SET_MAX_CYCLE_COUNT;
	event.data32[0] = maxCycleCount;

	return m_com->write((void*) &event, sizeof(event));
}

int Qemu::setQemuRobotParameters(QemuRobotParameters param)
{
	m_robotParam = param;
	QemuAtlantronicModelEvent event;
	event.type = EVENT_SET_ROBOT_PARAMETERS;
	memcpy(&event.data[0], &param, sizeof(param));

	return m_com->write((void*) &event, sizeof(event));
}

