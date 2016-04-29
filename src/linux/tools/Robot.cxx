#include "Robot.h"
#include "linux/tools/com/com_usb.h"
#include "linux/tools/com/com_tcp.h"
#include "linux/tools/com/com_udp.h"
#include "linux/tools/com/com_xbee.h"

Robot::Robot()
{
	m_com = NULL;
}

int Robot::init(const char* name,
		bool simu, const char* path, const char* progStm, int gdb_port,
		const char* ip,
		bool xbee,
		bool serverTcp,
		bool serverUdp,
		const char* file_stm,
		void (*_callback)(void*), void* arg)
{
	m_simulation = simu;
	if( simu )
	{
		char qemuPath[256];
		snprintf(qemuPath, sizeof(qemuPath), "%sqemu/arm-softmmu/qemu-system-arm", path);

		int res = m_qemu.init(name, qemuPath, progStm, gdb_port);
		if( res )
		{
			fprintf(stderr, "qemu_init : error\n");
			return -1;
		}

		m_com = new ComUsb(m_qemu.m_file_board_read, m_qemu.m_file_board_write);
	}
	else if(ip && serverTcp)
	{
		m_com = new ComTcp(ip);
	}
	else if(ip && serverUdp)
	{
		m_com = new ComUdp(ip);
	}
	else if( xbee )
	{
		m_com = new ComXbee("/dev/ttyUSB0");
	}
	else
	{
		m_com = new ComUsb(file_stm, file_stm);
	}

	m_robotItf.init(name, m_com, serverTcp,serverUdp, _callback, arg);
	return true;
}

void Robot::destroy()
{
	m_robotItf.destroy();
	m_qemu.destroy();
	if( m_com )
	{
		delete m_com;
	}
}
