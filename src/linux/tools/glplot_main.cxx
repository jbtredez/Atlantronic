#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdio.h>
#include "glplot.h"
#include "linux/tools/com/com_usb.h"
#include "linux/tools/com/com_tcp.h"
#include "linux/tools/com/com_xbee.h"

static Qemu qemu;
static RobotInterface robotItf;

void robotItfCallback(void* arg);

int main(int argc, char *argv[])
{
	const char* file_stm_read = NULL;
	const char* file_stm_write = NULL;
	const char* prog_stm = NULL;
	const char* ip = NULL;
	int gdb_port = 0;
	int simulation = 0;
	bool serverTcp = true; // TODO option ?
	bool xbee = false;
	Com* com;

	setenv("LC_ALL","C",1);

	if(argc > 1)
	{
		int option = -1;
		while( (option = getopt(argc, argv, "gi:s:x")) != -1)
		{
			switch(option)
			{
				case 'g':
					gdb_port = 1235;
					break;
				case 'i':
					ip = optarg;
					break;
				case 's':
					simulation = 1;
					prog_stm = optarg;
					break;
				case 'x':
					xbee = true;
					break;
				default:
					fprintf(stderr, "option %c inconnue", (char)option);
					return -1;
					break;
			}
		}
	}

	if( argc - optind > 0)
	{
		file_stm_read = argv[optind];
		file_stm_write = file_stm_read;
	}

	if(simulation)
	{
		int res = qemu.init("qemu/arm-softmmu/qemu-system-arm", prog_stm, gdb_port);
		if( res )
		{
			fprintf(stderr, "qemu_init : error");
			return -1;
		}

		file_stm_read = qemu.m_file_board_read;
		file_stm_write = qemu.m_file_board_write;
	}

	if(file_stm_read)
	{
		com = new ComUsb(file_stm_read, file_stm_write);
	}
	else if(ip)
	{
		com = new ComTcp(ip);
	}
	else if( xbee )
	{
		com = new ComXbee("/dev/ttyUSB0");
	}
	else
	{
		com = new ComUsb("/dev/discovery0", "/dev/discovery0");
	}

	robotItf.init("discovery", com, serverTcp, robotItfCallback, NULL);

	int res = glplot_main("", simulation, true, &qemu, &robotItf);

	robotItf.destroy();

	qemu.destroy();

	return res;
}

void robotItfCallback(void* /*arg*/)
{
	glplot_update();
}
