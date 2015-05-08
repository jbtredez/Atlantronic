#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>

#include "linux/tools/com/com_usb.h"
#include "linux/tools/com/com_tcp.h"
#include "linux/tools/com/com_xbee.h"
#include "linux/tools/cmd.h"
#include "linux/tools/robot_interface.h"
#include "linux/tools/qemu.h"
#include "table.h"

static RobotInterface robotItf;
static Qemu qemu;
static pthread_mutex_t quitMutex;
static pthread_cond_t quitCond;

void cli_quit();

int main(int argc, char** argv)
{
	const char* file_stm_read = NULL;
	const char* file_stm_write = NULL;
	const char* prog_stm = NULL;
	const char* ip = NULL;
	int gdb_port = 0;
	int simulation = 0;
	bool serverTcp = false;
	bool xbee = false;
	Com* com;

	pthread_mutex_init(&quitMutex, NULL);
	pthread_cond_init(&quitCond, NULL);

	if(argc > 1)
	{
		int option = -1;
		while( (option = getopt(argc, argv, "gi:s:")) != -1)
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

		// ajout de la table dans qemu
		for(int i = 0; i < TABLE_OBJ_SIZE; i++)
		{
			qemu.add_object(OBJECT_FLOOR_FOOTPRINT, table_obj[i]);
		}
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

	robotItf.init("discovery", com, serverTcp, NULL, NULL);

	pthread_mutex_lock(&quitMutex);
	cmd_init(&robotItf, &qemu, cli_quit);
	pthread_cond_wait(&quitCond, &quitMutex);
	pthread_mutex_unlock(&quitMutex);

	robotItf.destroy();

	qemu.destroy();

	return 0;
}

void cli_quit()
{
	pthread_cond_signal(&quitCond);
}
