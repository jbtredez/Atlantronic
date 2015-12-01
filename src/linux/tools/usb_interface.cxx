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
#include "disco/table.h"
#include "linux/tools/Robot.h"

enum
{
	ROBOT_MAIN = 0,
	ROBOT_PMI,
	ROBOT_MAX,
};

static const char* robotName[ROBOT_MAX] =
{
	"main",
	"pmi",
};

static Robot robot[ROBOT_MAX];
static pthread_mutex_t quitMutex;
static pthread_cond_t quitCond;

void cli_quit();

int main(int argc, char** argv)
{
	const char* file_stm = "/dev/discovery0";
	const char* prog_stm[ROBOT_MAX];
	const char* ip = NULL;
	int gdb_port[ROBOT_MAX] = {0, 0};
	bool simulation[ROBOT_MAX] = {false, false};
	bool serverTcp = false; // TODO option ?
	bool xbee = false;

	setenv("LC_ALL","C",1);

	pthread_mutex_init(&quitMutex, NULL);
	pthread_cond_init(&quitCond, NULL);

	// lecture des options
	if(argc > 1)
	{
		int option = -1;
		while( (option = getopt(argc, argv, "gi:p:s:x")) != -1)
		{
			switch(option)
			{
				case 'g':
					gdb_port[ROBOT_MAIN] = 1235;
					gdb_port[ROBOT_PMI] = 1236;
					break;
				case 'i':
					ip = optarg;
					break;
				case 'p':
					simulation[ROBOT_PMI] = true;
					prog_stm[ROBOT_PMI] = optarg;
					break;
				case 's':
					simulation[ROBOT_MAIN] = true;
					prog_stm[ROBOT_MAIN] = optarg;
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

	// gestion des arguments restants
	if( argc - optind > 0)
	{
		if( ! simulation[ROBOT_MAIN] && ! simulation[ROBOT_PMI] )
		{
			file_stm = argv[optind];
		}
		else
		{
			for(int i = 0; i < argc - optind; i++)
			{
				fprintf(stderr, "unknown arguments %s\n", argv[optind+i]);
			}
			return -1;
		}
	}

	// init
	for(int i = 0; i < ROBOT_MAX; i++)
	{
		int res = robot[i].init(robotName[i],
				simulation[i], "", prog_stm[i], gdb_port[i],  // TODO path = argv[0] ?;
				ip,
				xbee, serverTcp,
				file_stm,
				NULL, NULL);
		if( ! res )
		{
			fprintf(stderr, "robot init failed\n");
			return -1;
		}

		// ajout de la table dans qemu
		for(int i = 0; i < TABLE_OBJ_SIZE; i++)
		{
			robot[0].m_qemu.add_object(OBJECT_SEEN_BY_HOKUYO, table_obj[i]);
		}
	}

	pthread_mutex_lock(&quitMutex);
	cmd_init(&robot[0].m_robotItf, &robot[0].m_qemu, cli_quit);
	pthread_cond_wait(&quitCond, &quitMutex);
	pthread_mutex_unlock(&quitMutex);

	// destruction
	for(int i = 0; i < ROBOT_MAX; i++)
	{
		robot[i].destroy();
	}

	return 0;
}

void cli_quit()
{
	pthread_cond_signal(&quitCond);
}
