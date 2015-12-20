#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdio.h>
#include "glplot.h"

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

void robotItfCallback(void* arg);

int main(int argc, char *argv[])
{
	const char* file_stm[ROBOT_MAX] = {"/dev/discovery0", "/dev/discovery1"};
	const char* prog_stm[ROBOT_MAX];
	const char* ip = NULL;
	int gdb_port[ROBOT_MAX] = {0, 0};
	bool simulation[ROBOT_MAX] = {false, false};
	bool serverTcp = false; // TODO option ?
	bool xbee = false;

	setenv("LC_ALL","C",1);

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
			file_stm[0] = argv[optind];
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
				file_stm[i],
				robotItfCallback, NULL);
		if( ! res )
		{
			fprintf(stderr, "robot init failed\n");
			return -1;
		}
	}

	int res = glplot_main(true, robot, ROBOT_MAX);

	// destruction
	for(int i = 0; i < ROBOT_MAX; i++)
	{
		robot[i].destroy();
	}

	return res;
}

void robotItfCallback(void* /*arg*/)
{
	glplot_update();
}

