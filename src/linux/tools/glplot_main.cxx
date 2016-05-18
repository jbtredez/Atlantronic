#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include "glplot.h"
#include "disco/star/star.h"
#include "disco/gate/gate.h"
#include "server_udp.h"
enum
{
	ROBOT_MAIN = 0,
	ROBOT_PMI,
	ROBOT_MAX,
};

static const char* robotName[ROBOT_MAX] =
{
	"Star",
	"Gate",
};

static Robot robot[ROBOT_MAX];

static QemuRobotParameters robotParam[ROBOT_MAX] =
{
	{
		STAR_ODO1_WHEEL_RADIUS,
		STAR_ODO2_WHEEL_RADIUS,
		STAR_ODO1_WAY,
		STAR_ODO2_WAY,
		STAR_ODO_ENCODER_RESOLUTION,
		STAR_VOIE_ODO,
		STAR_VOIE_MOT,
		STAR_DRIVING1_WHEEL_RADIUS,
		STAR_DRIVING2_WHEEL_RADIUS,
		STAR_MOTOR_ENCODER_RESOLUTION,
		STAR_MOTOR_DRIVING1_RED,
		STAR_MOTOR_DRIVING2_RED,
		1,
	},
	{
		GATE_ODO1_WHEEL_RADIUS,
		GATE_ODO2_WHEEL_RADIUS,
		GATE_ODO1_WAY,
		GATE_ODO2_WAY,
		GATE_ODO_ENCODER_RESOLUTION,
		GATE_VOIE_ODO,
		GATE_VOIE_MOT,
		GATE_DRIVING1_WHEEL_RADIUS,
		GATE_DRIVING2_WHEEL_RADIUS,
		1024,
		GATE_MOTOR_DRIVING1_RED,
		GATE_MOTOR_DRIVING2_RED,
		0,
	}
};

void robotItfCallback(void* arg);

int main(int argc, char *argv[])
{
	const char* file_stm[ROBOT_MAX] = {"/dev/discovery0","/dev/discovery1"};
	const char* prog_stm[ROBOT_MAX];
	const char* ip[ROBOT_MAX]={NULL,NULL};
	int gdb_port[ROBOT_MAX] = {0,0};
	bool simulation[ROBOT_MAX] = {false,false};
	bool serverTcp = false; // TODO option ?
	ServerUdp serverUdp ;// TODO option ?
	bool xbee = false;
	setenv("LC_ALL","C",1);

	// lecture des options
	if(argc > 1)
	{
		int option = -1;
		while( (option = getopt(argc, argv, "gi:p:s:x:u")) != -1)
		{
			switch(option)
			{
				case 'g':
					gdb_port[ROBOT_MAIN] = 1235;
					gdb_port[ROBOT_PMI] = 1236;
					break;
				case 'i':
					ip[ROBOT_MAIN]= optarg;
					break;
				case 'p':
					simulation[ROBOT_PMI] = true;
					prog_stm[ROBOT_PMI] = optarg;
					break;
				case 's':
					simulation[ROBOT_MAIN] = true;
					prog_stm[ROBOT_MAIN] = optarg;
					break;
				case 'l':
					ip[ROBOT_PMI]= optarg;
					break;
				case 'x':
					xbee = true;
					break;
				case 'u':
					///Préparation de l'ouverture du port udp
						serverUdp.configure(55056);;
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
				ip[i],
				xbee, serverTcp,&serverUdp,
				file_stm[i],
				robotItfCallback, NULL);
		if( ! res )
		{
			fprintf(stderr, "robot init failed\n");
			return -1;
		}

		if( robot[i].m_qemu.isInitDone() )
		{
			robot[i].m_qemu.setQemuRobotParameters(robotParam[i]);
		}
	}

///Démarage du serveur
	serverUdp.start();
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
	int minCycleCount = INT_MAX;
	for(int i = 0; i < ROBOT_MAX; i++)
	{
		if( robot[i].m_qemu.isInitDone() )
		{
			int t_ms = (int)(robot[i].m_robotItf.current_time * 1000);
			if( t_ms < minCycleCount )
			{
				minCycleCount = t_ms;
			}
		}
	}

	for(int i = 0; i < ROBOT_MAX; i++)
	{
		if( robot[i].m_qemu.isInitDone() )
		{
			robot[i].m_qemu.setMaxCycleCount(minCycleCount + 500);
		}
	}
}

