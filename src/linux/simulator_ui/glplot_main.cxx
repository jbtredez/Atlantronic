#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include <time.h>
#include "linux/simulator_ui/glplot.h"
#include "disco/star/star.h"
#include "disco/gate/gate.h"


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
		STAR_VOIE_ODO_POS,
		STAR_VOIE_MOT,
		60 * STAR_MOTOR_DRIVING1_RED / (float)(2 * M_PI * STAR_DRIVING1_WHEEL_RADIUS),
		2 * M_PI * STAR_DRIVING1_WHEEL_RADIUS / (float)(STAR_MOTOR_ENCODER_RESOLUTION * STAR_MOTOR_DRIVING1_RED),
		60 * STAR_MOTOR_DRIVING2_RED / (float)(2 * M_PI * STAR_DRIVING2_WHEEL_RADIUS),
		2 * M_PI * STAR_DRIVING2_WHEEL_RADIUS / (float)(STAR_MOTOR_ENCODER_RESOLUTION * STAR_MOTOR_DRIVING2_RED),
		1,
		STAR_HALF_LENGTH,
		STAR_HALF_WIDTH,
		STAR_HOKUYO_1_X,
		STAR_HOKUYO_1_Y,
		STAR_HOKUYO_1_THETA,
		STAR_HOKUYO_2_X,
		STAR_HOKUYO_2_Y,
		STAR_HOKUYO_2_THETA,
	},
	{
		GATE_ODO1_WHEEL_RADIUS,
		GATE_ODO2_WHEEL_RADIUS,
		GATE_ODO1_WAY,
		GATE_ODO2_WAY,
		GATE_ODO_ENCODER_RESOLUTION,
		GATE_VOIE_ODO_POSITIF,
		GATE_VOIE_MOT,
		60 * GATE_MOTOR_DRIVING1_RED / (float)(2 * M_PI * GATE_DRIVING1_WHEEL_RADIUS) * GATE_MOTOR_RPM_TO_VOLT,
		1,
		60 * GATE_MOTOR_DRIVING2_RED / (float)(2 * M_PI * GATE_DRIVING2_WHEEL_RADIUS) * GATE_MOTOR_RPM_TO_VOLT,
		1,
		0,
		GATE_HALF_LENGTH,
		GATE_HALF_WIDTH,
		GATE_HOKUYO_1_X,
		GATE_HOKUYO_1_Y,
		GATE_HOKUYO_1_THETA,
		GATE_HOKUYO_2_X,
		GATE_HOKUYO_2_Y,
		GATE_HOKUYO_2_THETA,
	}
};

void robotItfCallback(void* arg);

void printHelp()
{
	printf(
		"Options :\n"
		"  -g : gdb (port 1235 for main robot and 1236 for PMI)\n"
		"  -h : help\n"
		"  -i : ip (distant IHM)\n"
		"  -p : PMI simulation\n"
		"  -s : main robot simulation\n"
		"  -x : xbee (distant IHM)\n"
	);
}

int main(int argc, char *argv[])
{
	const char* file_stm[ROBOT_MAX] = {"/dev/discovery0", "/dev/discovery1"};
	const char* prog_stm[ROBOT_MAX];
	const char* ip = NULL;
	int gdb_port[ROBOT_MAX] = {0, 0};
	bool simulation[ROBOT_MAX] = {false, false};
	bool serverTcp = false; // TODO option ?
	bool xbee = false;
	int selectRobotId = 0;

	setenv("LC_ALL","C",1);

	// lecture des options
	if(argc > 1)
	{
		int option = -1;
		while( (option = getopt(argc, argv, "ghi:p:s:x")) != -1)
		{
			switch(option)
			{
				case 'g':
					gdb_port[ROBOT_MAIN] = 1235;
					gdb_port[ROBOT_PMI] = 1236;
					break;
				case 'h':
					printHelp();
					return 0;
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
	if( simulation[ROBOT_PMI] && ! simulation[ROBOT_MAIN])
	{
		selectRobotId = 1;
	}

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

		if( robot[i].m_qemu.isInitDone() )
		{
			robot[i].m_qemu.setQemuRobotParameters(robotParam[i]);
		}
	}

	int res = glplot_main(true, robot, ROBOT_MAX, selectRobotId);

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

