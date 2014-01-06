#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "linux/tools/com.h"
#include "linux/tools/cmd.h"
#include "linux/tools/robot_interface.h"

static RobotInterface robot_interface;

int main(int argc, char** argv)
{
	const char* file = NULL;

	if( argc > 1)
	{
		file = argv[1];
	}

	robot_interface.init("discovery", file, file, NULL, NULL);
	cmd_init(&robot_interface, NULL, NULL);

	while(1)
	{
		pause();
	}

	robot_interface.destroy();

	return 0;
}
