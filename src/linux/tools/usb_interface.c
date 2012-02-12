#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "linux/tools/com.h"
#include "linux/tools/cmd.h"
#include "linux/tools/robot_interface.h"

static struct robot_interface robot_interface;

int main(int argc, char** argv)
{
	const char* file_foo = NULL;
	const char* file_bar = NULL;

	if( argc > 1)
	{
		file_foo = argv[1];
	}

	if(argc > 2)
	{
		file_bar = argv[2];
	}

	robot_interface_init(&robot_interface, file_foo, file_bar, NULL, NULL);
	cmd_init(&robot_interface, NULL);

	while(1)
	{
		pause();
	}

	robot_interface_destroy(&robot_interface);

	return 0;
}
