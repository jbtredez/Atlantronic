#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "linux/tools/com.h"
#include "linux/tools/cmd.h"
#include "linux/tools/foo_interface.h"

static struct foo_interface foo;

int main(int argc, char** argv)
{
	if(argc < 2)
	{
		printf("indiquer le peripherique\n");
		return -1;
	}

	foo_interface_init(&foo, argv[1], NULL, NULL);
	cmd_init(&foo.com);

	while(1)
	{
		pause();
	}

	com_close(&foo.com);

	return 0;
}
