#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdio.h>
#include "glplot.h"

int main(int argc, char *argv[])
{
	const char* file_stm_read = NULL;
	const char* file_stm_write = NULL;
	const char* prog_stm = NULL;
	int gdb_port = 0;
	int simulation = 0;
	Qemu qemu;

	if(argc > 1)
	{
		int option = -1;
		while( (option = getopt(argc, argv, "s:g")) != -1)
		{
			switch(option)
			{
				case 's':
					simulation = 1;
					prog_stm = optarg;
					break;
				case 'g':
					gdb_port = 1235;
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
		int res = qemu.init(prog_stm, gdb_port);
		if( res )
		{
			fprintf(stderr, "qemu_init : error");
			return -1;
		}

		file_stm_read = qemu.file_board_read;
		file_stm_write = qemu.file_board_write;
	}

	int res = glplot_main(simulation, &qemu, file_stm_read, file_stm_write);

	qemu.destroy();

	return res;
}
