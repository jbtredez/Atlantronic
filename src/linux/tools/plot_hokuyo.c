#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "kernel/hokuyo_tools.h"

int fd = -1;
static uint16_t hokuyo_distance[682]; //!< distances des angles 44 à 725 du hokuyo
static float hokuyo_x[682]; //!< x des points 44 à 725
static float hokuyo_y[682]; //!< y des points 44 à 725
static unsigned char buffer[10240];
static int buffer_end = 0;
static int buffer_begin = 0;
static int buffer_size = 0;
static struct vect_pos pos_robot = {0, 0, 0, 1, 0};

#define MAX_TABLE_PTS   88
float table_pts[MAX_TABLE_PTS] =
{
	-1500,  1050,
	 1500,  1050,
	 1500, -1050,
	 1100, -1050,
	 1100,  -650,
	 1500,  -650,
	 1500, -1050,
	-1500, -1050,
	-1100, -1050,
	-1100,  -650,
	-1500,  -650,
	-1500, -1050,
	-1500,  1050,
	-1100,  1050,
	-1100, -1050,
	-1050, -1050,
	-1050,  1050,
	-1050,   700,
	 -700,   700,
	 1050,   700,
	 -700,   700,
	 -700, -1050,
	 -350, -1050,
	 -350,   700,
	    0,   700,
	    0, -1050,
	  350, -1050,
	  350,   700,
	  700,   700,
	  700, -1050,
	 1100, -1050,
	 1100,  1050,
	 1050,  1050,
	 1050, -1050,
	 1050,  -700,
	-1050,  -700,
	-1050,  -350,
	 1050,  -350,
	 1050,     0,
	-1050,     0,
	-1050,   350,
	 1050,   350,
	 1050,   700,
	-1050,   700,
};

enum
{
	CMD_UNKNOWN = 0,
	CMD_GS
};

int knownCommand(char a, char b)
{
	int res = CMD_UNKNOWN;

	if(a == 'G' && b == 'S')
	{
		res = CMD_GS;
	}

	return res;
}

int read_buffer()
{
	int max1 = sizeof(buffer) - buffer_size;
	int max2 = sizeof(buffer) - buffer_end;
	int max = max1;

	if( max2 < max1)
	{
		max = max2;
	}

	int size = read(fd, buffer + buffer_end, max);
	if(size <= 0)
	{
		perror("sync - read");
		return -1;
	}

	buffer_end = (buffer_end + size) % sizeof(buffer);
	buffer_size += size;

	return 0;
}

int sync_data()
{
	int res = 0;

	while(1)
	{
		int err = read_buffer();
		if(err)
		{
			res = err;
			goto end;
		}

		while( buffer_size >= 2)
		{
			char a = buffer[buffer_begin];
			char b = buffer[(buffer_begin + 1) % sizeof(buffer)];
			int cmd = knownCommand(a,b);
			if( cmd )
			{
				res = cmd;
				goto end;
			}
			else
			{
				buffer_size--;
				buffer_begin++;
			}
		}
	}

end:
	return res;
}

int process_gs_cmd()
{
	int res = 0;
	int i;

	while( buffer_size < 1432+12)
	{
		int err = read_buffer();
		if(err)
		{
			res = err;
			goto end;
		}
	}

	unsigned char hoku_buf[1432+12];
	for(i=0; i<1432+12; i++)
	{
		hoku_buf[i] = buffer[buffer_begin];
//		printf("%c", hoku_buf[i]);
		buffer_size--;
		buffer_begin = (buffer_begin + 1) % sizeof(buffer);
	}

	hokuyo_tools_decode_buffer(hoku_buf, 1432, hokuyo_distance, 682);
	hokuyo_compute_xy(hokuyo_distance, 682, hokuyo_x, hokuyo_y, -1);

	// FIXME patch envoi de position sur ep2
	memcpy(&pos_robot.x, hoku_buf + 1432, 4);
	memcpy(&pos_robot.y, hoku_buf + 1436, 4);
	memcpy(&pos_robot.alpha, hoku_buf + 1440, 4);
	pos_robot.sa = sin(pos_robot.alpha);
	pos_robot.ca = cos(pos_robot.alpha);

end:
	return res;
}

int main(int argc, char** argv)
{
	int i;

	if(argc < 2)
	{
		printf("indiquer le peripherique\n");
		return -1;
	}

	// affichage des données brutes
	FILE* plot_d = popen("gnuplot", "w");
	if(plot_d == NULL)
	{
		perror("popen");
		return 0;
	}

	fprintf(plot_d, "set term x11 noraise\n");
	fprintf(plot_d, "set mouse\n");
	fprintf(plot_d, "set title \"Distances selon l'indice\"\n");
	fprintf(plot_d, "set xlabel \"indice\"\n");
	fprintf(plot_d, "set ylabel \"distance\"\n");
	fprintf(plot_d, "set xrange [0:682]\n");
	fprintf(plot_d, "set yrange [0:4100]\n");

	// affichage dans le repere hokuyo
	FILE* plot_xy = popen("gnuplot", "w");
	if(plot_xy == NULL)
	{
		perror("popen");
		return 0;
	}

	fprintf(plot_xy, "set term x11 noraise\n");
	fprintf(plot_xy, "set mouse\n");
	fprintf(plot_xy, "set title \"Points dans le repere hokuyo\"\n");
	fprintf(plot_xy, "set xlabel \"y\"\n");
	fprintf(plot_xy, "set ylabel \"x\"\n");
	fprintf(plot_xy, "set xrange [-3000:3000]\n");
	fprintf(plot_xy, "set yrange [0:4100]\n");

	// affichage dans le repere table
	FILE* plot_table = popen("gnuplot", "w");
	if(plot_table == NULL)
	{
		perror("popen");
		return 0;
	}

	fprintf(plot_table, "set term x11 noraise\n");
	fprintf(plot_table, "set mouse\n");
	fprintf(plot_table, "set title \"Points dans le repere table\"\n");
	fprintf(plot_table, "set xlabel \"x\"\n");
	fprintf(plot_table, "set ylabel \"y\"\n");
	fprintf(plot_table, "set xrange [-1800:1800]\n");
	fprintf(plot_table, "set yrange [-1200:1200]\n");

	fd = open(argv[1], O_RDONLY);
	if(fd <= 0)
	{
		perror("fopen");
		return -1;
	}

	while(1)
	{
		int cmd = sync_data();
		switch( cmd)
		{
			case CMD_UNKNOWN:
				break;
			case CMD_GS:
				process_gs_cmd();
				break;
			default:
				return -1;
				break;
		}

		fprintf(plot_d, "plot \"-\"\n");
		for(i=0; i < 682; i++)
		{
			fprintf(plot_d, "%i %i\n",i, hokuyo_distance[i]);
		}
		fprintf(plot_d, "e\n");
		fflush(plot_d);

		fprintf(plot_xy, "plot \"-\"\n");
		for(i=0; i < 682; i++)
		{
			fprintf(plot_xy, "%f %f\n", hokuyo_y[i], hokuyo_x[i]);
		}
		fprintf(plot_xy, "e\n");
		fflush(plot_xy);

		fprintf(plot_table, "plot \"-\" with lines lc rgbcolor \"black\", \"-\"\n");
		for(i=0; i < MAX_TABLE_PTS; i+=2)
		{
			fprintf(plot_table, "%f %f\n", table_pts[i], table_pts[i+1]);
		}
		fprintf(plot_table, "e\n");

		struct vect_pos pos_hokuyo = {0, 0, 0, 1, 0};
		struct vect_pos pos_table = {0, 0, 0, 1, 0};

		for(i=0; i < 682; i++)
		{
			// TODO recup position du robot
			pos_hokuyo.x = hokuyo_x[i];
			pos_hokuyo.y = hokuyo_y[i];
			pos_hokuyo_to_table(&pos_robot, &pos_hokuyo, &pos_table);
			fprintf(plot_table, "%f %f\n", pos_table.x, pos_table.y);
		}
		fprintf(plot_table, "e\n");
		fflush(plot_table);
	}

	close(fd);
	fclose(plot_table);
	fclose(plot_xy);
	fclose(plot_d);

	return 0;
}
