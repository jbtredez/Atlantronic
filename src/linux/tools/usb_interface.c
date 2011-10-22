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
#include "kernel/driver/usb.h"
#include <errno.h>

int fd = -1;
static unsigned char buffer[10000];
static int buffer_end = 0;
static int buffer_begin = 0;
static int buffer_size = 0;
static uint16_t hokuyo_distance[682]; //!< distances des angles 44 à 725 du hokuyo
static float hokuyo_x[682]; //!< x des points 44 à 725
static float hokuyo_y[682]; //!< y des points 44 à 725
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

int read_buffer(int min_buffer_size)
{
	if( min_buffer_size > (int) sizeof(buffer) )
	{
		printf("error : buffer trop petit : %i > %i\n", min_buffer_size, (int) sizeof(buffer));
		return -1;
	}

	while( buffer_size < min_buffer_size )
	{
		int max1 = sizeof(buffer) - buffer_size;
		int max2 = sizeof(buffer) - buffer_end;
		int max = max1;

		if( max2 < max1)
		{
			max = max2;
		}

		int size = read(fd, buffer + buffer_end, max);
		if(size == 0)
		{
			printf("close usb\n");
			close(fd);
			fd = -1;
			return -1;
		}

		if(size < 0)
		{
			perror("sync - read");
			return -1;
		}

		buffer_end = (buffer_end + size) % sizeof(buffer);
		buffer_size += size;
	}

	return 0;
}

int read_header(uint16_t* type, uint16_t* size)
{
	int res = 0;

	int err = read_buffer(4);
	if(err)
	{
		res = err;
		goto end;
	}

	unsigned char a = buffer[buffer_begin];
	unsigned char b = buffer[(buffer_begin + 1) % sizeof(buffer)];
	unsigned char c = buffer[(buffer_begin + 2) % sizeof(buffer)];
	unsigned char d = buffer[(buffer_begin + 3) % sizeof(buffer)];
	*type = ( a << 8 ) + b;
	*size = ( c << 8 ) + d;

end:
	return res;
}

void open_usb(const char* file)
{
	int last_error = 0;
	buffer_end = 0;
	buffer_begin = 0;
	buffer_size = 0;

	if(fd > 0)
	{
		printf("close usb (=> reopen)\n");
		if( close(fd) )
		{
			perror("close");
		}
	}

	while(1)
	{
		fd = open(file, O_RDONLY);
		if(fd <= 0)
		{
			if( last_error != errno )
			{
				perror("open");
				last_error = errno;
			}
		}
		else
		{
			printf("open usb\n");
			return;
		}
		usleep(1000000);
	}
}

int process_log(char* msg, uint16_t size)
{
	int res = 0;

	if( msg[size-1] != '\n' )
	{
		res = -1;
		goto end;
	}

	printf("%s", msg);

end:
	return res;
}

int process_hokuyo(char* msg, uint16_t size)
{
	int res = 0;

	if(size != 1444)
	{
		res = -1;
	}

	hokuyo_tools_decode_buffer((unsigned char*)msg, 1432, hokuyo_distance, 682);
	hokuyo_compute_xy(hokuyo_distance, 682, hokuyo_x, hokuyo_y, -1);

	// FIXME patch envoi de position avec les data hokuyo
	memcpy(&pos_robot.x, msg + 1432, 4);
	memcpy(&pos_robot.y, msg + 1436, 4);
	memcpy(&pos_robot.alpha, msg + 1440, 4);
	pos_robot.sa = sin(pos_robot.alpha);
	pos_robot.ca = cos(pos_robot.alpha);

	return res;
}

FILE* open_gnuplot(const char* title, const char* xlabel, const char* ylabel, int xmin, int xmax, int ymin, int ymax)
{
	FILE* fd = popen("gnuplot", "w");
	if(fd == NULL)
	{
		perror("popen");
		goto end;
	}

	fprintf(fd, "set term x11 noraise\n");
	fprintf(fd, "set mouse\n");
	fprintf(fd, "set title \"%s\"\n", title);
	fprintf(fd, "set xlabel \"%s\"\n", xlabel);
	fprintf(fd, "set ylabel \"%s\"\n", ylabel);
	fprintf(fd, "set xrange [%d:%d]\n", xmin, xmax);
	fprintf(fd, "set yrange [%d:%d]\n", ymin, ymax);

end:
	return fd;
}

int main(int argc, char** argv)
{
	int res;
	int i;
	int j;

	if(argc < 2)
	{
		printf("indiquer le peripherique\n");
		return -1;
	}

	// affichage des données brutes
	FILE* plot_d = open_gnuplot("Distances selon l'indice", "indice", "distance", 0, 682, 0, 4100);
	if(plot_d == NULL)
	{
		return 0;
	}

	// affichage dans le repere hokuyo
	FILE* plot_xy = open_gnuplot("Points dans le repere hokuyo", "y", "x", -3000, 3000, 0, 4100);
	if(plot_xy == NULL)
	{
		return 0;
	}

	// affichage dans le repere table
	FILE* plot_table = open_gnuplot("Points dans le repere table", "x", "y", -1800, 1800, -1200, 1200);
	if(plot_table == NULL)
	{
		return 0;
	}

	// fait apres les pipes pour eviter un pb sur le close(fd)
	open_usb(argv[1]);

	while(1)
	{
		uint16_t type;
		uint16_t size;
		// lecture entete
		res = read_header(&type, &size);
		if( res )
		{
			open_usb(argv[1]);
			continue;
		}

		// lecture du message
		res = read_buffer( size + 4);
		if( res )
		{
			open_usb(argv[1]);
			continue;
		}

		// copie du message (vers un buffer non circulaire)
		char msg[size+1];
		i = (buffer_begin + 4) % sizeof(buffer);
		for(j = 0; j < size ; j++)
		{
			msg[j] = buffer[i];
			i = (i + 1) % sizeof(buffer);
		}
		msg[size] = 0;

		// traitement du message
		switch( type )
		{
			case USB_LOG:
				res = process_log(msg, size);
				break;
			case USB_HOKUYO:
				res = process_hokuyo(msg, size);
				break;
			default:
				res = -1;
				break;
		}

		if( res )
		{
			printf("wrong format, type : %i, size = %i, buffer_size = %i\n", type, size, buffer_size);
			buffer_begin = (buffer_begin + 1) % sizeof(buffer);
			buffer_size--;
		}
		else
		{
			size += 4;
			buffer_size -= size;
			buffer_begin = (buffer_begin + size) % sizeof(buffer);
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

	if(fd > 0)
	{
		close(fd);
	}

	if( plot_table != NULL )
	{
		fclose(plot_table);
	}

	if( plot_xy != NULL )
	{
		fclose(plot_xy);
	}

	if( plot_d != NULL )
	{
		fclose(plot_d);
	}

	return 0;
}
