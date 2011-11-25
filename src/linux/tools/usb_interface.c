#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <time.h>
#include "linux/tools/com.h"
#include "linux/tools/cmd.h"
#include "linux/tools/foo_interface.h"

static struct foo_interface foo;

FILE* plot_d_fd = NULL;
FILE* plot_xy_fd = NULL;
FILE* plot_table_fd = NULL;
FILE* plot_speed_fd = NULL;

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

FILE* open_gnuplot(const char* title, const char* xlabel, const char* ylabel, int xmin, int xmax, int ymin, int ymax)
{
	FILE* gnuplot_fd = NULL;
	pid_t pID;

	int outfd[2];
	// int infd[2];

	int res = pipe(outfd);
	if(res)
	{
		perror("pipe");
		goto end;
	}

	// pipe(infd);

	pID = vfork();

	if( pID < 0)
	{
		perror("vfork");
		goto end;
	}

	if( pID == 0)
	{
//		close(STDOUT_FILENO);
		close(STDIN_FILENO);
		close(STDERR_FILENO);

		dup2(outfd[0], STDIN_FILENO);
//		dup2(infd[1], STDOUT_FILENO);

		close(outfd[0]);
		close(outfd[1]);
//		close(infd[0]);
//		close(infd[1]);

		char* arg[2];

		arg[0] = (char*) "/usr/bin/gnuplot";
		arg[1] = NULL;

		execv(arg[0], arg);
		perror("execv");
		exit(-1);
	}

	close(outfd[0]);
//	close(infd[1]);

	gnuplot_fd = fdopen(outfd[1], "w");

	if( gnuplot_fd == NULL)
	{
		goto end;
	}

	fprintf(gnuplot_fd, "set term x11 noraise title \"%s\"\n", title);
	fprintf(gnuplot_fd, "set mouse\n");
	fprintf(gnuplot_fd, "set title \"%s\"\n", title);
	fprintf(gnuplot_fd, "set xlabel \"%s\"\n", xlabel);
	fprintf(gnuplot_fd, "set ylabel \"%s\"\n", ylabel);
	fprintf(gnuplot_fd, "set xrange [%d:%d]\n", xmin, xmax);
	fprintf(gnuplot_fd, "set yrange [%d:%d]\n", ymin, ymax);

end:
	return gnuplot_fd;
}

void plot_hokuyo_distance(FILE* gnuplot_fd)
{
	int i;
	fprintf(gnuplot_fd, "plot \"-\"\n");
	for(i=0; i < 682; i++)
	{
		fprintf(gnuplot_fd, "%i %i\n",i, foo.hokuyo_scan.distance[i]);
	}
	fprintf(gnuplot_fd, "e\n");
	fflush(gnuplot_fd);
}

void plot_hokuyo_xy(FILE* gnuplot_fd)
{
	int i;
	fprintf(gnuplot_fd, "plot \"-\"\n");
	for(i=0; i < 682; i++)
	{
		fprintf(gnuplot_fd, "%f %f\n", foo.hokuyo_y[i], foo.hokuyo_x[i]);
	}
	fprintf(gnuplot_fd, "e\n");
	fflush(gnuplot_fd);
}

void plot_table(FILE* gnuplot_fd)
{
	int i;
	fprintf(gnuplot_fd, "plot \"-\" with lines lc rgbcolor \"black\", \"-\" lc rgbcolor \"red\", \"-\" with lines lc rgbcolor \"blue\", \"-\" lc rgbcolor \"green\"\n");
	for(i=0; i < MAX_TABLE_PTS; i+=2)
	{
		fprintf(gnuplot_fd, "%f %f\n", table_pts[i], table_pts[i+1]);
	}
	fprintf(gnuplot_fd, "e\n");

	struct vect_pos pos_hokuyo = {0, 0, 0, 1, 0};
	struct vect_pos pos_table = {0, 0, 0, 1, 0};

	for(i=0; i < 682; i++)
	{
		pos_hokuyo.x = foo.hokuyo_x[i];
		pos_hokuyo.y = foo.hokuyo_y[i];
		pos_hokuyo_to_table(&foo.hokuyo_scan.pos, &pos_hokuyo, &pos_table);
		fprintf(gnuplot_fd, "%f %f\n", pos_table.x, pos_table.y);
	}
	fprintf(gnuplot_fd, "e\n");

	int max = foo.control_usb_data_count % CONTROL_USB_DATA_MAX;
	for(i=0; i< max; i++)
	{
		if(foo.control_usb_data[i].control_state != CONTROL_READY_ASSER && foo.control_usb_data[i].control_state != CONTROL_READY_FREE)
		{
			fprintf(gnuplot_fd, "%f %f\n", foo.control_usb_data[i].control_cons_x, foo.control_usb_data[i].control_cons_y);
		}
	}

	fprintf(gnuplot_fd, "e\n");
	for(i=0; i < max; i++)
	{
		fprintf(gnuplot_fd, "%f %f\n", foo.control_usb_data[i].control_pos_x, foo.control_usb_data[i].control_pos_y);
	}

	fprintf(gnuplot_fd, "e\n");
	fflush(gnuplot_fd);
}

void plot_speed(FILE* gnuplot_fd)
{
	int i;
	fprintf(gnuplot_fd, "plot \"-\" title \"consigne de vitesse\" with lines lc rgbcolor \"blue\", \"-\" title \"mesure de vitesse\" with lines lc rgbcolor \"red\", \"-\" title \"intensite droite\" with lines lc rgbcolor \"green\", \"-\" title \"intensite gauche\" with lines lc rgbcolor \"orange\"\n");
	for(i=0; i < foo.control_usb_data_count; i++)
	{
		fprintf(gnuplot_fd, "%d %f\n", 5*i, foo.control_usb_data[i].control_v_dist_cons*200);
	}
	fprintf(gnuplot_fd, "e\n");
	for(i=1; i < foo.control_usb_data_count; i++)
	{
		fprintf(gnuplot_fd, "%d %f\n", 5*i, foo.control_usb_data[i].control_v_dist_mes*200);
	}
	fprintf(gnuplot_fd, "e\n");
	for(i=1; i < foo.control_usb_data_count; i++)
	{
		fprintf(gnuplot_fd, "%d %u\n", 5*i, (unsigned int)foo.control_usb_data[i].control_i_right);
	}
	fprintf(gnuplot_fd, "e\n");
	for(i=1; i < foo.control_usb_data_count; i++)
	{
		fprintf(gnuplot_fd, "%d %u\n", 5*i, (unsigned int)foo.control_usb_data[i].control_i_left);
	}
	fprintf(gnuplot_fd, "e\n");
	fflush(gnuplot_fd);
}

void replot()
{
	plot_hokuyo_distance(plot_d_fd);
	plot_hokuyo_xy(plot_xy_fd);
	plot_table(plot_table_fd);
	plot_speed(plot_speed_fd);
}

int main(int argc, char** argv)
{
	if(argc < 2)
	{
		printf("indiquer le peripherique\n");
		return -1;
	}

	// affichage des données brutes
	plot_d_fd = open_gnuplot("Distances selon l'indice", "indice", "distance", 0, 682, 0, 4100);
	if(plot_d_fd == NULL)
	{
		return 0;
	}

	// affichage dans le repere hokuyo
	plot_xy_fd = open_gnuplot("Points dans le repere hokuyo", "y", "x", -3000, 3000, 0, 4100);
	if(plot_xy_fd == NULL)
	{
		return 0;
	}

	// affichage dans le repere table
	plot_table_fd = open_gnuplot("Points dans le repere table", "x", "y", -1800, 1800, -1200, 1200);
	if(plot_table_fd == NULL)
	{
		return 0;
	}

	// affichage de la consigne et de la mesure de vitesse
	plot_speed_fd = open_gnuplot("Suivit en vitesse", "t", "v", 0, 90000, -2000, 2000);
	if(plot_speed_fd == NULL)
	{
		return 0;
	}

	// affichage des graph
	replot();

	foo_interface_init(&foo, argv[1], NULL, NULL);
	cmd_init(&foo.com);

	while(1)
	{
		int res = pthread_mutex_lock(&foo.mutex);
		if(res == 0)
		{
			replot();
			pthread_mutex_unlock(&foo.mutex);
		}
		usleep(500000);
	}

	com_close(&foo.com);

	if( plot_table_fd != NULL )
	{
		fclose(plot_table_fd);
	}

	if( plot_xy_fd != NULL )
	{
		fclose(plot_xy_fd);
	}

	if( plot_d_fd != NULL )
	{
		fclose(plot_d_fd);
	}

	return 0;
}
