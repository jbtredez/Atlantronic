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
#include "kernel/hokuyo_tools.h"
#include "kernel/driver/usb.h"
#include "foo/control/control.h"


#define CONTROL_USB_DATA_MAX        18000 //!< 90s de données avec l'asservissement à 200Hz

static uint16_t hokuyo_distance[682]; //!< distances des angles 44 à 725 du hokuyo
static float hokuyo_x[682]; //!< x des points 44 à 725
static float hokuyo_y[682]; //!< y des points 44 à 725
static struct vect_pos pos_robot = {0, 0, 0, 1, 0};
static struct control_usb_data control_usb_data[CONTROL_USB_DATA_MAX];
static int control_usb_data_count = 0;
static struct com foo;

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
		goto end;
	}

	hokuyo_tools_decode_buffer((unsigned char*)msg, 1432, hokuyo_distance, 682);
	hokuyo_compute_xy(hokuyo_distance, 682, hokuyo_x, hokuyo_y, -1);

	// FIXME patch envoi de position avec les data hokuyo
	memcpy(&pos_robot.x, msg + 1432, 4);
	memcpy(&pos_robot.y, msg + 1436, 4);
	memcpy(&pos_robot.alpha, msg + 1440, 4);
	pos_robot.sa = sin(pos_robot.alpha);
	pos_robot.ca = cos(pos_robot.alpha);

end:
	return res;
}

int process_control(char* msg, uint16_t size)
{
	int res = 0;

	if(size != sizeof(struct control_usb_data) )
	{
		res = -1;
		goto end;
	}

	memcpy(control_usb_data + control_usb_data_count, msg, size);
	control_usb_data_count = (control_usb_data_count + 1) % CONTROL_USB_DATA_MAX;

end:
	return res;
}

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

		// au cas ou on a déjà ouvert l'usb : on ne le donne pas a gnuplot
		if(foo.fd > 0)
		{
			close(foo.fd);
		}
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
		fprintf(gnuplot_fd, "%i %i\n",i, hokuyo_distance[i]);
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
		fprintf(gnuplot_fd, "%f %f\n", hokuyo_y[i], hokuyo_x[i]);
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
		// TODO recup position du robot
		pos_hokuyo.x = hokuyo_x[i];
		pos_hokuyo.y = hokuyo_y[i];
		pos_hokuyo_to_table(&pos_robot, &pos_hokuyo, &pos_table);
		fprintf(gnuplot_fd, "%f %f\n", pos_table.x, pos_table.y);
	}
	fprintf(gnuplot_fd, "e\n");

	int max = control_usb_data_count % CONTROL_USB_DATA_MAX;
	for(i=0; i< max; i++)
	{
		if(control_usb_data[i].control_state != CONTROL_READY_ASSER && control_usb_data[i].control_state != CONTROL_READY_FREE)
		{
			fprintf(gnuplot_fd, "%f %f\n", control_usb_data[i].control_cons_x, control_usb_data[i].control_cons_y);
		}
	}

	fprintf(gnuplot_fd, "e\n");
	for(i=0; i < max; i++)
	{
		fprintf(gnuplot_fd, "%f %f\n", control_usb_data[i].control_pos_x, control_usb_data[i].control_pos_y);
	}

	fprintf(gnuplot_fd, "e\n");
	fflush(gnuplot_fd);
}

void plot_speed(FILE* gnuplot_fd)
{
	int i;
	fprintf(gnuplot_fd, "plot \"-\" with lines lc rgbcolor \"blue\", \"-\" with lines lc rgbcolor \"red\"\n");
	for(i=0; i < control_usb_data_count; i++)
	{
		fprintf(gnuplot_fd, "%d %f\n", 5*i, control_usb_data[i].control_v_dist_cons*1000);
	}
	fprintf(gnuplot_fd, "e\n");
	for(i=1; i < control_usb_data_count; i++)
	{
		float dx = control_usb_data[i].control_pos_x - control_usb_data[i-1].control_pos_x;
		float dy = control_usb_data[i].control_pos_x - control_usb_data[i-1].control_pos_x;
		float ds = sqrt(dx*dx+dy*dy);
		fprintf(gnuplot_fd, "%d %f\n", 5*i, ds*200);
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
	int res;
	uint64_t frame_count[3] = {0, 0, 0};
	struct timespec last_plot_time;
	struct timespec current_time;

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

	com_open(&foo, argv[1]);

	clock_gettime(CLOCK_MONOTONIC, &last_plot_time);

	while(1)
	{
		uint16_t type;
		uint16_t size;

		// lecture entete
		res = com_read_header(&foo, &type, &size);
		if( res )
		{
			com_open(&foo, argv[1]);
			continue;
		}

		// lecture du message
		res = com_read(&foo, size + 4);
		if( res )
		{
			com_open(&foo, argv[1]);
			continue;
		}

		// copie du message (vers un buffer non circulaire)
		char msg[size+1];
		com_copy_msg(&foo, msg, size+1);

		// traitement du message
		switch( type )
		{
			case USB_LOG:
				res = process_log(msg, size);
				if( res == 0)
				{
					frame_count[0]++;
				}
				break;
			case USB_HOKUYO:
				res = process_hokuyo(msg, size);
				if( res == 0)
				{
					frame_count[1]++;
				}
				break;
			case USB_CONTROL:
				res = process_control(msg, size);
				if( res == 0)
				{
					frame_count[2]++;
				}
				break;
			default:
				res = -1;
				break;
		}

		if( res )
		{
//			printf("wrong format, type : %i, size = %i, - skip %#.2x (%c)\n", type, size, foo.buffer[foo.buffer_begin], foo.buffer[foo.buffer_begin]);
			com_skip(&foo, 1);
		}
		else
		{
			size += 4;
			com_skip(&foo, size);
		}

		clock_gettime(CLOCK_MONOTONIC, &current_time);
		double dt = current_time.tv_sec - last_plot_time.tv_sec + (current_time.tv_nsec - last_plot_time.tv_nsec) * 0.000000001;
		if( dt > 0.5 )
		{
//			printf("frame %li %li %li\r", frame_count[0], frame_count[1], frame_count[2]);
//			fflush(stdout);
			replot();
			clock_gettime(CLOCK_MONOTONIC, &last_plot_time);
		}
	}

	com_close(&foo);

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
