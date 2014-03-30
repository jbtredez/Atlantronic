#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "linux/tools/graphique.h"

void Graphique::init(const char* Name, float Xmin, float Xmax, float Ymin, float Ymax, int Screen_width, int Screen_height, int Bordure_pixel_x, int Bordure_pixel_y)
{
	int i = 0;

	name = (char*)malloc(strlen(Name)+1);
	strcpy(name, Name);

	for(i = 0; i < MAX_COURBES; i++)
	{
		courbes_names[i] = NULL;
		courbes_activated[i] = 0;
	}

	memset(color, 0x00, sizeof(color));

	xmin = Xmin;
	xmax = Xmax;
	ymin = Ymin;
	ymax = Ymax;

	screen_width = Screen_width;
	screen_height = Screen_height;
	bordure_pixel_x = Bordure_pixel_x;
	bordure_pixel_y = Bordure_pixel_y;

	// par defaut roi = tout
	reset_roi();
}

void Graphique::add_courbe(int id, const char* name, int activated, float r, float g, float b)
{
	if(id >= MAX_COURBES || id < 0)
	{
		return;
	}

	if(courbes_names[id])
	{
		free(courbes_names[id]);
	}

	color[3*id] = r;
	color[3*id+1] = g;
	color[3*id+2] = b;
	courbes_activated[id] = activated;
	courbes_names[id] = (char*)malloc(strlen(name)+1);
	strcpy(courbes_names[id], name);
}

Graphique::~Graphique()
{
	int i;
	if(name)
	{
		free(name);
		name = NULL;
	}

	for(i = 0; i < MAX_COURBES; i++)
	{
		if( courbes_names[i] )
		{
			free( courbes_names[i]);
			courbes_names[i] = NULL;
		}
	}
}

void Graphique::reset_roi()
{
	roi_xmin = xmin;
	roi_xmax = xmax;
	roi_ymin = ymin;
	roi_ymax = ymax;

	update_data();
}

void Graphique::update_data()
{
	tics_dx = quantize(roi_xmax - roi_xmin);
	tics_dy = quantize(roi_ymax - roi_ymin);

	ratio_x = (roi_xmax - roi_xmin) / (screen_width - 2 * bordure_pixel_x);
	ratio_y = (roi_ymax - roi_ymin) / (screen_height - 2 * bordure_pixel_y);

	float bordure_x = bordure_pixel_x * ratio_x;
	float bordure_y = bordure_pixel_y * ratio_y;

	plot_xmin = roi_xmin - bordure_x;
	plot_xmax = roi_xmax + bordure_x;
	plot_ymin = roi_ymin - bordure_y;
	plot_ymax = roi_ymax + bordure_y;
}

float Graphique::quantize(float range)
{
	float power = pow(10, floor(log10(range)));
	float xnorm = range / power;
	float posns = 20 / xnorm;
	float tics;

	if (posns > 40)
	{
		tics = 0.05;
	}
	else if (posns > 20)
	{
		tics = 0.1;
	}
	else if (posns > 10)
	{
		tics = 0.2;
	}
	else if (posns > 4)
	{
		tics = 0.5;
	}
	else if (posns > 2)
	{
		tics = 1;
	}
	else if (posns > 0.5)
	{
		tics = 2;
	}
	else
	{
		tics = ceil(xnorm);
	}

	return tics * power;
}

void Graphique::resize_screen(int Screen_width, int Screen_height)
{
	screen_width = Screen_width;
	screen_height = Screen_height;

	update_data();
}

void Graphique::set_border(int Bordure_pixel_x, int Bordure_pixel_y)
{
	bordure_pixel_x = Bordure_pixel_x;
	bordure_pixel_y = Bordure_pixel_y;

	update_data();
}

void Graphique::resize_axis_x(float Xmin, float Xmax)
{
	int resetroi = 0;

	if( xmin == roi_xmin && xmax == roi_xmax )
	{
		resetroi = 1;
	}

	xmin = Xmin;
	xmax = Xmax;

	if(resetroi)
	{
		reset_roi();
	}

	update_data();
}

void Graphique::zoom(float mouse_x1, float mouse_x2, float mouse_y1, float mouse_y2)
{
	if(mouse_x1 > mouse_x2)
	{
		float tmp = mouse_x1;
		mouse_x1 = mouse_x2;
		mouse_x2 = tmp;
	}
	if(mouse_y1 > mouse_y2)
	{
		float tmp = mouse_y1;
		mouse_y1 = mouse_y2;
		mouse_y2 = tmp;
	}

	float zoom_x1 = (mouse_x1 - bordure_pixel_x) / (screen_width - 2 * bordure_pixel_x);
	float zoom_x2 = (mouse_x2 - bordure_pixel_x) / (screen_width - 2 * bordure_pixel_x);
	float zoom_y1 = (mouse_y1 - bordure_pixel_y) / (screen_height - 2 * bordure_pixel_y);
	float zoom_y2 = (mouse_y2 - bordure_pixel_y) / (screen_height - 2 * bordure_pixel_y);

	float xrange = roi_xmax - roi_xmin;
	float yrange = roi_ymax - roi_ymin;

	roi_xmin += zoom_x1 * xrange;
	roi_xmax -= (1-zoom_x2) * xrange;
	roi_ymin += zoom_y1 * yrange;
	roi_ymax -= (1-zoom_y2) * yrange;

	update_data();
}

void Graphique::zoomf(float zoom)
{
	float xrange = roi_xmax - roi_xmin;
	float yrange = roi_ymax - roi_ymin;

	float xmed = 0.5*(roi_xmax + roi_xmin);
	float ymed = 0.5*(roi_ymax + roi_ymin);

	roi_xmin = xmed - 0.5 * xrange * zoom;
	roi_xmax = xmed + 0.5 * xrange * zoom;
	roi_ymin = ymed - 0.5 * yrange * zoom;
	roi_ymax = ymed + 0.5 * yrange * zoom;

	update_data();
}
