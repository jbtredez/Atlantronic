#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "linux/tools/graph.h"

static float graph_quantize(float range);
static void graph_update_data(struct graph* graph);

void graph_init(struct graph* graph, const char* name, float xmin, float xmax, float ymin, float ymax, int screen_width, int screen_height, int bordure_pixel_x, int bordure_pixel_y)
{
	int i = 0;

	graph->name = malloc(strlen(name)+1);
	strcpy(graph->name, name);

	for(i = 0; i < MAX_COURBES; i++)
	{
		graph->courbes_names[i] = NULL;
		graph->courbes_activated[i] = 0;
	}

	memset(graph->color, 0x00, sizeof(graph->color));

	graph->xmin = xmin;
	graph->xmax = xmax;
	graph->ymin = ymin;
	graph->ymax = ymax;

	graph->screen_width = screen_width;
	graph->screen_height = screen_height;
	graph->bordure_pixel_x = bordure_pixel_x;
	graph->bordure_pixel_y = bordure_pixel_y;

	// par defaut roi = tout
	graph_reset_roi(graph);
	graph_update_data(graph);
}

void graph_add_courbe(struct graph* graph, int id, const char* name, int activated, float r, float g, float b)
{
	if(id >= MAX_COURBES || id < 0)
	{
		return;
	}

	if(graph->courbes_names[id])
	{
		free(graph->courbes_names[id]);
	}

	graph->color[3*id] = r;
	graph->color[3*id+1] = g;
	graph->color[3*id+2] = b;
	graph->courbes_activated[id] = activated;
	graph->courbes_names[id] = malloc(strlen(name)+1);
	strcpy(graph->courbes_names[id], name);
}

void graph_destroy(struct graph* graph)
{
	int i;
	if(graph->name)
	{
		free(graph->name);
		graph->name = NULL;
	}

	for(i = 0; i < MAX_COURBES; i++)
	{
		if( graph->courbes_names[i] )
		{
			free( graph->courbes_names[i]);
			graph->courbes_names[i] = NULL;
		}
	}
}

void graph_reset_roi(struct graph* graph)
{
	graph->roi_xmin = graph->xmin;
	graph->roi_xmax = graph->xmax;
	graph->roi_ymin = graph->ymin;
	graph->roi_ymax = graph->ymax;

	graph_update_data(graph);
}

static void graph_update_data(struct graph* graph)
{
	graph->tics_dx = graph_quantize(graph->roi_xmax - graph->roi_xmin);
	graph->tics_dy = graph_quantize(graph->roi_ymax - graph->roi_ymin);

	graph->ratio_x = (graph->roi_xmax - graph->roi_xmin) / (graph->screen_width - 2 * graph->bordure_pixel_x);
	graph->ratio_y = (graph->roi_ymax - graph->roi_ymin) / (graph->screen_height - 2 * graph->bordure_pixel_y);

	float bordure_x = graph->bordure_pixel_x * graph->ratio_x;
	float bordure_y = graph->bordure_pixel_y * graph->ratio_y;

	graph->plot_xmin = graph->roi_xmin - bordure_x;
	graph->plot_xmax = graph->roi_xmax + bordure_x;
	graph->plot_ymin = graph->roi_ymin - bordure_y;
	graph->plot_ymax = graph->roi_ymax + bordure_y;
}

static float graph_quantize(float range)
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

void graph_resize_screen(struct graph* graph, int screen_width, int screen_height)
{
	graph->screen_width = screen_width;
	graph->screen_height = screen_height;

	graph_update_data(graph);
}

void graph_set_border(struct graph* graph, int bordure_pixel_x, int bordure_pixel_y)
{
	graph->bordure_pixel_x = bordure_pixel_x;
	graph->bordure_pixel_y = bordure_pixel_y;

	graph_update_data(graph);
}

void graph_resize_axis_x(struct graph* graph, float xmin, float xmax)
{
	int reset_roi = 0;

	if( graph->xmin == graph->roi_xmin && graph->xmax == graph->roi_xmax )
	{
		reset_roi = 1;
	}

	graph->xmin = xmin;
	graph->xmax = xmax;

	if(reset_roi)
	{
		graph_reset_roi(graph);
	}

	graph_update_data(graph);
}

void graph_zoom(struct graph* graph, float mouse_x1, float mouse_x2, float mouse_y1, float mouse_y2)
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

	float zoom_x1 = (mouse_x1 - graph->bordure_pixel_x) / (graph->screen_width - 2 * graph->bordure_pixel_x);
	float zoom_x2 = (mouse_x2 - graph->bordure_pixel_x) / (graph->screen_width - 2 * graph->bordure_pixel_x);
	float zoom_y1 = (mouse_y1 - graph->bordure_pixel_y) / (graph->screen_height - 2 * graph->bordure_pixel_y);
	float zoom_y2 = (mouse_y2 - graph->bordure_pixel_y) / (graph->screen_height - 2 * graph->bordure_pixel_y);

	float xrange = graph->roi_xmax - graph->roi_xmin;
	float yrange = graph->roi_ymax - graph->roi_ymin;

	graph->roi_xmin += zoom_x1 * xrange;
	graph->roi_xmax -= (1-zoom_x2) * xrange;
	graph->roi_ymin += zoom_y1 * yrange;
	graph->roi_ymax -= (1-zoom_y2) * yrange;

	graph_update_data(graph);
}