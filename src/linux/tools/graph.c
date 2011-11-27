#include <math.h>

#include "linux/tools/graph.h"

static float graph_quantize(float range);

void graph_init(struct graph* graph, float xmin, float xmax, float ymin, float ymax)
{
	graph->xmin = xmin;
	graph->xmax = xmax;
	graph->ymin = ymin;
	graph->ymax = ymax;

	// par defaut roi = tout
	graph_reset_roi(graph);
}

void graph_reset_roi(struct graph* graph)
{
	graph->roi_xmin = graph->xmin;
	graph->roi_xmax = graph->xmax;
	graph->roi_ymin = graph->ymin;
	graph->roi_ymax = graph->ymax;

	graph->tics_dx = graph_quantize(graph->roi_xmax - graph->roi_xmin);
	graph->tics_dy = graph_quantize(graph->roi_ymax - graph->roi_ymin);
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

void graph_zoom(struct graph* graph, float zoom_x1, float zoom_x2, float zoom_y1, float zoom_y2)
{
	float xrange = graph->roi_xmax - graph->roi_xmin;
	float yrange = graph->roi_ymax - graph->roi_ymin;

	graph->roi_xmin += zoom_x1 * xrange;
	graph->roi_xmax -= (1-zoom_x2) * xrange;
	graph->roi_ymin += zoom_y1 * yrange;
	graph->roi_ymax -= (1-zoom_y2) * yrange;

	graph->tics_dx = graph_quantize(graph->roi_xmax - graph->roi_xmin);
	graph->tics_dy = graph_quantize(graph->roi_ymax - graph->roi_ymin);
}