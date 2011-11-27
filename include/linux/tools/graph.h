#ifndef GRAPH_H
#define GRAPH_H

struct graph
{
	float xmin;
	float xmax;
	float ymin;
	float ymax;
	float roi_xmin;
	float roi_xmax;
	float roi_ymin;
	float roi_ymax;
	float tics_dx;
	float tics_dy;
};

void graph_init(struct graph* graph, float xmin, float xmax, float ymin, float ymax);

void graph_reset_roi(struct graph* graph);

void graph_zoom(struct graph* graph, float zoom_x1, float zoom_x2, float zoom_y1, float zoom_y2);

#endif