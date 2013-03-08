#ifndef GRAPHIQUE_H
#define GRAPHIQUE_H

#define MAX_COURBES        10

struct graphique
{
	char* name;
	char* courbes_names[MAX_COURBES];
	int courbes_activated[MAX_COURBES];
	float color[3*MAX_COURBES];
	float xmin; //!< min sur l'axe x du graphique (sans zoom)
	float xmax; //!< max sur l'axe x du graphique (sans zoom)
	float ymin; //!< min sur l'axe y du graphique (sans zoom)
	float ymax; //!< max sur l'axe y du graphique (sans zoom)

	float roi_xmin; //!< min sur l'axe x du graphique (avec zoom)
	float roi_xmax; //!< max sur l'axe x du graphique (avec zoom)
	float roi_ymin; //!< min sur l'axe y du graphique (avec zoom)
	float roi_ymax; //!< max sur l'axe y du graphique (avec zoom)

	float plot_xmin; //!< min sur l'axe x du graphique (avec zoom + bordure)
	float plot_xmax; //!< max sur l'axe x du graphique (avec zoom + bordure)
	float plot_ymin; //!< min sur l'axe y du graphique (avec zoom + bordure)
	float plot_ymax; //!< max sur l'axe y du graphique (avec zoom + bordure)

	float tics_dx; //!< delta entre 2 points de graduation de l'axe x
	float tics_dy; //!< delta entre 2 points de graduation de l'axe y

	int screen_width; //!< taille de l'écran en pixel selon l'axe x
	int screen_height; //!< taille de l'écran en pixel selon l'axe y
	int bordure_pixel_x;
	int bordure_pixel_y;

	float ratio_x; //!< ratio selon l'axe x en unite par pixel
	float ratio_y; //!< ratio selon l'axe y en unite par pixel
};

void graphique_init(struct graphique* graph, const char* name, float xmin, float xmax, float ymin, float ymax, int screen_width, int screen_height, int bordure_pixel_x, int bordure_pixel_y);

void graphique_set_border(struct graphique* graph, int bordure_pixel_x, int bordure_pixel_y);

void graphique_add_courbe(struct graphique* graph, int id, const char* name, int activated, float r, float g, float b);

void graphique_destroy(struct graphique* graph);

void graphique_reset_roi(struct graphique* graph);

void graphique_resize_screen(struct graphique* graph, int screen_width, int screen_height);

void graphique_resize_axis_x(struct graphique* graph, float xmin, float xmax);

void graphique_zoom(struct graphique* graph, float mouse_x1, float mouse_x2, float mouse_y1, float mouse_y2);

void graphique_zoomf(struct graphique* graph, float zoom);

#endif
