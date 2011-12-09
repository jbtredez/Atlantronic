#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <gdk/gdkkeysyms.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>
#include <X11/Intrinsic.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#include "linux/tools/foo_interface.h"
#include "linux/tools/cmd.h"
#include "linux/tools/graph.h"
#include "kernel/robot_parameters.h"

// limitation du rafraichissement
// hokuyo => 10fps. On met juste un peu plus
#define MAX_FPS    11

enum
{
	GRAPH_TABLE = 0,
	GRAPH_HOKUYO_HIST,
	GRAPH_SPEED_DIST,
	GRAPH_NUM,
};

enum
{
	SUBGRAPH_TABLE_POS_ROBOT = 0,
	SUBGRAPH_TABLE_HOKUYO_FOO,
	SUBGRAPH_TABLE_HOKUYO_BAR,
	SUBGRAPH_TABLE_POS_CONS,
	SUBGRAPH_TABLE_POS_MES,
	SUBGRAPH_TABLE_NUM,
};

enum
{
	GRAPH_HOKUYO_HIST_FOO = 0,
	GRAPH_HOKUYO_HIST_BAR,
	GRAPH_HOKUYO_HIST_NUM,
};

enum
{
	SUBGRAPH_SPEED_DIST_MES = 0,
	SUBGRAPH_SPEED_DIST_CONS,
	SUBGRAPH_SPEED_DIST_NUM,
};

static GLuint font_base;
static char font_name[] = "fixed";
static int font_height = 0;
static int font_digit_height = 0;
static int font_width = 0;
static XFontStruct* font_info = NULL;
static int screen_width = 0;
static int screen_height = 0;
static struct foo_interface foo;
static float mouse_x1 = 0;
static float mouse_y1 = 0;
static float mouse_x2 = 0;
static float mouse_y2 = 0;
static int drawing_zoom_selection = 0;
static int current_graph = GRAPH_TABLE;
static GtkWidget* opengl_window;

struct graph graph[GRAPH_NUM];

static void close_gtk(GtkWidget* widget, gpointer arg);
static void select_graph(GtkWidget* widget, gpointer arg);
static void select_active_courbe(GtkWidget* widget, gpointer arg);
static void init(GtkWidget* widget, gpointer arg);
static gboolean config(GtkWidget* widget, GdkEventConfigure* ev, gpointer arg);
static gboolean afficher(GtkWidget* widget, GdkEventExpose* ev, gpointer arg);
static void mounse_press(GtkWidget* widget, GdkEventButton* event);
static void mounse_release(GtkWidget* widget, GdkEventButton* event);
static gboolean keyboard_press(GtkWidget* widget, GdkEventKey* event, gpointer arg);
static gboolean keyboard_release(GtkWidget* widget, GdkEventKey* event, gpointer arg);
static void mouse_move(GtkWidget* widget, GdkEventMotion* event);
static int init_font(GLuint base, char* f);
static void draw_plus(float x, float y, float rx, float ry);
static void glPrintf_xright2_ycenter(float x, float y, float x_ratio, float y_ratio, GLuint base, char* s, ...) __attribute__(( format(printf, 6, 7) ));
static void glPrintf_xcenter_yhigh2(float x, float y, float x_ratio, float y_ratio, GLuint base, char* s, ...) __attribute__(( format(printf, 6, 7) ));
static void glprint(float x, float y, GLuint base, char* buffer, int size);
void read_callback();

int main(int argc, char *argv[])
{
	long i = 0;
	long j = 0;

	if(argc < 2)
	{
		printf("indiquer le peripherique\n");
		return -1;
	}

	if( ! g_thread_supported() )
	{
		g_thread_init(NULL);
	}
	else
	{
		fprintf(stderr, "pas de support des g_thread, risque de bug (non prévu et non testé) - abandon");
		return 0;
	}

	graph_init(&graph[GRAPH_TABLE], "Table", -1500, 1500, -1000, 1000, 800, 600, 0, 0);
	graph_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_POS_ROBOT, "Robot", 1);
	graph_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_HOKUYO_FOO, "Hokuyo foo", 1);
	graph_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_HOKUYO_BAR, "Hokuyo bar", 1);
	graph_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_POS_CONS, "Position (consigne)", 1);
	graph_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_POS_MES, "Position (mesure)", 1);

	graph_init(&graph[GRAPH_HOKUYO_HIST], "Hokuyo (histograme)", 0, 682, 0, 4100, 800, 600, 0, 0);
	graph_add_courbe(&graph[GRAPH_HOKUYO_HIST], GRAPH_HOKUYO_HIST_FOO, "Hokuyo foo", 1);
	graph_add_courbe(&graph[GRAPH_HOKUYO_HIST], GRAPH_HOKUYO_HIST_BAR, "Hokuyo bar", 1);

	graph_init(&graph[GRAPH_SPEED_DIST], "Vitesses (avance)", 0, 90000, -1500, 1500, 800, 600, 0, 0);
	graph_add_courbe(&graph[GRAPH_SPEED_DIST], SUBGRAPH_SPEED_DIST_MES, "Vitesse d'avance mesuree", 1);
	graph_add_courbe(&graph[GRAPH_SPEED_DIST], SUBGRAPH_SPEED_DIST_CONS, "Vitesse d'avance de consigne", 1);

	gdk_threads_init();
	gdk_threads_enter();

	gtk_init(&argc, &argv);
	gtk_gl_init(&argc, &argv);

	// config de opengl
	GdkGLConfig* glconfig = gdk_gl_config_new_by_mode((GdkGLConfigMode) (GDK_GL_MODE_RGB | GDK_GL_MODE_DEPTH | GDK_GL_MODE_DOUBLE));
	if(glconfig == NULL)
	{
		fprintf(stderr, "double buffer non géré");
		glconfig = gdk_gl_config_new_by_mode((GdkGLConfigMode)(GDK_GL_MODE_RGB | GDK_GL_MODE_DEPTH));
		if(glconfig == NULL)
		{
			fprintf(stderr, "opengl non géré, abandon");
			return 0;
		}
	}

	// création de la fenêtre
	GtkWidget* main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title(GTK_WINDOW(main_window),"USB Interface");
	gtk_window_set_default_size(GTK_WINDOW(main_window), 400, 300);

	g_signal_connect(G_OBJECT(main_window), "delete_event", G_CALLBACK(gtk_main_quit), NULL);
	gtk_signal_connect(GTK_OBJECT(main_window), "destroy", GTK_SIGNAL_FUNC(close_gtk), NULL);

	// fenetre opengl
	opengl_window = gtk_drawing_area_new();
	gtk_widget_set_size_request(opengl_window, 800, 600);
	gtk_widget_set_gl_capability(opengl_window, glconfig, NULL, TRUE, GDK_GL_RGBA_TYPE);

	gtk_widget_add_events(opengl_window, GDK_VISIBILITY_NOTIFY_MASK | GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK);

	g_signal_connect_after(G_OBJECT(opengl_window), "realize", G_CALLBACK(init), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "configure_event", G_CALLBACK(config), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "expose_event", G_CALLBACK(afficher), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "button_press_event", G_CALLBACK(mounse_press), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "button_release_event", G_CALLBACK(mounse_release), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "motion_notify_event", G_CALLBACK(mouse_move), NULL);
	g_signal_connect_swapped(G_OBJECT(main_window), "key_press_event", G_CALLBACK(keyboard_press), opengl_window);
	g_signal_connect_swapped(G_OBJECT(main_window), "key_release_event", G_CALLBACK(keyboard_release), opengl_window);


	GtkWidget* menu0 = gtk_menu_bar_new();	// menu "niveau 0" (ie: barre de menu)
	GtkWidget* menu1 = gtk_menu_new();	// menu "niveau 1"
	GtkWidget* menuObj;

	// menu Fichier
	menuObj = gtk_menu_item_new_with_label("Fichier");
	gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu0), menuObj);

	menuObj = gtk_menu_item_new_with_label("Quitter");
	g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(close_gtk), (GtkWidget*) main_window);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);


	// menu courbe
	menu1 = gtk_menu_new();	// menu "niveau 1"
	menuObj = gtk_menu_item_new_with_label("Courbe");
	gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu0), menuObj);

	GSList *group = NULL;
	for( i = 0; i < GRAPH_NUM; i++)
	{
		menuObj = gtk_radio_menu_item_new_with_label(group, graph[i].name);
		group = gtk_radio_menu_item_get_group (GTK_RADIO_MENU_ITEM (menuObj));
		g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(select_graph), (void*)i);
		gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);
		if( i == current_graph )
		{
			gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM (menuObj), TRUE);
		}
	}

	for( i = 0 ; i < GRAPH_NUM; i++)
	{
		menu1 = gtk_menu_new();	// menu "niveau 1"
		menuObj = gtk_menu_item_new_with_label( graph[i].name);
		gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);
		gtk_menu_shell_append(GTK_MENU_SHELL(menu0), menuObj);
		for( j = 0; j < MAX_COURBES; j++)
		{
			char* name = graph[i].courbes_names[j];
			if( name )
			{
				menuObj = gtk_check_menu_item_new_with_label(name);
				gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(menuObj), graph[i].courbes_activated[j]);
				g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(select_active_courbe), &graph[i].courbes_activated[j]);
				gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);
			}
		}
	}

	// rangement des éléments dans la fenetre
	// vbox la fenetre principale : menu + fenetre opengl
	GtkWidget* main_vbox = gtk_vbox_new(FALSE, 0);
	gtk_container_add(GTK_CONTAINER(main_window), main_vbox);

	gtk_box_pack_start(GTK_BOX(main_vbox), menu0, FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(main_vbox), opengl_window, TRUE, TRUE, 0);

	gtk_widget_show_all(main_window);

	foo_interface_init(&foo, argv[1], read_callback, opengl_window);
	cmd_init(&foo.com);

	gtk_main();

	gdk_threads_leave();

	foo_interface_destroy(&foo);

	return 0;
}

void read_callback(GtkWidget* widget)
{
	static struct timespec last_plot = {0, 0};
	struct timespec current;

	clock_gettime(CLOCK_MONOTONIC, &current);
	double delta = (current.tv_sec - last_plot.tv_sec) + (current.tv_nsec - last_plot.tv_nsec) / ((double)1000000000.0f);
	if(delta >= 1.0f/MAX_FPS)
	{
		gdk_threads_enter();
		if(widget->window)
		{
			gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
		}
		gdk_threads_leave();
		last_plot = current;
	}
}

static void close_gtk(GtkWidget* widget, gpointer arg)
{
	(void) widget;
	(void) arg;
	gtk_main_quit();
}

static void select_graph(GtkWidget* widget, gpointer arg)
{
	(void) widget;

	unsigned long id = (unsigned long) arg;
	if(id < GRAPH_NUM)
	{
		current_graph = id;
	}
	gdk_window_invalidate_rect(opengl_window->window, &opengl_window->allocation, FALSE);
}

static void select_active_courbe(GtkWidget* widget, gpointer arg)
{
	(void) widget;
	int* activated = (int*) arg;


	if(*activated)
	{
		*activated = 0;
	}
	else
	{
		*activated = 1;
	}

	gdk_window_invalidate_rect(opengl_window->window, &opengl_window->allocation, FALSE);
}

static void init(GtkWidget* widget, gpointer arg)
{
	(void) arg;
	int i;

	GdkGLContext* glcontext = gtk_widget_get_gl_context(widget);
	GdkGLDrawable* gldrawable = gtk_widget_get_gl_drawable(widget);

	if(!gdk_gl_drawable_gl_begin(gldrawable, glcontext)) return;

	font_base = glGenLists(256);
	if (!glIsList(font_base))
	{
		fprintf(stderr, "my_init(): Out of display lists. - Exiting.\n");
		exit(-1);
	}
	else
	{
		int res = init_font(font_base, font_name);
		if(res != 0)
		{
			fprintf(stderr, "font error - Exiting.\n");
			exit(-1);
		}
	}

	for(i = 0; i < GRAPH_NUM; i++)
	{
		graph_set_border(&graph[i], 10 * font_width, font_height*3);
	}

	gdk_gl_drawable_gl_end(gldrawable);
}

static gboolean config(GtkWidget* widget, GdkEventConfigure* ev, gpointer arg)
{
	(void) ev;
	(void) arg;

	GdkGLContext* glcontext = gtk_widget_get_gl_context(widget);
	GdkGLDrawable* gldrawable = gtk_widget_get_gl_drawable(widget);

	if(!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
	{
		return FALSE;
	}

	screen_width = widget->allocation.width;
	screen_height = widget->allocation.height;

	graph_resize_screen(&graph[current_graph], screen_width, screen_height);

	glViewport(0, 0, screen_width, screen_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, screen_width, screen_height, 0, 0, 1);

	glDisable(GL_DEPTH_TEST);

	gdk_gl_drawable_gl_end(gldrawable);

	return TRUE;
}

static void draw_plus(float x, float y, float rx, float ry)
{
	glBegin(GL_LINES);
	glVertex2f(x-rx, y);
	glVertex2f(x+rx, y);
	glVertex2f(x, y-ry);
	glVertex2f(x, y+ry);
	glEnd();
}

void plot_axes(struct graph* graph)
{
	float roi_xmin = graph->roi_xmin;
	float roi_xmax = graph->roi_xmax;
	float roi_ymin = graph->roi_ymin;
	float roi_ymax = graph->roi_ymax;

	float ratio_x = graph->ratio_x;
	float ratio_y = graph->ratio_y;

	glBegin(GL_LINE_STRIP);
	glVertex2f(roi_xmax, roi_ymin);
	glVertex2f(roi_xmin, roi_ymin);
	glVertex2f(roi_xmin, roi_ymax);
	glEnd();

	// axe x
	float dx = graph->tics_dx;
	float x;
	for(x = 0; x <= roi_xmax; x+=dx)
	{
		draw_plus(x, roi_ymin, font_width*ratio_x, font_width*ratio_y);
		glPrintf_xcenter_yhigh2(x, roi_ymin, ratio_x, ratio_y, font_base, "%g", x);
	}
	for(x = -dx; x >= roi_xmin; x-=dx)
	{
		draw_plus(x, roi_ymin, font_width*ratio_x, font_width*ratio_y);
		glPrintf_xcenter_yhigh2(x, roi_ymin, ratio_x, ratio_y, font_base, "%g", x);
	}

	// axe y
	float dy = graph->tics_dy;
	float y;
	for(y = 0; y <= roi_ymax; y+=dy)
	{
		draw_plus(roi_xmin, y, font_width*ratio_x, font_width*ratio_y);
		glPrintf_xright2_ycenter(roi_xmin, y, ratio_x, ratio_y, font_base, "%g", y);
	}
	for(y = -dy; y >= roi_ymin; y-=dy)
	{
		draw_plus(roi_xmin, y, font_width*ratio_x, font_width*ratio_y);
		glPrintf_xright2_ycenter(roi_xmin, y, ratio_x, ratio_y, font_base, "%g", y);
	}
}

void plot_table(struct graph* graph)
{
	float ratio_x = graph->ratio_x;
	float ratio_y = graph->ratio_y;

	struct vect_pos pos_hokuyo = {0, 0, 0, 1, 0};
	struct vect_pos pos_table = {0, 0, 0, 1, 0};

	int i;

	if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO_FOO] )
	{
		glColor3f(1,0,0);
		for(i=0; i < HOKUYO_NUM_POINTS; i++)
		{
			pos_hokuyo.x = foo.hokuyo_x[i];
			pos_hokuyo.y = foo.hokuyo_y[i];
			pos_hokuyo_to_table(&foo.hokuyo_scan[HOKUYO_FOO].pos_robot, &foo.hokuyo_scan[HOKUYO_FOO].pos_hokuyo, &pos_hokuyo, &pos_table);
			draw_plus(pos_table.x, pos_table.y, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO_BAR] )
	{
		glColor3f(0.5,0.5,0);
		for(i=HOKUYO_NUM_POINTS; i < 2*HOKUYO_NUM_POINTS; i++)
		{
			pos_hokuyo.x = foo.hokuyo_x[i];
			pos_hokuyo.y = foo.hokuyo_y[i];
			pos_hokuyo_to_table(&foo.hokuyo_scan[HOKUYO_FOO_BAR].pos_robot, &foo.hokuyo_scan[HOKUYO_FOO_BAR].pos_hokuyo, &pos_hokuyo, &pos_table);
			draw_plus(pos_table.x, pos_table.y, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	int max = foo.control_usb_data_count % CONTROL_USB_DATA_MAX;

	if( graph->courbes_activated[SUBGRAPH_TABLE_POS_CONS] )
	{
		glColor3f(0,0,1);
		for(i=0; i< max; i++)
		{
			if(foo.control_usb_data[i].control_state != CONTROL_READY_ASSER && foo.control_usb_data[i].control_state != CONTROL_READY_FREE)
			{
				draw_plus(foo.control_usb_data[i].control_cons_x, foo.control_usb_data[i].control_cons_y, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
			}
		}
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_POS_MES] )
	{
		glColor3f(0,1,0);
		for(i=0; i < max; i++)
		{
			draw_plus(foo.control_usb_data[i].control_pos_x, foo.control_usb_data[i].control_pos_y, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	// affichage du repère robot
	if( graph->courbes_activated[SUBGRAPH_TABLE_POS_ROBOT] )
	{
		glColor3f(0,0,0);
		struct vect_pos pos_robot;
		pos_robot.x = foo.control_usb_data[max-1].control_pos_x;
		pos_robot.y = foo.control_usb_data[max-1].control_pos_y;
		pos_robot.alpha = foo.control_usb_data[max-1].control_pos_alpha;
		pos_robot.ca = cos(pos_robot.alpha);
		pos_robot.sa = sin(pos_robot.alpha);

		glPushMatrix();
		glTranslatef(pos_robot.x, pos_robot.y, 0);
		glRotatef(pos_robot.alpha * 180 / M_PI, 0, 0, 1);
		glBegin(GL_LINES);
		glVertex2f(0, 0);
		glVertex2f(5 * font_height, 0);
		glVertex2f(0, 0);
		glVertex2f(0, 5 * font_height);
		glEnd();

		glColor3f(1,1,0);
		glBegin(GL_LINE_STRIP);
		glVertex2f(PARAM_NP_X, -150);
		glVertex2f(PARAM_NP_X, 150);
		glVertex2f(300 + PARAM_NP_X, 150);
		glVertex2f(300 + PARAM_NP_X, -150);
		glVertex2f(PARAM_NP_X, -150);
		glEnd();
		glPopMatrix();
	}
}

void plot_hokuyo_hist(struct graph* graph)
{
	int i;

	float ratio_x = graph->ratio_x;
	float ratio_y = graph->ratio_y;

	if( graph->courbes_activated[GRAPH_HOKUYO_HIST_FOO] )
	{
		glColor3f(1,0,0);
		for(i = 0; i < HOKUYO_NUM_POINTS; i++)
		{
			draw_plus(i, foo.hokuyo_scan[HOKUYO_FOO].distance[i], 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}
	if( graph->courbes_activated[GRAPH_HOKUYO_HIST_BAR] )
	{
		glColor3f(0.5,0.5,0);
		for(i = 0; i < HOKUYO_NUM_POINTS; i++)
		{
			draw_plus(i, foo.hokuyo_scan[HOKUYO_FOO_BAR].distance[i], 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}
}

void plot_speed_dist(struct graph* graph)
{
	int i;

	float ratio_x = graph->ratio_x;
	float ratio_y = graph->ratio_y;

	if( graph->courbes_activated[SUBGRAPH_SPEED_DIST_CONS] )
	{
		glColor3f(0,0,1);
		for(i=0; i < foo.control_usb_data_count; i++)
		{
			draw_plus(5*i, foo.control_usb_data[i].control_v_dist_cons*200, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_SPEED_DIST_MES] )
	{
		glColor3f(0,1,0);
		for(i=1; i < foo.control_usb_data_count; i++)
		{
			draw_plus(5*i, foo.control_usb_data[i].control_v_dist_mes*200, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}
}

static gboolean afficher(GtkWidget* widget, GdkEventExpose* ev, gpointer arg)
{
	(void) ev;
	(void) arg;
	GdkGLContext* glcontext = gtk_widget_get_gl_context(widget);
	GdkGLDrawable* gldrawable = gtk_widget_get_gl_drawable(widget);

	if(!gdk_gl_drawable_gl_begin(gldrawable, glcontext))
	{
		return FALSE;
	}

	// efface le frame buffer
	glClearColor(1,1,1,1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, screen_width, screen_height, 0, 0, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glColor3f(0,0,0);

	if( drawing_zoom_selection )
	{
		glBegin(GL_LINE_STRIP);
		glVertex2f(mouse_x1, mouse_y1);
		glVertex2f(mouse_x2, mouse_y1);
		glVertex2f(mouse_x2, mouse_y2);
		glVertex2f(mouse_x1, mouse_y2);
		glVertex2f(mouse_x1, mouse_y1);
		glEnd();
	}

	int res = pthread_mutex_lock(&foo.mutex);
	if(res == 0)
	{
		if(current_graph == GRAPH_SPEED_DIST)
		{
			graph_resize_axis_x(&graph[current_graph], 0, foo.control_usb_data_count * 5);
		}

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(graph[current_graph].plot_xmin, graph[current_graph].plot_xmax, graph[current_graph].plot_ymin, graph[current_graph].plot_ymax, 0, 1);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		glColor3f(0,0,0);
		plot_axes(&graph[current_graph]);

		switch(current_graph)
		{
			default:
			case GRAPH_TABLE:
				plot_table(&graph[current_graph]);
				break;
			case GRAPH_HOKUYO_HIST:
				plot_hokuyo_hist(&graph[current_graph]);
				break;
			case GRAPH_SPEED_DIST:
				plot_speed_dist(&graph[current_graph]);
				break;
		}
		pthread_mutex_unlock(&foo.mutex);
	}

	if(gdk_gl_drawable_is_double_buffered(gldrawable))
	{
		gdk_gl_drawable_swap_buffers(gldrawable);
	}
	else
	{
		glFlush();
	}

	gdk_gl_drawable_gl_end(gldrawable);

	return TRUE;
}

static int init_font(GLuint base, char* f)
{
	Display* display;
	int first;
	int last;

	// Need an X Display before calling any Xlib routines
	display = XOpenDisplay(0);
	if (display == 0)
	{
		fprintf(stderr, "XOpenDisplay() failed\n");
		return -1;
	}

	// Load the font
	font_info = XLoadQueryFont(display, f);
	if (!font_info)
	{
		fprintf(stderr, "XLoadQueryFont() failed\n");
		return -1;
	}

	// Tell GLX which font & glyphs to use
	first = font_info->min_char_or_byte2;
	last  = font_info->max_char_or_byte2;
	glXUseXFont(font_info->fid, first, last-first+1, base+first);

	font_height = font_info->ascent;
	font_digit_height = font_info->per_char['0'].ascent; // on prend la hauteur de 0
	font_width = font_info->max_bounds.width;
	XCloseDisplay(display);

	return 0;
}

static void mounse_press(GtkWidget* widget, GdkEventButton* event)
{
	if(event->button == 1)
	{
		drawing_zoom_selection = 1;
		mouse_x1 = event->x;
		mouse_y1 = event->y;
		mouse_x2 = mouse_x1;
		mouse_y2 = mouse_y1;
	}
	else
	{
		graph_reset_roi(&graph[current_graph]);
	}
	gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
}

static void mounse_release(GtkWidget* widget, GdkEventButton* event)
{
	if(event->button == 1)
	{
		if( drawing_zoom_selection && mouse_x1 != mouse_x2 && mouse_y1 != mouse_y2)
		{
			graph_zoom(&graph[current_graph], mouse_x1, mouse_x2, screen_height - mouse_y1, screen_height - mouse_y2);
		}

		drawing_zoom_selection = 0;
		mouse_x1 = 0;
		mouse_y1 = 0;
		mouse_x2 = 0;
		mouse_y2 = 0;
		gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
	}
}

static void mouse_move(GtkWidget* widget, GdkEventMotion* event)
{
	if(event->state & GDK_BUTTON1_MASK)
	{
		mouse_x2 = event->x;
		mouse_y2 = event->y;
		gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
	}
}

static gboolean keyboard_press(GtkWidget* widget, GdkEventKey* event, gpointer arg)
{
	(void) arg;
	switch(event->keyval)
	{
		case GDK_Escape:
			drawing_zoom_selection = 0;
			break;
		case GDK_u:
			graph_reset_roi(&graph[GRAPH_TABLE]);
			break;
	}

	gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);

	return TRUE;
}

static gboolean keyboard_release(GtkWidget* widget, GdkEventKey* event, gpointer arg)
{
	(void) widget;
	(void) event;
	(void) arg;
	return TRUE;
}

static void glprint(float x, float y, GLuint base, char* buffer, int size)
{
	if( size != 0)
	{
		glRasterPos2f(x, y);
		glPushAttrib(GL_LIST_BIT);
		glListBase(base);
		glCallLists(size, GL_UNSIGNED_BYTE, (GLubyte *)buffer);
		glPopAttrib();
	}
}

static void glPrintf_xright2_ycenter(float x, float y, float x_ratio,float y_ratio, GLuint base, char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x - x_ratio * (size+2) * font_width, y - font_digit_height / 2.0f * y_ratio, base, buffer, size);
}

static void glPrintf_xcenter_yhigh2(float x, float y, float x_ratio,float y_ratio, GLuint base, char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x - x_ratio * size/2.0f * font_width, y - 2*font_digit_height * y_ratio, base, buffer, size);
}
