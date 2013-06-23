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
#include <sys/stat.h>

#define STM32F10X_CL
#include "linux/tools/robot_interface.h"
#include "linux/tools/qemu.h"
#include "linux/tools/cmd.h"
#include "linux/tools/graphique.h"
#include "linux/tools/joystick.h"
#include "kernel/robot_parameters.h"
#include "kernel/math/fx_math.h"
#include "foo/pwm.h"
#include "foo/graph.h"
#include "foo/table.h"

// limitation du rafraichissement
// hokuyo => 10fps. On met juste un peu plus
#define MAX_FPS    11
#define QEMU_OPPONENT_ID   (TABLE_OBJ_SIZE+1)
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
	SUBGRAPH_TABLE_STATIC_ELM,
	SUBGRAPH_TABLE_HOKUYO_FOO,
	SUBGRAPH_TABLE_HOKUYO_BAR,
	SUBGRAPH_TABLE_HOKUYO_FOO_SEG,
	SUBGRAPH_TABLE_POS_CONS,
	SUBGRAPH_TABLE_POS_MES,
	SUBGRAPH_TABLE_GRAPH,
	SUBGRAPH_TABLE_GRAPH_LINK,
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
	SUBGRAPH_CONTROL_SPEED_DIST_MES = 0,
	SUBGRAPH_CONTROL_SPEED_DIST_CONS,
	SUBGRAPH_CONTROL_SPEED_ROT_MES,
	SUBGRAPH_CONTROL_SPEED_ROT_CONS,
	SUBGRAPH_CONTROL_PWM_RIGHT,
	SUBGRAPH_CONTROL_PWM_LEFT,
	SUBGRAPH_CONTROL_I_RIGHT,
	SUBGRAPH_CONTROL_I_LEFT,
	SUBGRAPH_CONTROL_NUM,
};

static GLuint font_base;
static char font_name[] = "fixed";
static int font_height = 0;
static int font_digit_height = 0;
static int font_width = 0;
static XFontStruct* font_info = NULL;
static int screen_width = 0;
static int screen_height = 0;
static struct robot_interface robot_interface;
static struct qemu qemu;
static float mouse_x1 = 0;
static float mouse_y1 = 0;
static float mouse_x2 = 0;
static float mouse_y2 = 0;
static int drawing_zoom_selection = 0;
static int move_oponent_robot = 0;
static int current_graph = GRAPH_TABLE;
static GtkWidget* opengl_window;
static GtkWidget* main_window;

struct graphique graph[GRAPH_NUM];
struct joystick joystick;

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
static void joystick_event(int event, float val);
static void mouse_move(GtkWidget* widget, GdkEventMotion* event);
static int init_font(GLuint base, char* f);
static void draw_plus(float x, float y, float rx, float ry);
static void glPrintf_xright2_ycenter(float x, float y, float x_ratio, float y_ratio, GLuint base, char* s, ...) __attribute__(( format(printf, 6, 7) ));
static void glPrintf_xright2_yhigh(float x, float y, float x_ratio, float y_ratio, GLuint base, char* s, ...) __attribute__(( format(printf, 6, 7) ));
static void glPrintf_xcenter_yhigh2(float x, float y, float x_ratio, float y_ratio, GLuint base, char* s, ...) __attribute__(( format(printf, 6, 7) ));
static void glPrintf_xcenter_ycenter(float x, float y, float x_ratio, float y_ratio, GLuint base, char* s, ...) __attribute__(( format(printf, 6, 7) ));
static void glprint(float x, float y, GLuint base, char* buffer, int size);
void gtk_end();

void read_callback();

struct fx_vect2 oponent_robot_pt[5] =
{
	{ 100 << 16, 100 << 16},
	{ -100 << 16, 100 << 16},
	{ -100 << 16, -100 << 16},
	{ 100 << 16, -100 << 16},
	{ 100 << 16, 100 << 16},
};

struct polyline oponent_robot =
{
		oponent_robot_pt,
		5
};

struct fx_vect_pos opponent_robot_pos = {1800 << 16, 800 << 16, 0, 1<<30, 0};

int main(int argc, char *argv[])
{
	long i = 0;
	long j = 0;

	const char* file_foo_read = NULL;
	const char* file_foo_write = NULL;
	const char* file_bar_read = NULL;
	const char* file_bar_write = NULL;

	int simulation = 0;
	const char* prog_foo = NULL;
	int gdb_port = 0;

	if(argc > 1)
	{
		int option = -1;
		while( (option = getopt(argc, argv, "s:g")) != -1)
		{
			switch(option)
			{
				case 's':
					simulation = 1;
					prog_foo = optarg;
					break;
				case 'g':
					gdb_port = 1235;
					break;
				default:
					fprintf(stderr, "option %c inconnue", (char)option);
					return -1;
					break;
			}
		}
	}

	if( argc - optind > 0)
	{
		file_foo_read = argv[optind];
		file_foo_write = file_foo_read;
	}

	if(argc - optind > 1)
	{
		file_bar_read = argv[optind+1];
		file_bar_write = file_bar_read;
	}

	if(simulation)
	{
		int res = qemu_init(&qemu, prog_foo, gdb_port);
		if( res )
		{
			fprintf(stderr, "qemu_init : error");
			return -1;
		}

		file_foo_read = qemu.file_foo_read;
		file_foo_write = qemu.file_foo_write;
	}

	graphique_init(&graph[GRAPH_TABLE], "Table", -1600, 1600, -1100, 1100, 800, 600, 0, 0);
	graphique_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_POS_ROBOT, "Robot", 1, 1, 1, 0);
	graphique_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_STATIC_ELM, "Elements", 1, 1, 1, 0);
	graphique_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_HOKUYO_FOO, "Hokuyo foo", 1, 1, 0, 0);
	graphique_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_HOKUYO_FOO_SEG, "Hokuyo foo - poly", 1, 0, 1, 0);
	graphique_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_HOKUYO_BAR, "Hokuyo bar", 1, 0.5, 0.5, 0);
	graphique_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_POS_CONS, "Position (consigne)", 1, 0, 0, 1);
	graphique_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_POS_MES, "Position (mesure)", 1, 0, 1, 0);
	graphique_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_GRAPH, "Graph", 1, 0, 0, 0);
	graphique_add_courbe(&graph[GRAPH_TABLE], SUBGRAPH_TABLE_GRAPH_LINK, "Graph links", 1, 0, 1, 1);

	graphique_init(&graph[GRAPH_HOKUYO_HIST], "Hokuyo", 0, 682, 0, 4100, 800, 600, 0, 0);
	graphique_add_courbe(&graph[GRAPH_HOKUYO_HIST], GRAPH_HOKUYO_HIST_FOO, "Hokuyo foo", 1, 1, 0, 0);
	graphique_add_courbe(&graph[GRAPH_HOKUYO_HIST], GRAPH_HOKUYO_HIST_BAR, "Hokuyo bar", 1, 0, 0, 1);

	graphique_init(&graph[GRAPH_SPEED_DIST], "Control", 0, 90000, -1500, 1500, 800, 600, 0, 0);
	graphique_add_courbe(&graph[GRAPH_SPEED_DIST], SUBGRAPH_CONTROL_SPEED_DIST_MES, "Vitesse d'avance mesuree", 1, 0, 1, 0);
	graphique_add_courbe(&graph[GRAPH_SPEED_DIST], SUBGRAPH_CONTROL_SPEED_DIST_CONS, "Vitesse d'avance de consigne", 1, 0, 0, 1);
	graphique_add_courbe(&graph[GRAPH_SPEED_DIST], SUBGRAPH_CONTROL_SPEED_ROT_MES, "Vitesse de rotation mesuree", 1, 0.5, 0.5, 0);
	graphique_add_courbe(&graph[GRAPH_SPEED_DIST], SUBGRAPH_CONTROL_SPEED_ROT_CONS, "Vitesse de rotation de consigne", 1, 1, 0, 0);
	graphique_add_courbe(&graph[GRAPH_SPEED_DIST], SUBGRAPH_CONTROL_PWM_RIGHT, "PWM droite", 0, 1, 0, 1);
	graphique_add_courbe(&graph[GRAPH_SPEED_DIST], SUBGRAPH_CONTROL_PWM_LEFT, "PWM gauche ", 0, 0, 1, 1);
	graphique_add_courbe(&graph[GRAPH_SPEED_DIST], SUBGRAPH_CONTROL_I_RIGHT, "I droite ", 0, 1, 0.65, 0);
	graphique_add_courbe(&graph[GRAPH_SPEED_DIST], SUBGRAPH_CONTROL_I_LEFT, "I gauche ", 0, 1, 0, 0);

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
	main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
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

	joystick_init(&joystick, "/dev/input/js0", joystick_event);

	robot_interface_init(&robot_interface, file_foo_read, file_foo_write, file_bar_read, file_bar_write, read_callback, opengl_window);

	if( simulation )
	{
		cmd_init(&robot_interface, &qemu, gtk_end);

		// ajout de la table dans qemu
		for(i = 0; i < TABLE_OBJ_SIZE; i++)
		{
			qemu_add_object(&qemu, table_obj[i]);
		}

		// ajout d'un robot adverse
		qemu_add_object(&qemu, oponent_robot);

		// on le met a sa position de depart
		struct fx_vect2 origin = {0, 0};
		qemu_move_object(&qemu, 10, origin, opponent_robot_pos);
	}
	else
	{
		cmd_init(&robot_interface, NULL, gtk_end);
	}

	gtk_main();

	gdk_threads_leave();

	qemu_destroy(&qemu);

	robot_interface_destroy(&robot_interface);

	joystick_destroy(&joystick);

	return 0;
}

void gtk_end()
{
	gdk_threads_enter();
	gtk_main_quit();
	gdk_threads_leave();
}

void read_callback(GtkWidget* widget)
{
	static struct timespec last_plot = {0, 0};
	struct timespec current;

	clock_gettime(CLOCK_MONOTONIC, &current);
	double delta = (current.tv_sec - last_plot.tv_sec) + (current.tv_nsec - last_plot.tv_nsec) / ((double)1000000000.0f);
	if((delta >= 1.0f/MAX_FPS && (current_graph == GRAPH_TABLE || current_graph == GRAPH_HOKUYO_HIST) )
		|| delta >= 1.0f/5)
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
		graphique_set_border(&graph[i], 10 * font_width, font_height*3);
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

	int i;
	for( i = 0; i < GRAPH_NUM; i++)
	{
		graphique_resize_screen(&graph[i], screen_width, screen_height);
	}

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

void plot_axes(struct graphique* graph)
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

void plot_legende(struct graphique* graph)
{
	int i = 0;
	int dy = 0;
	for(i = 0; i < MAX_COURBES; i++)
	{
		if( graph->courbes_activated[i] )
		{
			glColor3fv(&graph->color[3*i]);
			glPrintf_xright2_yhigh(graph->roi_xmax, graph->roi_ymax + dy, graph->ratio_x, graph->ratio_y, font_base, "%s", graph->courbes_names[i]);
			dy -= 2*font_digit_height * graph->ratio_y;
		}
	}
}

void plot_table(struct graphique* graph)
{
	float ratio_x = graph->ratio_x;
	float ratio_y = graph->ratio_y;
	float plus_dx = 16384 * font_width * ratio_x;
	float plus_dy = 16384 * font_width * ratio_y;
	int i;
	int j;

	glPushMatrix();
	glColor3f(0,0,0);
	glScalef(1/65536.0f, 1/65536.0f, 1);

	if(graph->courbes_activated[SUBGRAPH_TABLE_STATIC_ELM])
	{
		// éléments statiques de la table partagés avec le code du robot (obstacles statiques)
		for(i = 0; i < TABLE_OBJ_SIZE; i++)
		{
			glBegin(GL_LINE_STRIP);
			for(j = 0; j < table_obj[i].size; j++)
			{
				glVertex2f(table_obj[i].pt[j].x, table_obj[i].pt[j].y);
			}
			glEnd();
		}

		// robot adverse
		glPushMatrix();
		glColor3f(1, 0, 0);
		glTranslatef(opponent_robot_pos.x, opponent_robot_pos.y, 0);
		glRotated(opponent_robot_pos.alpha* 360.0f / ((float)(1<<26)), 0, 0, 1);
		glBegin(GL_LINE_STRIP);
		for(i = 0; i < oponent_robot.size; i++)
		{
			glVertex2f(oponent_robot.pt[i].x, oponent_robot.pt[i].y);
		}
		glEnd();
		glPopMatrix();

		// couleurs sur les bords des cases de depart
		glColor3f(1, 0, 1);
		glBegin(GL_LINE_STRIP);
		glVertex2f(-1000 * 65536, 1000 * 65536);
		glVertex2f(-1500 * 65536, 1000 * 65536);
		glVertex2f(-1500 * 65536,  550 * 65536);
		glVertex2f(-1000 * 65536,  550 * 65536);
		glEnd();

		glColor3f(1, 0 ,0);
		glBegin(GL_LINE_STRIP);
		glVertex2f(1000 * 65536, 1000 * 65536);
		glVertex2f(1500 * 65536, 1000 * 65536);
		glVertex2f(1500 * 65536,  550 * 65536);
		glVertex2f(1000 * 65536,  550 * 65536);
		glEnd();

		// bouteilles
		glColor3f(1, 0, 1);
		glBegin(GL_TRIANGLE_STRIP);
		glVertex2f(-870 * 65536, -1000 * 65536);
		glVertex2f(-850 * 65536, -1000 * 65536);
		glVertex2f(-870 * 65536, -1060 * 65536);
		glVertex2f(-850 * 65536, -1060 * 65536);
		glEnd();
		glBegin(GL_TRIANGLE_STRIP);
		glVertex2f( 393 * 65536, -1000 * 65536);
		glVertex2f( 373 * 65536, -1000 * 65536);
		glVertex2f( 393 * 65536, -1060 * 65536);
		glVertex2f( 373 * 65536, -1060 * 65536);
		glEnd();

		glColor3f(1, 0, 0);
		glBegin(GL_TRIANGLE_STRIP);
		glVertex2f( 870 * 65536, -1000 * 65536);
		glVertex2f( 850 * 65536, -1000 * 65536);
		glVertex2f( 870 * 65536, -1060 * 65536);
		glVertex2f( 850 * 65536, -1060 * 65536);
		glEnd();
		glBegin(GL_TRIANGLE_STRIP);
		glVertex2f(-393 * 65536, -1000 * 65536);
		glVertex2f(-373 * 65536, -1000 * 65536);
		glVertex2f(-393 * 65536, -1060 * 65536);
		glVertex2f(-373 * 65536, -1060 * 65536);
		glEnd();

		// ligne de fin / cale
		glEnable(GL_LINE_STIPPLE);
		glLineStipple(1, 0xAAAA);
		glColor3f(0, 0, 0);

		glBegin(GL_LINES);
		glVertex2f(-1500 * 65536,  -390 * 65536);
		glVertex2f(-1160 * 65536,  -390 * 65536);
		glVertex2f( 1500 * 65536,  -390 * 65536);
		glVertex2f( 1160 * 65536,  -390 * 65536);
		glVertex2f(-1175 * 65536, -1000 * 65536);
		glVertex2f(-1100 * 65536,   500 * 65536);
		glVertex2f( 1175 * 65536, -1000 * 65536);
		glVertex2f( 1100 * 65536,   500 * 65536);
		glEnd();

		glDisable(GL_LINE_STIPPLE);
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO_FOO_SEG])
	{
		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_HOKUYO_FOO_SEG]);
		for(i = 0; i < robot_interface.detection_dynamic_object_size; i++)
		{
			if(robot_interface.detection_dynamic_obj[i].size > 1)
			{
				glBegin(GL_LINE_STRIP);
				for(j = 0; j < robot_interface.detection_dynamic_obj[i].size; j++)
				{
					struct fx_vect2 pt = robot_interface.detection_dynamic_obj[i].pt[j];
					glVertex2f(pt.x, pt.y);
				}
				glEnd();
			}
		}
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO_FOO] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_HOKUYO_FOO]);
		for(i=HOKUYO1*HOKUYO_NUM_POINTS; i < (HOKUYO1+1)*HOKUYO_NUM_POINTS; i++)
		{
			draw_plus(robot_interface.detection_hokuyo_pos[i].x, robot_interface.detection_hokuyo_pos[i].y, plus_dx, plus_dy);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO_BAR] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_HOKUYO_BAR]);
		for(i=HOKUYO2*HOKUYO_NUM_POINTS; i < (HOKUYO2+1)*HOKUYO_NUM_POINTS; i++)
		{
			draw_plus(robot_interface.detection_hokuyo_pos[i].x, robot_interface.detection_hokuyo_pos[i].y, plus_dx, plus_dy);
		}
	}

	int max = robot_interface.control_usb_data_count % CONTROL_USB_DATA_MAX;

	if( graph->courbes_activated[SUBGRAPH_TABLE_POS_CONS] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_POS_CONS]);
		for(i=0; i< max; i++)
		{
			if(robot_interface.control_usb_data[i].control_state != CONTROL_READY_ASSER && robot_interface.control_usb_data[i].control_state != CONTROL_READY_FREE)
			{
				draw_plus(robot_interface.control_usb_data[i].control_cons_x, robot_interface.control_usb_data[i].control_cons_y, plus_dx, plus_dy);
			}
		}
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_POS_MES] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_POS_MES]);
		for(i=0; i < max; i++)
		{
			draw_plus(robot_interface.control_usb_data[i].control_pos_x, robot_interface.control_usb_data[i].control_pos_y, plus_dx, plus_dy);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_GRAPH_LINK] )
	{
		glEnable(GL_LINE_STIPPLE);
		glLineStipple(1, 0xAAAA);
		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_GRAPH_LINK]);
		for(i=0; i < GRAPH_NUM_LINK; i++)
		{
			int a = graph_link[i].a;
			int b = graph_link[i].b;
			// on trace les liens une seule fois
			if( a < b)
			{
				float x1 = graph_node[a].pos.x;
				float y1 = graph_node[a].pos.y;
				float x2 = graph_node[b].pos.x;
				float y2 = graph_node[b].pos.y;
				glBegin(GL_LINES);
				glVertex2f(x1, y1);
				glVertex2f(x2, y2);
				glEnd();
				glPrintf_xcenter_ycenter(0.5f * (x1 + x2), 0.5f * (y1 + y2), ratio_x * 65536, ratio_y * 65536, font_base, "%d", graph_link[i].dist);
			}
		}
		glDisable(GL_LINE_STIPPLE);
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_GRAPH] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_GRAPH]);
		for(i=0; i < GRAPH_NUM_NODE; i++)
		{
			draw_plus(graph_node[i].pos.x, graph_node[i].pos.y, plus_dx, plus_dy);
			glPrintf_xcenter_yhigh2(graph_node[i].pos.x, graph_node[i].pos.y, ratio_x * 65536, ratio_y * 65536, font_base, "%d", i);
		}
	}

	// affichage du repère robot
	if( graph->courbes_activated[SUBGRAPH_TABLE_POS_ROBOT] && max > 0)
	{
		glColor3f(0, 0, 0);
		float x_robot = robot_interface.control_usb_data[max-1].control_pos_x;
		float y_robot = robot_interface.control_usb_data[max-1].control_pos_y;
		float alpha_robot = robot_interface.control_usb_data[max-1].control_pos_alpha * 360.0f / ((float)(1<<26)); // en degrés

		glPushMatrix();
		glTranslatef(x_robot, y_robot, 0);
		glRotatef(alpha_robot, 0, 0, 1);
		glBegin(GL_LINES);
		glVertex2f(0, 0);
		glVertex2f(50 * 65536, 0);
		glVertex2f(0, 0);
		glVertex2f(0, 50 * 65536);
		glEnd();

		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_POS_ROBOT]);
		glBegin(GL_LINE_STRIP);
		glVertex2f(PARAM_NP_X, PARAM_RIGHT_CORNER_Y);
		glVertex2f(PARAM_NP_X, PARAM_LEFT_CORNER_Y);
		glVertex2f(PARAM_LEFT_CORNER_X, PARAM_LEFT_CORNER_Y);
		glVertex2f(PARAM_RIGHT_CORNER_X, PARAM_RIGHT_CORNER_Y);
		glVertex2f(PARAM_NP_X, PARAM_RIGHT_CORNER_Y);
		glEnd();
		glPopMatrix();
	}
	glPopMatrix();
}

void plot_hokuyo_hist(struct graphique* graph)
{
	int i;

	float ratio_x = graph->ratio_x;
	float ratio_y = graph->ratio_y;

	if( graph->courbes_activated[GRAPH_HOKUYO_HIST_FOO] )
	{
		glColor3fv(&graph->color[3*GRAPH_HOKUYO_HIST_FOO]);
		for(i = 0; i < HOKUYO_NUM_POINTS; i++)
		{
			draw_plus(i, robot_interface.hokuyo_scan[HOKUYO1].distance[i], 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	if( graph->courbes_activated[GRAPH_HOKUYO_HIST_BAR] )
	{
		glColor3fv(&graph->color[3*GRAPH_HOKUYO_HIST_BAR]);
		for(i = 0; i < HOKUYO_NUM_POINTS; i++)
		{
			draw_plus(i, robot_interface.hokuyo_scan[HOKUYO2].distance[i], 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}
}

void plot_speed_dist(struct graphique* graph)
{
	int i;

	float ratio_x = graph->ratio_x;
	float ratio_y = graph->ratio_y;

	if( graph->courbes_activated[SUBGRAPH_CONTROL_SPEED_DIST_CONS] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_CONTROL_SPEED_DIST_CONS]);
		for(i=0; i < robot_interface.control_usb_data_count; i++)
		{
			draw_plus(5*i, robot_interface.control_usb_data[i].control_v_dist_cons*CONTROL_HZ/65536.0f, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_CONTROL_SPEED_ROT_CONS] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_CONTROL_SPEED_ROT_CONS]);
		for(i=1; i < robot_interface.control_usb_data_count; i++)
		{
			draw_plus(5*i, robot_interface.control_usb_data[i].control_v_rot_cons*1000.0f * CONTROL_HZ/67108864.0f, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_CONTROL_SPEED_DIST_MES] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_CONTROL_SPEED_DIST_MES]);
		for(i=1; i < robot_interface.control_usb_data_count; i++)
		{
			draw_plus(5*i, robot_interface.control_usb_data[i].control_v_dist_mes*CONTROL_HZ/65536.0f, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_CONTROL_SPEED_ROT_MES] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_CONTROL_SPEED_ROT_MES]);
		for(i=1; i < robot_interface.control_usb_data_count; i++)
		{
			draw_plus(5*i, robot_interface.control_usb_data[i].control_v_rot_mes*1000.0f * CONTROL_HZ/67108864.0f, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_CONTROL_PWM_RIGHT] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_CONTROL_PWM_RIGHT]);
		for(i=0; i < robot_interface.control_usb_data_count; i++)
		{
			draw_plus(5*i, (float)robot_interface.control_usb_data[i].control_u_right * 1000.0f / (PWM_ARR + 1), 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_CONTROL_PWM_LEFT] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_CONTROL_PWM_LEFT]);
		for(i=0; i < robot_interface.control_usb_data_count; i++)
		{
			draw_plus(5*i, (float)robot_interface.control_usb_data[i].control_u_left * 1000.0f / (PWM_ARR + 1), 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_CONTROL_I_RIGHT] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_CONTROL_I_RIGHT]);
		for(i=0; i < robot_interface.control_usb_data_count; i++)
		{
			draw_plus(5*i, (float)robot_interface.control_usb_data[i].control_i_right, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_CONTROL_I_LEFT] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_CONTROL_I_LEFT]);
		for(i=0; i < robot_interface.control_usb_data_count; i++)
		{
			draw_plus(5*i, (float)robot_interface.control_usb_data[i].control_i_left, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

#if 0
	// TODO : precalculer
	{
		glColor3f(0, 1, 1);
		float dist = 0;
		for(i=1; i < robot_interface.control_usb_data_count; i++)
		{
			if(robot_interface.control_usb_data[i].control_state == CONTROL_READY_FREE)
			{
				dist = 0;
			}
			float dx = (robot_interface.control_usb_data[i].control_cons_x - robot_interface.control_usb_data[i-1].control_cons_x)/65536.0f;
			float dy = (robot_interface.control_usb_data[i].control_cons_y - robot_interface.control_usb_data[i-1].control_cons_y)/65536.0f;
			dist += sqrtf(dx*dx+dy*dy);
			draw_plus(5*i, dist, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	{
		// TODO : precalculer
		glColor3f(1, 1, 0);
		float dist = 0;
		for(i=1; i < robot_interface.control_usb_data_count; i++)
		{
			if(robot_interface.control_usb_data[i].control_state == CONTROL_READY_FREE)
			{
				dist = 0;
			}
			float dx = (robot_interface.control_usb_data[i].control_pos_x - robot_interface.control_usb_data[i-1].control_pos_x)/65536.0f;
			float dy = (robot_interface.control_usb_data[i].control_pos_y - robot_interface.control_usb_data[i-1].control_pos_y)/65536.0f;
			dist += sqrtf(dx*dx+dy*dy);
			draw_plus(5*i, dist, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}
#endif
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

	int res = pthread_mutex_lock(&robot_interface.mutex);
	if(res == 0)
	{
		if(current_graph == GRAPH_SPEED_DIST)
		{
			graphique_resize_axis_x(&graph[current_graph], 0, robot_interface.control_usb_data_count * 5);
		}

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(graph[current_graph].plot_xmin, graph[current_graph].plot_xmax, graph[current_graph].plot_ymin, graph[current_graph].plot_ymax, 0, 1);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		glColor3f(0,0,0);
		plot_axes(&graph[current_graph]);
		plot_legende(&graph[current_graph]);

		glColor3f(0,0,0);
		int id = robot_interface.control_usb_data_count - 1;
		if( id < 0)
		{
			id = 0;
		}

		glPrintf_xcenter_ycenter(0, graph[current_graph].roi_ymax + 2*font_digit_height * graph->ratio_y, graph[current_graph].ratio_x, graph[current_graph].ratio_y, font_base, "%13.6f", robot_interface.current_time);
		if( robot_interface.start_time != 0)
		{
			glPrintf_xcenter_ycenter(0, graph[current_graph].roi_ymax,
										 graph[current_graph].ratio_x, graph[current_graph].ratio_y, font_base, "%13.6f", robot_interface.current_time - robot_interface.start_time);
		}

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
		pthread_mutex_unlock(&robot_interface.mutex);
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
	else if(event->button == 3)
	{
		struct graphique* gr = &graph[current_graph];
		float xrange = gr->roi_xmax - gr->roi_xmin;
		float yrange = gr->roi_ymax - gr->roi_ymin;

		float x1 = gr->roi_xmin + (event->x - gr->bordure_pixel_x) / (gr->screen_width - 2 * gr->bordure_pixel_x) * xrange;
		float y1 = gr->roi_ymin + (event->y - gr->bordure_pixel_y) / (gr->screen_height - 2 * gr->bordure_pixel_y) * yrange;

		double dx = x1 - opponent_robot_pos.x/65536.0f;
		double dy = -y1 - opponent_robot_pos.y/65536.0f;

		// on verifie qu'on clic a peu pres sur le robot
		if( sqrt(dx*dx+dy*dy) < 150 )
		{
			move_oponent_robot = 1;
			mouse_x1 = event->x;
			mouse_y1 = event->y;
			mouse_x2 = mouse_x1;
			mouse_y2 = mouse_y1;
		}
	}
	else
	{
		graphique_reset_roi(&graph[current_graph]);
	}
	gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
}

static void mounse_release(GtkWidget* widget, GdkEventButton* event)
{
	if(event->button == 1)
	{
		if( drawing_zoom_selection && mouse_x1 != mouse_x2 && mouse_y1 != mouse_y2)
		{
			graphique_zoom(&graph[current_graph], mouse_x1, mouse_x2, screen_height - mouse_y1, screen_height - mouse_y2);
		}

		drawing_zoom_selection = 0;
		mouse_x1 = 0;
		mouse_y1 = 0;
		mouse_x2 = 0;
		mouse_y2 = 0;
		gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
	}
	else if(event->button == 3)
	{
		if(move_oponent_robot == 1)
		{
			struct graphique* gr = &graph[current_graph];

			float xrange = gr->roi_xmax - gr->roi_xmin;
			float yrange = gr->roi_ymax - gr->roi_ymin;

			float x1 = (mouse_x1 - gr->bordure_pixel_x) / (gr->screen_width - 2 * gr->bordure_pixel_x) * xrange;
			float x2 = (mouse_x2 - gr->bordure_pixel_x) / (gr->screen_width - 2 * gr->bordure_pixel_x) * xrange;
			float y1 = (mouse_y1 - gr->bordure_pixel_y) / (gr->screen_height - 2 * gr->bordure_pixel_y) * yrange;
			float y2 = (mouse_y2 - gr->bordure_pixel_y) / (gr->screen_height - 2 * gr->bordure_pixel_y) * yrange;

			// on le met a sa position de depart
			struct fx_vect2 origin = {opponent_robot_pos.x, opponent_robot_pos.y};
			struct fx_vect_pos delta = {65536*(x2 - x1), 65536*(y1 - y2), 0, 1, 0};
			opponent_robot_pos.x += delta.x;
			opponent_robot_pos.y += delta.y;
			qemu_move_object(&qemu, 10, origin, delta);

			move_oponent_robot = 0;
			mouse_x1 = 0;
			mouse_y1 = 0;
			mouse_x2 = 0;
			mouse_y2 = 0;
			gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
		}
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
	else if(event->state & GDK_BUTTON3_MASK)
	{
		mouse_x2 = event->x;
		mouse_y2 = event->y;
	}
}

static gboolean keyboard_press(GtkWidget* widget, GdkEventKey* event, gpointer arg)
{
	(void) arg;
	int res;

	switch(event->keyval)
	{
		case GDK_Escape:
			drawing_zoom_selection = 0;
			break;
		case GDK_KP_Subtract:
		case GDK_minus:
			graphique_zoomf(&graph[current_graph], 2);
			break;
		case GDK_KP_Add:
		case GDK_plus:
			graphique_zoomf(&graph[current_graph], 0.5);
			break;
		case GDK_r:
			res = pthread_mutex_lock(&robot_interface.mutex);
			if(res == 0)
			{
				robot_interface.control_usb_data_count = 0;
				pthread_mutex_unlock(&robot_interface.mutex);
			}
			break;
		case GDK_u:
			graphique_reset_roi(&graph[current_graph]);
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

static void joystick_event(int event, float val)
{
	if(event & JOYSTICK_BTN_BASE)
	{
		int v = rint(val);
		// c'est un bouton
		switch(event & ~JOYSTICK_BTN_BASE)
		{
			case 0: // A (xbox)
				if( v )
				{
					robot_interface_pince(&robot_interface, PINCE_OPEN, PINCE_OPEN);
				}
				break;
			case 1: // B (xbox)
				if( v )
				{
					robot_interface_pince(&robot_interface, PINCE_CLOSE, PINCE_CLOSE);
				}
				break;
			case 2: // X (xbox)
				break;
			case 3: // Y (xbox)
				break;
			case 4: // LB (xbox)
				break;
			case 5: // RB (xbox)
				break;
			default:
				break;
		}
		//log_info("bouton %d : %d", event & ~JOYSTICK_BTN_BASE, (int)rint(val));
	}
	else
	{
		// c'est un axe
		switch(event)
		{
			case 0: // joystick gauche (xbox)
				robot_interface_rotate_speed(&robot_interface, val);
				break;
			case 2: // gachette gauche (xbox)
				robot_interface_straight_speed(&robot_interface, val);
				break;
			case 5: // gachette droite (xbox)
				robot_interface_straight_speed(&robot_interface, -val);
				break;
			default:
				break;
		}
		//log_info("axe %d : %f", event, val);
	}
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

static void glPrintf_xcenter_ycenter(float x, float y, float x_ratio,float y_ratio, GLuint base, char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x- x_ratio * size/2.0f * font_width, y - font_digit_height / 2.0f * y_ratio, base, buffer, size);
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

static void glPrintf_xright2_yhigh(float x, float y, float x_ratio,float y_ratio, GLuint base, char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x - x_ratio * (size+2) * font_width, y - font_digit_height * y_ratio, base, buffer, size);
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
