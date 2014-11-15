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
#include <string.h>

#include "linux/tools/robot_interface.h"
#include "linux/tools/qemu.h"
#include "linux/tools/cmd.h"
#include "linux/tools/graphique.h"
#include "linux/tools/joystick.h"
#include "linux/tools/table3d.h"
#include "kernel/robot_parameters.h"
#include "kernel/math/vect_plan.h"
#include "kernel/math/fx_math.h"
#include "kernel/math/matrix_homogeneous.h"
#include "kernel/motion/graph.h"
#include "kernel/table.h"

// limitation du rafraichissement
// hokuyo => 10fps. On met juste un peu plus
#define MAX_FPS    11
#define QEMU_OPPONENT_ID   (TABLE_OBJ_SIZE+1)

enum
{
	GL_NAME_NONE = 0,
	GL_NAME_ROBOT,
	GL_NAME_OPPONENT,
	GL_NAME_FEET_0,
	GL_NAME_FEET_1,
	GL_NAME_FEET_2,
	GL_NAME_FEET_3,
	GL_NAME_FEET_4,
	GL_NAME_FEET_5,
	GL_NAME_FEET_6,
	GL_NAME_FEET_7,
	GL_NAME_FEET_8,
	GL_NAME_FEET_9,
	GL_NAME_FEET_10,
	GL_NAME_FEET_11,
	GL_NAME_FEET_12,
	GL_NAME_FEET_13,
	GL_NAME_FEET_14,
	GL_NAME_FEET_15,
	GL_NAME_GLASS_0,
	GL_NAME_GLASS_1,
	GL_NAME_GLASS_2,
	GL_NAME_GLASS_3,
	GL_NAME_GLASS_4,
};

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
	SUBGRAPH_TABLE_TABLE,
	SUBGRAPH_TABLE_HOKUYO1,
	SUBGRAPH_TABLE_HOKUYO2,
	SUBGRAPH_TABLE_HOKUYO1_SEG,
	SUBGRAPH_TABLE_POS_CONS,
	SUBGRAPH_TABLE_POS_MES,
	SUBGRAPH_TABLE_GRAPH,
	SUBGRAPH_TABLE_GRAPH_LINK,
	SUBGRAPH_TABLE_NUM,
};

enum
{
	GRAPH_HOKUYO1_HIST = 0,
	GRAPH_HOKUYO2_HIST,
	GRAPH_HOKUYO_HIST_NUM,
};

enum
{
	SUBGRAPH_MOTION_SPEED_DIST_MES = 0,
	SUBGRAPH_MOTION_SPEED_DIST_CONS,
	SUBGRAPH_MOTION_SPEED_ROT_MES,
	SUBGRAPH_MOTION_SPEED_ROT_CONS,
	SUBGRAPH_MOTION_V1,
	SUBGRAPH_MOTION_V2,
	SUBGRAPH_MOTION_VBAT,
	SUBGRAPH_MOTION_I1,
	SUBGRAPH_MOTION_I2,
	SUBGRAPH_MOTION_I3,
	SUBGRAPH_MOTION_I4,
	SUBGRAPH_MOTION_NUM,
};

static GLuint font_base;
static char font_name[] = "fixed";
static int font_height = 0;
static int font_digit_height = 0;
static int font_width = 0;
static XFontStruct* font_info = NULL;
static int screen_width = 0;
static int screen_height = 0;
static RobotInterface* robotItf;
static Qemu* qemu;
static float mouse_x1 = 0;
static float mouse_y1 = 0;
static float mouse_x2 = 0;
static float mouse_y2 = 0;
static int drawing_zoom_selection = 0;
static int move_oponent_robot = 0;
static int current_graph = GRAPH_TABLE;
static GtkWidget* opengl_window = NULL;
static GtkWidget* main_window = NULL;
static int simulation = 0;
static char* atlantronicPath;
static bool glplot_show_legend = false;
static MatrixHomogeneous glplot_view;
static Table3d table3d;
static Object3d robot3d;
Graphique graph[GRAPH_NUM];
struct joystick joystick;
static int glplot_init_done = 0;
static float glplot_table_scale = 1;

static void close_gtk(GtkWidget* widget, gpointer arg);
static void select_graph(GtkWidget* widget, gpointer arg);
static void show_legend(GtkWidget* widget, gpointer arg);
static void select_active_courbe(GtkWidget* widget, gpointer arg);
static void init(GtkWidget* widget, gpointer arg);
static gboolean config(GtkWidget* widget, GdkEventConfigure* ev, gpointer arg);
static gboolean display(GtkWidget* widget, GdkEventExpose* ev, gpointer arg);
static void drawScene(GLenum mode);
static void mounse_press(GtkWidget* widget, GdkEventButton* event);
static void mounse_release(GtkWidget* widget, GdkEventButton* event);
static gboolean keyboard_press(GtkWidget* widget, GdkEventKey* event, gpointer arg);
static gboolean keyboard_release(GtkWidget* widget, GdkEventKey* event, gpointer arg);
static void simu_toggle_btn_color(GtkWidget* widget, gpointer arg);
static void simu_go(GtkWidget* widget, gpointer arg);
static void joystick_event(int event, float val);
static void mouse_move(GtkWidget* widget, GdkEventMotion* event);
static void mouse_scroll(GtkWidget* widget, GdkEventScroll* event);
static int init_font(GLuint base, char* f);
static void draw_plus(float x, float y, float rx, float ry);
static void glPrintf(float x, float y, GLuint base, const char* s, ...);
static void glPrintf_xright2_ycenter(float x, float y, float x_ratio, float y_ratio, GLuint base, const char* s, ...) __attribute__(( format(printf, 6, 7) ));
static void glPrintf_xright2_yhigh(float x, float y, float x_ratio, float y_ratio, GLuint base, const char* s, ...) __attribute__(( format(printf, 6, 7) ));
static void glPrintf_xcenter_yhigh2(float x, float y, float x_ratio, float y_ratio, GLuint base, const char* s, ...) __attribute__(( format(printf, 6, 7) ));
static void glPrintf_xcenter_ycenter(float x, float y, float x_ratio, float y_ratio, GLuint base, const char* s, ...) __attribute__(( format(printf, 6, 7) ));
static void glprint(float x, float y, GLuint base, const char* buffer, int size);
void gtk_end();

#define OPPONENT_PERIMETER         128.0f

Vect2 opponent_robot_pt[] =
{
	Vect2( OPPONENT_PERIMETER, 0),
	Vect2( OPPONENT_PERIMETER * 0.707106781, OPPONENT_PERIMETER * 0.707106781),
	Vect2( 0, OPPONENT_PERIMETER),
	Vect2( -OPPONENT_PERIMETER * 0.707106781, OPPONENT_PERIMETER * 0.707106781),
	Vect2( -OPPONENT_PERIMETER, 0),
	Vect2( -OPPONENT_PERIMETER * 0.707106781, -OPPONENT_PERIMETER * 0.707106781),
	Vect2( 0, -OPPONENT_PERIMETER),
	Vect2( OPPONENT_PERIMETER * 0.707106781, -OPPONENT_PERIMETER * 0.707106781),
	Vect2( OPPONENT_PERIMETER, 0),
};

struct polyline oponent_robot =
{
		opponent_robot_pt,
		sizeof(opponent_robot_pt) / sizeof(opponent_robot_pt[0])
};

struct VectPlan opponent_robot_pos(1300, 0, M_PI);

int glplot_main(const char* AtlantronicPath, int Simulation, bool cli, Qemu* Qemu, RobotInterface* RobotItf)
{
	long i = 0;
	long j = 0;
	atlantronicPath = strdup(AtlantronicPath);
	simulation = Simulation;
	if( ! simulation )
	{
		Qemu = NULL;
	}
	qemu = Qemu;
	robotItf = RobotItf;

	graph[GRAPH_TABLE].init("Table", -1600, 2500, -1100, 1100, 800, 600, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_POS_ROBOT, "Robot", 1, 0, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_STATIC_ELM, "Elements", 1, 0, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_TABLE, "Table", 1, 0, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_HOKUYO1, "Hokuyo 1", 1, 1, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_HOKUYO1_SEG, "Hokuyo 1 - poly", 1, 0, 1, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_HOKUYO2, "Hokuyo 2", 1, 0.5, 0.5, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_POS_CONS, "Position (consigne)", 1, 0, 0, 1);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_POS_MES, "Position (mesure)", 1, 0, 1, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_GRAPH, "Graph", 1, 1, 1, 1);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_GRAPH_LINK, "Graph links", 1, 0, 1, 1);

	graph[GRAPH_HOKUYO_HIST].init("Hokuyo", 0, 682, 0, 4100, 800, 600, 0, 0);
	graph[GRAPH_HOKUYO_HIST].add_courbe(GRAPH_HOKUYO1_HIST, "Hokuyo 1", 1, 1, 0, 0);
	graph[GRAPH_HOKUYO_HIST].add_courbe(GRAPH_HOKUYO2_HIST, "Hokuyo 2", 1, 0, 0, 1);

	graph[GRAPH_SPEED_DIST].init("Control", 0, 90000, -1500, 1500, 800, 600, 0, 0);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_SPEED_DIST_MES, "Vitesse d'avance mesuree", 1, 0, 1, 0);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_SPEED_DIST_CONS, "Vitesse d'avance de consigne", 1, 0, 0, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_SPEED_ROT_MES, "Vitesse de rotation mesuree", 1, 0.5, 0.5, 0);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_SPEED_ROT_CONS, "Vitesse de rotation de consigne", 1, 1, 0, 0);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_V1, "v1", 0, 1, 0, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_V2, "v2", 0, 0.5, 0, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_VBAT, "vBat", 0, 1, 0, 0);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_I1, "i1", 0, 0, 0, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_I2, "i2", 0, 0, 0, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_I3, "i3", 0, 0, 0, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_I4, "i4", 0, 0, 0, 1);

#ifdef OLD_GTK
	g_thread_init(NULL);
#endif

	gdk_threads_init();
	gdk_threads_enter();

	gtk_init(0, NULL);
	gtk_gl_init(0, NULL);

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
	g_signal_connect(G_OBJECT(opengl_window), "expose_event", G_CALLBACK(display), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "button_press_event", G_CALLBACK(mounse_press), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "button_release_event", G_CALLBACK(mounse_release), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "motion_notify_event", G_CALLBACK(mouse_move), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "scroll-event", G_CALLBACK(mouse_scroll), NULL);
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

	menu1 = gtk_menu_new();	// menu "niveau 1"
	menuObj = gtk_menu_item_new_with_label("Options");
	gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu0), menuObj);

	menuObj = gtk_check_menu_item_new_with_label("afficher legende");
	gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM (menuObj), false);
	g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(show_legend), NULL);
	gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM (menuObj), FALSE);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

	if(simulation)
	{
		menu1 = gtk_menu_new();	// menu "niveau 1"
		menuObj = gtk_menu_item_new_with_label("Simulation");
		gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);
		gtk_menu_shell_append(GTK_MENU_SHELL(menu0), menuObj);

		menuObj = gtk_menu_item_new_with_label("toggle BTN color");
		g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(simu_toggle_btn_color), NULL);
		gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

		menuObj = gtk_check_menu_item_new_with_label("go");
		gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM (menuObj), false);
		g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(simu_go), NULL);
		gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);
	}

	// rangement des éléments dans la fenetre
	// vbox la fenetre principale : menu + fenetre opengl
	GtkWidget* main_vbox = gtk_vbox_new(FALSE, 0);
	gtk_container_add(GTK_CONTAINER(main_window), main_vbox);

	gtk_box_pack_start(GTK_BOX(main_vbox), menu0, FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(main_vbox), opengl_window, TRUE, TRUE, 0);

	gtk_widget_show_all(main_window);

	joystick_init(&joystick, "/dev/input/js0", joystick_event);

	if( cli )
	{
		cmd_init(robotItf, qemu, gtk_end);
	}

	if( simulation )
	{
		// ajout de la table dans qemu
		for(i = 0; i < TABLE_OBJ_SIZE; i++)
		{
			qemu->add_object(table_obj[i]);
		}

		// ajout d'un robot adverse
		qemu->add_object(oponent_robot);

		// on le met a sa position de depart
		Vect2 origin(0, 0);
		qemu->move_object(QEMU_OPPONENT_ID, origin, opponent_robot_pos);
	}

	gtk_main();

	gdk_threads_leave();

	joystick_destroy(&joystick);

	return 0;
}

void gtk_end()
{
	gdk_threads_enter();
	gtk_main_quit();
	gdk_threads_leave();
}

void glplot_update()
{
	static struct timespec last_plot = {0, 0};
	struct timespec current;

	if( opengl_window && glplot_init_done )
	{
		clock_gettime(CLOCK_MONOTONIC, &current);
		double delta = (current.tv_sec - last_plot.tv_sec) + (current.tv_nsec - last_plot.tv_nsec) / ((double)1000000000.0f);
		if((delta >= 1.0f/MAX_FPS && (current_graph == GRAPH_TABLE || current_graph == GRAPH_HOKUYO_HIST) )
			|| delta >= 1.0f/5)
		{
			gdk_threads_enter();
			if(opengl_window->window)
			{
				gdk_window_invalidate_rect(opengl_window->window, &opengl_window->allocation, FALSE);
			}
			gdk_threads_leave();
			last_plot = current;
		}
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

static void show_legend(GtkWidget* widget, gpointer arg)
{
	(void) widget;
	(void) arg;

	glplot_show_legend = ! glplot_show_legend;
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
	// la verif ne passe pas sur une vm de virtualbox
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
		graph[i].set_border(10 * font_width, font_height*3);
	}

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH);

	int glSelectFeetName[16] =
	{
		GL_NAME_FEET_0,
		GL_NAME_FEET_1,
		GL_NAME_FEET_2,
		GL_NAME_FEET_3,
		GL_NAME_FEET_4,
		GL_NAME_FEET_5,
		GL_NAME_FEET_6,
		GL_NAME_FEET_7,
		GL_NAME_FEET_8,
		GL_NAME_FEET_9,
		GL_NAME_FEET_10,
		GL_NAME_FEET_11,
		GL_NAME_FEET_12,
		GL_NAME_FEET_13,
		GL_NAME_FEET_14,
		GL_NAME_FEET_15,
	};
	int glSelectGlassName[5] =
	{
		GL_NAME_GLASS_0,
		GL_NAME_GLASS_1,
		GL_NAME_GLASS_2,
		GL_NAME_GLASS_3,
		GL_NAME_GLASS_4,
	};
	if( ! table3d.init(glSelectFeetName, glSelectGlassName) )
	{
		fprintf(stderr, "table3d.init() error - Exiting.\n");
		exit(-1);
	}

	if( ! robot3d.init("media/robot2011.3ds") )
	{
		fprintf(stderr, "table3d.init() error - Exiting.\n");
		exit(-1);
	}

	gdk_gl_drawable_gl_end(gldrawable);
	glplot_init_done = 1;
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
		graph[i].resize_screen(screen_width, screen_height);
	}

	glViewport(0, 0, screen_width, screen_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, screen_width, screen_height, 0, 0, 1);

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

void plot_axes(Graphique* graph)
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

void plot_legende(Graphique* graph)
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

void plot_pave(float x, float y, float z, float dx, float dy, float dz)
{
	glTranslatef(x, y, z);
	dx /= 2;
	dy /= 2;
	dz /= 2;

	glBegin(GL_QUADS);
	glNormal3f(-1, 0, 0);
	glVertex3f(-dx, -dy, -dz);
	glVertex3f(-dx,  dy, -dz);
	glVertex3f(-dx,  dy,  dz);
	glVertex3f(-dx, -dy,  dz);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(1, 0, 0);
	glVertex3f( dx, -dy, -dz);
	glVertex3f( dx,  dy, -dz);
	glVertex3f( dx,  dy,  dz);
	glVertex3f( dx, -dy,  dz);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(0, -1, 0);
	glVertex3f(-dx, -dy, -dz);
	glVertex3f(-dx, -dy,  dz);
	glVertex3f( dx, -dy,  dz);
	glVertex3f( dx, -dy, -dz);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(0, 1, 0);
	glVertex3f(-dx,  dy, -dz);
	glVertex3f(-dx,  dy,  dz);
	glVertex3f( dx,  dy,  dz);
	glVertex3f( dx,  dy, -dz);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(0, 0, -1);
	glVertex3f(-dx, -dy, -dz);
	glVertex3f( dx, -dy, -dz);
	glVertex3f( dx,  dy, -dz);
	glVertex3f(-dx,  dy, -dz);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(0, 0, 1);
	glVertex3f(-dx, -dy, dz);
	glVertex3f( dx, -dy, dz);
	glVertex3f( dx,  dy, dz);
	glVertex3f(-dx,  dy, dz);
	glEnd();

	glTranslatef(-x, -y, -z);
}

void plot_table(Graphique* graph)
{
	float ratio_x = graph->ratio_x;
	float ratio_y = graph->ratio_y;
	float plus_dx = 0.25 * font_width * ratio_x;
	float plus_dy = 0.25 * font_width * ratio_y;
	int i;
	int j;

	glPushMatrix();
	glColor3f(0,0,0);

	if(graph->courbes_activated[SUBGRAPH_TABLE_TABLE])
	{
		glDisable(GL_COLOR_MATERIAL);
		table3d.draw();
		glEnable(GL_COLOR_MATERIAL);
		glDisable(GL_CULL_FACE);
	}

	if(graph->courbes_activated[SUBGRAPH_TABLE_STATIC_ELM])
	{
		glColor3f(0, 0, 0);
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
	}

	/*if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO1_SEG])
	{
		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_HOKUYO1_SEG]);
		for(i = 0; i < robotItf->detection_dynamic_object_size; i++)
		{
			if(robotItf->detection_dynamic_obj[i].size > 1)
			{
				glBegin(GL_LINE_STRIP);
				for(j = 0; j < robotItf->detection_dynamic_obj[i].size; j++)
				{
					Vect2 pt = robotItf->detection_dynamic_obj[i].pt[j];
					glVertex2f(pt.x, pt.y);
				}
				glEnd();
			}
		}
	}*/

	for(i = 0; i < (int)robotItf->detection_dynamic_object_count1; i++)
	{
		glColor3f(0,0,1);
		draw_plus(robotItf->detection_obj1[i].x, robotItf->detection_obj1[i].y, 50, 50);
		//printf("obj_h1 : %7.2f %7.2f\n", robotItf->detection_obj1[i].x, robotItf->detection_obj1[i].y);
	}

	for(i = 0; i < (int)robotItf->detection_dynamic_object_count2; i++)
	{
		glColor3f(0,1,1);
		draw_plus(robotItf->detection_obj2[i].x, robotItf->detection_obj2[i].y, 50, 50);
		//printf("obj_h2 : %7.2f %7.2f\n", robotItf->detection_obj2[i].x, robotItf->detection_obj2[i].y);
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO1] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_HOKUYO1]);
		for(i=HOKUYO1*HOKUYO_NUM_POINTS; i < (HOKUYO1+1)*HOKUYO_NUM_POINTS; i++)
		{
			draw_plus(robotItf->detection_hokuyo_pos[i].x, robotItf->detection_hokuyo_pos[i].y, plus_dx, plus_dy);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_HOKUYO2] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_HOKUYO2]);
		for(i=HOKUYO2*HOKUYO_NUM_POINTS; i < (HOKUYO2+1)*HOKUYO_NUM_POINTS; i++)
		{
			draw_plus(robotItf->detection_hokuyo_pos[i].x, robotItf->detection_hokuyo_pos[i].y, plus_dx, plus_dy);
		}
	}

	int max = robotItf->control_usb_data_count % CONTROL_USB_DATA_MAX;

	if( graph->courbes_activated[SUBGRAPH_TABLE_POS_CONS] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_POS_CONS]);
		for(i=0; i< max; i++)
		{
			if(robotItf->control_usb_data[i].motion_state != MOTION_ENABLED && robotItf->control_usb_data[i].motion_state != MOTION_DISABLED)
			{
				draw_plus(robotItf->control_usb_data[i].cons.x, robotItf->control_usb_data[i].cons.y, plus_dx, plus_dy);
			}
		}
	}

	if( graph->courbes_activated[SUBGRAPH_TABLE_POS_MES] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_TABLE_POS_MES]);
		for(i=0; i < max; i++)
		{
			draw_plus(robotItf->control_usb_data[i].pos.x, robotItf->control_usb_data[i].pos.y, plus_dx, plus_dy);
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
				glPrintf_xcenter_ycenter(0.5f * (x1 + x2), 0.5f * (y1 + y2), ratio_x, ratio_y, font_base, "%d", graph_link[i].dist);
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
			glPrintf_xcenter_yhigh2(graph_node[i].pos.x, graph_node[i].pos.y, ratio_x, ratio_y, font_base, "%d", i);
		}
	}

	glPushName(GL_NAME_OPPONENT);
	// robot adverse
	glPushMatrix();
	glColor3f(1, 0, 0);
	glTranslatef(opponent_robot_pos.x, opponent_robot_pos.y, 0);
	glRotated(opponent_robot_pos.theta * 360.0f / (2*M_PI), 0, 0, 1);
#if 0
	glBegin(GL_LINE_STRIP);
	for(i = 0; i < oponent_robot.size; i++)
	{
		glVertex2f(oponent_robot.pt[i].x, oponent_robot.pt[i].y);
	}
	glEnd();
#endif
	glPushMatrix();
	glRotatef(90, 1, 0, 0);
	glDisable(GL_COLOR_MATERIAL);
	robot3d.draw();
	glEnable(GL_COLOR_MATERIAL);
	glDisable(GL_CULL_FACE);
	glPopMatrix();

	glPopMatrix();

	glPopName();

	glPushName(GL_NAME_ROBOT);
	// affichage du robot
	if( graph->courbes_activated[SUBGRAPH_TABLE_POS_ROBOT] && max > 0)
	{
		// robot
		VectPlan pos_robot = robotItf->control_usb_data[max-1].pos;
		glPushMatrix();
		glTranslatef(pos_robot.x, pos_robot.y, 0);
		glRotatef(pos_robot.theta * 180 / M_PI, 0, 0, 1);

		glColor3f(0, 0, 0);
		glBegin(GL_LINES);
		glVertex2f(0, 0);
		glVertex2f(50, 0);
		glVertex2f(0, 0);
		glVertex2f(0, 50);
		glEnd();

		glBegin(GL_LINE_STRIP);
		glVertex2f(PARAM_NP_X, PARAM_RIGHT_CORNER_Y);
		glVertex2f(PARAM_NP_X, PARAM_LEFT_CORNER_Y);
		glVertex2f(PARAM_LEFT_CORNER_X, PARAM_LEFT_CORNER_Y);
		glVertex2f(PARAM_RIGHT_CORNER_X, PARAM_RIGHT_CORNER_Y);
		glVertex2f(PARAM_NP_X, PARAM_RIGHT_CORNER_Y);
		glEnd();

		glPushMatrix();
		glRotatef(90, 1, 0, 0);
		glDisable(GL_COLOR_MATERIAL);
		robot3d.draw();
		glEnable(GL_COLOR_MATERIAL);
		glDisable(GL_CULL_FACE);
		glPopMatrix();

		glPopMatrix();
	}
	glPopName();
	glPopMatrix();
}

void plot_hokuyo_hist(Graphique* graph)
{
	int i;

	float ratio_x = graph->ratio_x;
	float ratio_y = graph->ratio_y;

	if( graph->courbes_activated[GRAPH_HOKUYO1_HIST] )
	{
		glColor3fv(&graph->color[3*GRAPH_HOKUYO1_HIST]);
		for(i = 0; i < HOKUYO_NUM_POINTS; i++)
		{
			draw_plus(i, robotItf->hokuyo_scan[HOKUYO1].distance[i], 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	if( graph->courbes_activated[GRAPH_HOKUYO2_HIST] )
	{
		glColor3fv(&graph->color[3*GRAPH_HOKUYO2_HIST]);
		for(i = 0; i < HOKUYO_NUM_POINTS; i++)
		{
			draw_plus(i, robotItf->hokuyo_scan[HOKUYO2].distance[i], 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}
}

void plot_speed_dist(Graphique* graph)
{
	int i;

	float ratio_x = graph->ratio_x;
	float ratio_y = graph->ratio_y;

	if( graph->courbes_activated[SUBGRAPH_MOTION_SPEED_DIST_CONS] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_MOTION_SPEED_DIST_CONS]);
		VectPlan old_cons = robotItf->control_usb_data[0].cons;
		for(i=1; i < robotItf->control_usb_data_count; i++)
		{
			VectPlan cons = robotItf->control_usb_data[i].cons;
			VectPlan v = (cons - old_cons) / CONTROL_DT;
			draw_plus(5*i, v.norm(), 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
			old_cons = cons;
		}
	}

	if( graph->courbes_activated[SUBGRAPH_MOTION_SPEED_ROT_CONS] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_MOTION_SPEED_ROT_CONS]);
		VectPlan old_cons = robotItf->control_usb_data[0].cons;
		for(i=1; i < robotItf->control_usb_data_count; i++)
		{
			VectPlan cons = robotItf->control_usb_data[i].cons;
			VectPlan v = (cons - old_cons) / CONTROL_DT;
			draw_plus(5*i, v.theta*1000.0f, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
			old_cons = cons;
		}
	}

	if( graph->courbes_activated[SUBGRAPH_MOTION_SPEED_DIST_MES] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_MOTION_SPEED_DIST_MES]);
		VectPlan old_pos = robotItf->control_usb_data[0].pos;
		for(i=1; i < robotItf->control_usb_data_count; i++)
		{
			VectPlan pos = robotItf->control_usb_data[i].pos;
			VectPlan v = (pos - old_pos) / CONTROL_DT;
			draw_plus(5*i, v.norm(), 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
			old_pos = pos;
		}
	}

	if( graph->courbes_activated[SUBGRAPH_MOTION_SPEED_ROT_MES] )
	{
		glColor3fv(&graph->color[3*SUBGRAPH_MOTION_SPEED_ROT_MES]);
		VectPlan old_pos = robotItf->control_usb_data[0].pos;
		for(i=1; i < robotItf->control_usb_data_count; i++)
		{
			VectPlan pos = robotItf->control_usb_data[i].pos;
			VectPlan v = (pos - old_pos) / CONTROL_DT;
			draw_plus(5*i, v.theta*1000.0f, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
			old_pos = pos;
		}
	}

	for(int j = 0; j < 2; j++)
	{
		if( graph->courbes_activated[SUBGRAPH_MOTION_V1 + j] )
		{
			glColor3fv(&graph->color[3*SUBGRAPH_MOTION_V1 + j]);
			for(i=0; i < robotItf->control_usb_data_count; i++)
			{
				draw_plus(5*i, robotItf->control_usb_data[i].cons_motors_v[j], 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
			}
		}
	}

	if( graph->courbes_activated[SUBGRAPH_MOTION_VBAT] )
	{
		glColor3fv(&graph->color[3*(SUBGRAPH_MOTION_VBAT)]);
		for(int j=1; j < robotItf->control_usb_data_count; j++)
		{
			float v = robotItf->control_usb_data[j].vBat;
			draw_plus(5*j, v, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	for(int i = 0; i < 4; i++)
	{
		if( graph->courbes_activated[SUBGRAPH_MOTION_I1+i] )
		{
			glColor3fv(&graph->color[3*(SUBGRAPH_MOTION_I1+i)]);
			for(int j=1; j < robotItf->control_usb_data_count; j++)
			{
				float current = robotItf->control_usb_data[j].iPwm[i];
				draw_plus(5*j, 1000*current, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
			}
		}
	}
}

void processHits(GLint hits, GLuint buffer[])
{
	GLuint names, *ptr;

	ptr = (GLuint *) buffer;
	for(int i = 0; i < hits; i++)
	{
		names = *ptr;
		ptr+= 3;
		if( names )
		{
			ptr += names - 1;
			int name_id = *ptr;
			if( name_id >= GL_NAME_FEET_0 && name_id <= GL_NAME_FEET_15)
			{
				table3d.unselectAll();
				table3d.selectFeet(name_id - GL_NAME_FEET_0);
			}
			else if( name_id >= GL_NAME_GLASS_0 && name_id <= GL_NAME_GLASS_4)
			{
				table3d.unselectAll();
				table3d.selectGlass(name_id - GL_NAME_GLASS_0);
			}
#if 0
			if( name_id != GL_NAME_NONE )
			{
				printf("name %d\n", name_id);
			}
#endif
			ptr++;
		}
	}
}

static gboolean display(GtkWidget* widget, GdkEventExpose* ev, gpointer arg)
{
	static GLuint openglSelectBuffer[1024];

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

	drawScene(GL_RENDER);

	glSelectBuffer(sizeof(openglSelectBuffer)/sizeof(openglSelectBuffer[0]), openglSelectBuffer);
	glRenderMode(GL_SELECT);
	glInitNames();
	glPushName(GL_NAME_NONE);

	drawScene(GL_SELECT);
	GLint hits = glRenderMode(GL_RENDER);
	processHits(hits, openglSelectBuffer);

	if(gdk_gl_drawable_is_double_buffered(gldrawable))
	{
		gdk_gl_drawable_swap_buffers(gldrawable);
	}
	else
	{
		glFlush();
	}

	glDisable(GL_COLOR_LOGIC_OP);
	glDepthFunc(GL_LESS);
	gdk_gl_drawable_gl_end(gldrawable);

	return TRUE;
}

static void drawScene(GLenum mode)
{
	int res = pthread_mutex_lock(&robotItf->mutex);
	if(res == 0)
	{
		if(current_graph == GRAPH_SPEED_DIST)
		{
			graph[current_graph].resize_axis_x(0, robotItf->control_usb_data_count * 5);
		}

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		if( mode == GL_SELECT)
		{
			GLint viewport[4];;
			glGetIntegerv(GL_VIEWPORT, viewport);
			gluPickMatrix((GLdouble) mouse_x1, (GLdouble) (viewport[3] - mouse_y1), 5.0, 5.0, viewport);
		}

		if( current_graph == GRAPH_TABLE)
		{
			gluPerspective(70, (float)graph[current_graph].screen_width/(float)graph[current_graph].screen_height, 1, 10000);
		}
		else
		{
			glOrtho(graph[current_graph].plot_xmin, graph[current_graph].plot_xmax, graph[current_graph].plot_ymin, graph[current_graph].plot_ymax, 0, 1);
		}

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		if( current_graph == GRAPH_TABLE)
		{
			gluLookAt(0, -1000*glplot_table_scale, 2000*glplot_table_scale, 0, 0, 0, 0, 0, 1);
			float mat[16];
			mat[0] = glplot_view.val[0];
			mat[1] = glplot_view.val[4];
			mat[2] = glplot_view.val[8];
			mat[3] = 0;
			mat[4] = glplot_view.val[1];
			mat[5] = glplot_view.val[5];
			mat[6] = glplot_view.val[9];
			mat[7] = 0;
			mat[8] = glplot_view.val[2];
			mat[9] = glplot_view.val[6];
			mat[10] = glplot_view.val[10];
			mat[11] = 0;
			mat[12] = glplot_view.val[3];
			mat[13] = glplot_view.val[7];
			mat[14] = glplot_view.val[11];
			mat[15] = 1;
			glMultMatrixf(mat);

			glEnable(GL_LIGHTING);
			glEnable(GL_DEPTH_TEST);

			//GLfloat global_ambient[] = { 0.5, 0.5, 0.5, 1 };
			GLfloat ambientLight[] = { 0.3, 0.3, 0.3, 1 };
			GLfloat diffuseLight[] = { 0.4, 0.4, 0.4, 1 };
			GLfloat specularLight[] = { 0.4, 0.4, 0.4, 1 };
			GLfloat light_position0[] = { 1500, 1000, 1000, 1 };
			GLfloat light_direction0[] = { -1500, -1000, -1000, 0};
			GLfloat light_position1[] = { -1500, 1000, 1000, 1 };
			GLfloat light_direction1[] = { 1500, -1000, -1000, 0};

			//glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
			glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
			glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
			glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
			glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
			glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light_direction0);
			//glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 45.0);
			//glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 2.0);
			//glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.00001f);
			//glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 1);

			glLightfv(GL_LIGHT1, GL_AMBIENT, ambientLight);
			glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight);
			glLightfv(GL_LIGHT1, GL_SPECULAR, specularLight);
			glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
			glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, light_direction1);
			//glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 45.0);
			//glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 2.0);
			//glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.00001f);
			//glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 1);
		}
		else
		{
			glDisable(GL_LIGHTING);
			glDisable(GL_DEPTH_TEST);
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

		glColor3f(0,0,0);
		plot_axes(&graph[current_graph]);
		if( glplot_show_legend )
		{
			plot_legende(&graph[current_graph]);
		}

		glColor3f(0,0,0);

		if( current_graph == GRAPH_TABLE )
		{
			int lineHeight = -2*font_digit_height * graph->ratio_y;
			int lineId = 0;
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "time  %13.6f", robotItf->current_time);
			lineId++;
			struct control_usb_data* data = &robotItf->last_control_usb_data;
			double match_time = 0;
			if( robotItf->start_time )
			{
				match_time = robotItf->current_time - robotItf->start_time;
			}
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "match %13.6f", match_time);
			lineId++;
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "power off %#9x", data->power_state);
			lineId++;
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "pos  %6.0f %6.0f %6.2f",
					data->pos.x, data->pos.y,
					data->pos.theta * 180 / M_PI);
			lineId++;
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "wpos %6.0f %6.0f %6.2f",
					data->wanted_pos.x, data->wanted_pos.y,
					data->wanted_pos.theta * 180 / M_PI);
			lineId++;
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "v %5.2f %5.2f (wanted %5.2f %5.2f)",
					data->mes_motors[0].v, data->mes_motors[1].v, data->cons_motors_v[0], data->cons_motors_v[1]);
			lineId++;
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "gyro %6.2f",
							data->pos_theta_gyro_euler * 180 / M_PI);
			lineId++;
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "vBat  %6.3f", data->vBat);
			lineId++;
			for(int i = 0; i < 4; i++)
			{
				glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "iPwm%i  %6.3f", i, data->iPwm[i]);
				lineId++;
			}
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "cod  %6d %6d %6d", data->encoder[0], data->encoder[1], data->encoder[2]);
			lineId++;
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "io %d%d %d%d %d%d %d%d %d%d %d%d ingo %d go %d",
					data->gpio & 0x01,
					(data->gpio >> 1) & 0x01,
					(data->gpio >> 2) & 0x01,
					(data->gpio >> 3) & 0x01,
					(data->gpio >> 4) & 0x01,
					(data->gpio >> 5) & 0x01,
					(data->gpio >> 6) & 0x01,
					(data->gpio >> 7) & 0x01,
					(data->gpio >> 8) & 0x01,
					(data->gpio >> 9) & 0x01,
					(data->gpio >> 10) & 0x01,
					(data->gpio >> 11) & 0x01,
					(data->gpio >> 12) & 0x01,
					(data->gpio >> 13) & 0x01);
			lineId++;
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "pump blocked %d %d %d %d",
					(data->pumpState & 0x01),
					((data->pumpState >> 1) & 0x01),
					((data->pumpState >> 2) & 0x01),
					((data->pumpState >> 3) & 0x01));
			lineId++;
			for(int i = 2; i < AX12_MAX_ID; i++)
			{
				glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "ax12 %2d pos %7.2f target %d stuck %d error %2x", i,
						robotItf->ax12[i].pos * 180 / M_PI,
						(robotItf->ax12[i].flags & DYNAMIXEL_FLAG_TARGET_REACHED)?1:0,
						(robotItf->ax12[i].flags & DYNAMIXEL_FLAG_STUCK) ? 1:0,
						(robotItf->ax12[i].error.transmit_error << 8) + robotItf->ax12[i].error.internal_error);
				lineId++;
			}
			for(int i = 2; i < RX24_MAX_ID; i++)
			{
				glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "rx24 %2d pos %7.2f target %d stuck %d error %2x", i,
						robotItf->rx24[i].pos * 180 / M_PI,
						(robotItf->rx24[i].flags & DYNAMIXEL_FLAG_TARGET_REACHED)?1:0,
						(robotItf->rx24[i].flags & DYNAMIXEL_FLAG_STUCK) ? 1:0,
						(robotItf->rx24[i].error.transmit_error << 8) + robotItf->rx24[i].error.internal_error);
				lineId++;
			}
			float* mat = data->arm_matrix.val;
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "arm_mat %5.2f %5.2f %5.2f %5.2f", mat[0], mat[1], mat[2], mat[3]);
			lineId++;
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "arm_mat %5.2f %5.2f %5.2f %5.2f", mat[4], mat[5], mat[6], mat[7]);
			lineId++;
			glPrintf(1600, graph->roi_ymax + lineId*lineHeight, font_base, "arm_mat %5.2f %5.2f %5.2f %5.2f", mat[8], mat[9], mat[10], mat[11]);
			lineId++;
		}

		pthread_mutex_unlock(&robotItf->mutex);
	}

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
	if(event->button == 1 )
	{
		if( current_graph != GRAPH_TABLE )
		{
			drawing_zoom_selection = 1;
		}
		mouse_x1 = event->x;
		mouse_y1 = event->y;
		mouse_x2 = mouse_x1;
		mouse_y2 = mouse_y1;
	}
	else if(event->button == 3 && current_graph == GRAPH_TABLE)
	{
		Graphique* gr = &graph[current_graph];
		float xrange = gr->roi_xmax - gr->roi_xmin;
		float yrange = gr->roi_ymax - gr->roi_ymin;

		float x1 = gr->roi_xmin + (event->x - gr->bordure_pixel_x) / (gr->screen_width - 2 * gr->bordure_pixel_x) * xrange;
		float y1 = gr->roi_ymin + (event->y - gr->bordure_pixel_y) / (gr->screen_height - 2 * gr->bordure_pixel_y) * yrange;

		double dx = x1 - opponent_robot_pos.x;
		double dy = -y1 - opponent_robot_pos.y;

		// on verifie qu'on clic a peu pres sur le robot
		if( sqrt(dx*dx+dy*dy) < OPPONENT_PERIMETER )
		{
			move_oponent_robot = 1;
			mouse_x1 = event->x;
			mouse_y1 = event->y;
			mouse_x2 = mouse_x1;
			mouse_y2 = mouse_y1;
		}
		else
		{
			// c'est pas un robot, on reset la roi
			graph[current_graph].reset_roi();
		}
	}
	else
	{
		graph[current_graph].reset_roi();
	}
	gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
}

static void mounse_release(GtkWidget* widget, GdkEventButton* event)
{
	if(event->button == 1 && current_graph != GRAPH_TABLE)
	{
		if( drawing_zoom_selection && mouse_x1 != mouse_x2 && mouse_y1 != mouse_y2)
		{
			graph[current_graph].zoom(mouse_x1, mouse_x2, screen_height - mouse_y1, screen_height - mouse_y2);
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
		if(current_graph == GRAPH_TABLE && move_oponent_robot == 1)
		{
			Graphique* gr = &graph[current_graph];

			float xrange = gr->roi_xmax - gr->roi_xmin;
			float yrange = gr->roi_ymax - gr->roi_ymin;

			float x1 = (mouse_x1 - gr->bordure_pixel_x) / (gr->screen_width - 2 * gr->bordure_pixel_x) * xrange;
			float x2 = (mouse_x2 - gr->bordure_pixel_x) / (gr->screen_width - 2 * gr->bordure_pixel_x) * xrange;
			float y1 = (mouse_y1 - gr->bordure_pixel_y) / (gr->screen_height - 2 * gr->bordure_pixel_y) * yrange;
			float y2 = (mouse_y2 - gr->bordure_pixel_y) / (gr->screen_height - 2 * gr->bordure_pixel_y) * yrange;

			// on le met a sa position de depart
			Vect2 origin(opponent_robot_pos.x, opponent_robot_pos.y);
			VectPlan delta(x2 - x1, y1 - y2, 0);
			opponent_robot_pos.x += delta.x;
			opponent_robot_pos.y += delta.y;
			if(simulation)
			{
				qemu->move_object(QEMU_OPPONENT_ID, origin, delta);
			}

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

		if(current_graph == GRAPH_TABLE && move_oponent_robot == 1)
		{
			Graphique* gr = &graph[current_graph];

			float xrange = gr->roi_xmax - gr->roi_xmin;
			float yrange = gr->roi_ymax - gr->roi_ymin;

			float x1 = (mouse_x1 - gr->bordure_pixel_x) / (gr->screen_width - 2 * gr->bordure_pixel_x) * xrange;
			float x2 = (mouse_x2 - gr->bordure_pixel_x) / (gr->screen_width - 2 * gr->bordure_pixel_x) * xrange;
			float y1 = (mouse_y1 - gr->bordure_pixel_y) / (gr->screen_height - 2 * gr->bordure_pixel_y) * yrange;
			float y2 = (mouse_y2 - gr->bordure_pixel_y) / (gr->screen_height - 2 * gr->bordure_pixel_y) * yrange;

			// on le met a sa position de depart
			Vect2 origin(opponent_robot_pos.x, opponent_robot_pos.y);
			VectPlan delta(x2 - x1, y1 - y2, 0);
			opponent_robot_pos.x += delta.x;
			opponent_robot_pos.y += delta.y;
			if(simulation)
			{
				qemu->move_object(QEMU_OPPONENT_ID, origin, delta);
			}
			mouse_x1 = event->x;
			mouse_y1 = event->y;
			gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
		}
	}
}

static void mouse_scroll(GtkWidget* widget, GdkEventScroll* event)
{
	if(current_graph == GRAPH_TABLE)
	{
		if( event->direction == GDK_SCROLL_UP )
		{
			gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
			glplot_table_scale *= 1.25;
		}
		else if( event->direction == GDK_SCROLL_DOWN )
		{
			gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
			glplot_table_scale /= 1.25;
		}
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
			graph[current_graph].zoomf(2);
			break;
		case GDK_KP_Add:
		case GDK_plus:
			graph[current_graph].zoomf(0.5);
			break;
		case GDK_Right:
			glplot_view.translate(-40,0,0);
			break;
		case GDK_Left:
			glplot_view.translate(40,0,0);
			break;
		case GDK_Up:
			glplot_view.translate(0,-40,0);
			break;
		case GDK_Down:
			glplot_view.translate(0,40,0);
			break;
		case GDK_r:
			res = pthread_mutex_lock(&robotItf->mutex);
			if(res == 0)
			{
				robotItf->control_usb_data_count = 0;
				pthread_mutex_unlock(&robotItf->mutex);
			}
			break;
		case GDK_u:
			graph[current_graph].reset_roi();
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

static void simu_toggle_btn_color(GtkWidget* widget, gpointer arg)
{
	(void) widget;
	(void) arg;
	if(qemu)
	{
		//qemu->set_io(GPIO_IN_BTN1, true);
		//qemu->set_io(GPIO_IN_BTN1, false);
	}
}

static void simu_go(GtkWidget* widget, gpointer arg)
{
	(void) widget;
	(void) arg;
	static bool go = false;
	if(qemu)
	{
		go = !go;
		qemu->set_io(GPIO_IN_GO, go);
	}
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
					robotItf->pince(PINCE_OPEN, PINCE_OPEN);
				}
				break;
			case 1: // B (xbox)
				if( v )
				{
					robotItf->pince(PINCE_CLOSE, PINCE_CLOSE);
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
				robotItf->motion_set_speed(VectPlan(0,0,1), val);
				break;
			case 2: // gachette gauche (xbox)
				robotItf->motion_set_speed(VectPlan(1,0,0), val*1000);
				break;
			case 5: // gachette droite (xbox)
				robotItf->motion_set_speed(VectPlan(0,1,0), val*1000);
				break;
			default:
				break;
		}
		//log_info("axe %d : %f", event, val);
	}
}

static void glprint(float x, float y, GLuint base, const char* buffer, int size)
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

static void glPrintf(float x, float y, GLuint base, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x, y, base, buffer, size);
}

static void glPrintf_xcenter_ycenter(float x, float y, float x_ratio,float y_ratio, GLuint base, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x- x_ratio * size/2.0f * font_width, y - font_digit_height / 2.0f * y_ratio, base, buffer, size);
}

static void glPrintf_xright2_ycenter(float x, float y, float x_ratio,float y_ratio, GLuint base, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x - x_ratio * (size+2) * font_width, y - font_digit_height / 2.0f * y_ratio, base, buffer, size);
}

static void glPrintf_xright2_yhigh(float x, float y, float x_ratio,float y_ratio, GLuint base, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x - x_ratio * (size+2) * font_width, y - font_digit_height * y_ratio, base, buffer, size);
}

static void glPrintf_xcenter_yhigh2(float x, float y, float x_ratio,float y_ratio, GLuint base, const char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x - x_ratio * size/2.0f * font_width, y - 2*font_digit_height * y_ratio, base, buffer, size);
}
