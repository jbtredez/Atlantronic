#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <gdk/gdkkeysyms.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>
#include <X11/Intrinsic.h>
#include <stdlib.h>
#include <math.h>
#include <sys/stat.h>
#include <string.h>

#include "linux/tools/qemu.h"
#include "linux/tools/cmd.h"
#include "linux/tools/graphique.h"
#include "linux/tools/joystick.h"
#include "linux/tools/opengl/table_scene.h"
#include "linux/tools/opengl/gl_font.h"
#include "linux/tools/opengl/gltools.h"
#include "kernel/math/matrix_homogeneous.h"

// limitation du rafraichissement
// hokuyo => 10fps. On met juste un peu plus
#define MAX_FPS    11
#define QEMU_OPPONENT_ID   (TABLE_OBJ_SIZE)

enum
{
	GRAPH_TABLE = 0,
	GRAPH_HOKUYO_HIST,
	GRAPH_SPEED_DIST,
	GRAPH_NUM,
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
	SUBGRAPH_MOTION_V1_MES,
	SUBGRAPH_MOTION_V2_MES,
	SUBGRAPH_MOTION_VBAT,
	SUBGRAPH_MOTION_I1,
	SUBGRAPH_MOTION_I2,
	SUBGRAPH_MOTION_I3,
	SUBGRAPH_MOTION_I4,
	SUBGRAPH_MOTION_NUM,
};

static GlFont glfont;
static char font_name[] = "fixed";
static int screen_width = 0;
static int screen_height = 0;
static RobotInterface* robotItf;
static Qemu* qemu;
static float mouse_x1 = 0;
static float mouse_y1 = 0;
static float mouse_x2 = 0;
static float mouse_y2 = 0;
static float mouse_scroll_x1 = 0;
static float mouse_scroll_y1 = 0;
static int drawing_zoom_selection = 0;
static int current_graph = GRAPH_TABLE;
static GtkWidget* opengl_window = NULL;
static GtkWidget* main_window = NULL;
static int simulation = 0;
static char* atlantronicPath;
static bool glplot_show_legend = false;
static TableScene tableScene;
Graphique graph[GRAPH_NUM];
struct joystick joystick;
static int glplot_init_done = 0;

static void close_gtk(GtkWidget* widget, gpointer arg);
static void select_graph(GtkWidget* widget, gpointer arg);
static void show_legend(GtkWidget* widget, gpointer arg);
static void select_active_courbe(GtkWidget* widget, gpointer arg);
static void init(GtkWidget* widget, gpointer arg);
static gboolean config(GtkWidget* widget, GdkEventConfigure* ev, gpointer arg);
static gboolean display(GtkWidget* widget, GdkEventExpose* ev, gpointer arg);
static void drawScene(GLenum mode);
static void mouse_press(GtkWidget* widget, GdkEventButton* event);
static void mouse_release(GtkWidget* widget, GdkEventButton* event);
static gboolean keyboard_press(GtkWidget* widget, GdkEventKey* event, gpointer arg);
static gboolean keyboard_release(GtkWidget* widget, GdkEventKey* event, gpointer arg);
static void toggle_color(GtkWidget* widget, gpointer arg);
static void toggle_go(GtkWidget* widget, gpointer arg);
static void joystick_event(int event, float val);
static void mouse_move(GtkWidget* widget, GdkEventMotion* event);
static void mouse_scroll(GtkWidget* widget, GdkEventScroll* event);
void gtk_end();

#define OPPONENT_R         150.0f

Vect2 opponent_robot_pt[] =
{
	Vect2( OPPONENT_R, 0),
	Vect2( OPPONENT_R * 0.707106781, OPPONENT_R * 0.707106781),
	Vect2( 0, OPPONENT_R),
	Vect2( -OPPONENT_R * 0.707106781, OPPONENT_R * 0.707106781),
	Vect2( -OPPONENT_R, 0),
	Vect2( -OPPONENT_R * 0.707106781, -OPPONENT_R * 0.707106781),
	Vect2( 0, -OPPONENT_R),
	Vect2( OPPONENT_R * 0.707106781, -OPPONENT_R * 0.707106781),
	Vect2( OPPONENT_R, 0),
};

struct polyline oponent_robot =
{
		opponent_robot_pt,
		sizeof(opponent_robot_pt) / sizeof(opponent_robot_pt[0])
};

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
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_STATIC_ELM, "Table 2d du robot", 1, 0, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_TABLE3D, "Table 3d simu", 1, 0, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_ELM3D, "Elements 3d simu", 1, 0, 0, 0);
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
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_V1_MES, "v1_mes", 0, 0, 1, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_V2_MES, "v2_mes", 0, 0, 0.5, 1);
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
	g_signal_connect(G_OBJECT(opengl_window), "button_press_event", G_CALLBACK(mouse_press), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "button_release_event", G_CALLBACK(mouse_release), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "motion_notify_event", G_CALLBACK(mouse_move), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "scroll-event", G_CALLBACK(mouse_scroll), NULL);
	g_signal_connect_swapped(G_OBJECT(main_window), "key_press_event", G_CALLBACK(keyboard_press), opengl_window);
	g_signal_connect_swapped(G_OBJECT(main_window), "key_release_event", G_CALLBACK(keyboard_release), opengl_window);


	GtkWidget* menu0 = gtk_menu_bar_new();	// menu "niveau 0" (ie: barre de menu)
	GtkWidget* menu1 = gtk_menu_new();	// menu "niveau 1"
	GtkWidget* menuObj;
#if 0
	// menu Fichier
	menuObj = gtk_menu_item_new_with_label("Fichier");
	gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu0), menuObj);

	menuObj = gtk_menu_item_new_with_label("Quitter");
	g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(close_gtk), (GtkWidget*) main_window);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);
#endif

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

	GtkWidget* toolbar = gtk_toolbar_new();
	gtk_toolbar_set_style(GTK_TOOLBAR(toolbar), GTK_TOOLBAR_ICONS);
	gtk_container_set_border_width(GTK_CONTAINER(toolbar), 0);
	gtk_toolbar_set_orientation(GTK_TOOLBAR(toolbar), GTK_ORIENTATION_VERTICAL);
	gtk_toolbar_set_icon_size(GTK_TOOLBAR(toolbar), GTK_ICON_SIZE_SMALL_TOOLBAR);

	GtkToolItem* toolBarBtn = gtk_tool_button_new_from_stock(GTK_STOCK_QUIT);
	g_signal_connect(G_OBJECT(toolBarBtn), "clicked", G_CALLBACK(close_gtk), (GtkWidget*) main_window);
	gtk_toolbar_insert(GTK_TOOLBAR(toolbar), toolBarBtn, -1);

	toolBarBtn = gtk_tool_button_new_from_stock(GTK_STOCK_SELECT_COLOR);
	g_signal_connect(G_OBJECT(toolBarBtn), "clicked", G_CALLBACK(toggle_color), NULL);
	gtk_toolbar_insert(GTK_TOOLBAR(toolbar), toolBarBtn, -1);

	toolBarBtn = gtk_tool_button_new_from_stock(GTK_STOCK_APPLY);
	g_signal_connect(G_OBJECT(toolBarBtn), "clicked", G_CALLBACK(toggle_go), NULL);
	gtk_toolbar_insert(GTK_TOOLBAR(toolbar), toolBarBtn, -1);

	// rangement des éléments dans la fenetre
	// vbox la fenetre principale : menu + fenetre opengl
	GtkWidget* main_vbox = gtk_vbox_new(FALSE, 0);
	GtkWidget* main_hbox = gtk_hbox_new(FALSE, 0);
	gtk_container_add(GTK_CONTAINER(main_window), main_hbox);

	gtk_box_pack_start(GTK_BOX(main_hbox), toolbar, FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(main_hbox), main_vbox, TRUE, TRUE, 0);

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
		VectPlan pos(1250, 0, M_PI/2);
		pos.symetric(COLOR_GREEN);
		qemu->setPosition(pos);

		// ajout de la table dans qemu
		for(i = 0; i < TABLE_OBJ_SIZE; i++)
		{
			qemu->add_object(table_obj[i]);
		}

		// ajout d'un robot adverse
		qemu->add_object(oponent_robot);

		// on le met a sa position de depart
		Vect2 origin(0, 0);
		qemu->move_object(QEMU_OPPONENT_ID, origin, tableScene.getOpponentPosition());
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

	bool res = glfont.init(font_name);
	if( ! res )
	{
		exit(-1);
	}

	for(i = 0; i < GRAPH_NUM; i++)
	{
		graph[i].set_border(10 * glfont.width, glfont.height*3);
	}

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH);

	res = tableScene.init(&glfont, robotItf);
	if( ! res )
	{
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
		draw_plus(x, roi_ymin, glfont.width*ratio_x, glfont.width*ratio_y);
		glfont.glPrintf_xcenter_yhigh2(x, roi_ymin, ratio_x, ratio_y, "%g", x);
	}
	for(x = -dx; x >= roi_xmin; x-=dx)
	{
		draw_plus(x, roi_ymin, glfont.width*ratio_x, glfont.width*ratio_y);
		glfont.glPrintf_xcenter_yhigh2(x, roi_ymin, ratio_x, ratio_y, "%g", x);
	}

	// axe y
	float dy = graph->tics_dy;
	float y;
	for(y = 0; y <= roi_ymax; y+=dy)
	{
		draw_plus(roi_xmin, y, glfont.width*ratio_x, glfont.width*ratio_y);
		glfont.glPrintf_xright2_ycenter(roi_xmin, y, ratio_x, ratio_y, "%g", y);
	}
	for(y = -dy; y >= roi_ymin; y-=dy)
	{
		draw_plus(roi_xmin, y, glfont.width*ratio_x, glfont.width*ratio_y);
		glfont.glPrintf_xright2_ycenter(roi_xmin, y, ratio_x, ratio_y, "%g", y);
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
			glfont.glPrintf_xright2_yhigh(graph->roi_xmax, graph->roi_ymax + dy, graph->ratio_x, graph->ratio_y, "%s", graph->courbes_names[i]);
			dy -= 2*glfont.digitHeight * graph->ratio_y;
		}
	}
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
			draw_plus(i, robotItf->hokuyo_scan[HOKUYO1].distance[i], 0.25*glfont.width*ratio_x, 0.25*glfont.width*ratio_y);
		}
	}

	if( graph->courbes_activated[GRAPH_HOKUYO2_HIST] )
	{
		glColor3fv(&graph->color[3*GRAPH_HOKUYO2_HIST]);
		for(i = 0; i < HOKUYO_NUM_POINTS; i++)
		{
			draw_plus(i, robotItf->hokuyo_scan[HOKUYO2].distance[i], 0.25*glfont.width*ratio_x, 0.25*glfont.width*ratio_y);
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
			draw_plus(5*i, v.norm(), 0.25*glfont.width*ratio_x, 0.25*glfont.width*ratio_y);
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
			draw_plus(5*i, v.theta*1000.0f, 0.25*glfont.width*ratio_x, 0.25*glfont.width*ratio_y);
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
			draw_plus(5*i, v.norm(), 0.25*glfont.width*ratio_x, 0.25*glfont.width*ratio_y);
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
			draw_plus(5*i, v.theta*1000.0f, 0.25*glfont.width*ratio_x, 0.25*glfont.width*ratio_y);
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
				draw_plus(5*i, robotItf->control_usb_data[i].cons_motors_v[j], 0.25*glfont.width*ratio_x, 0.25*glfont.width*ratio_y);
			}
		}
		if( graph->courbes_activated[SUBGRAPH_MOTION_V1_MES + j] )
		{
			glColor3fv(&graph->color[3*SUBGRAPH_MOTION_V1_MES + j]);
			for(i=0; i < robotItf->control_usb_data_count; i++)
			{
				draw_plus(5*i, robotItf->control_usb_data[i].mes_motors[j].v, 0.25*glfont.width*ratio_x, 0.25*glfont.width*ratio_y);
			}
		}
	}

	if( graph->courbes_activated[SUBGRAPH_MOTION_VBAT] )
	{
		glColor3fv(&graph->color[3*(SUBGRAPH_MOTION_VBAT)]);
		for(int j=1; j < robotItf->control_usb_data_count; j++)
		{
			float v = robotItf->control_usb_data[j].vBat;
			draw_plus(5*j, v, 0.25*glfont.width*ratio_x, 0.25*glfont.width*ratio_y);
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
				draw_plus(5*j, 1000*current, 0.25*glfont.width*ratio_x, 0.25*glfont.width*ratio_y);
			}
		}
	}
}

static gboolean display(GtkWidget* widget, GdkEventExpose* ev, gpointer arg)
{
	(void) ev;
	(void) arg;
	GdkGLContext* glcontext = gtk_widget_get_gl_context(widget);
	GdkGLDrawable* gldrawable = gtk_widget_get_gl_drawable(widget);

	if(!gdk_gl_drawable_gl_begin(gldrawable, glcontext))
	{
		return FALSE;
	}

	drawScene(GL_RENDER);

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

static void drawScene(GLenum mode)
{
	if( current_graph == GRAPH_TABLE)
	{
		tableScene.draw(mode, &graph[GRAPH_TABLE]);
	}
	else
	{
		// efface le frame buffer
		glClearColor(1,1,1,1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		int res = pthread_mutex_lock(&robotItf->mutex);
		if(res == 0)
		{
			if(current_graph == GRAPH_SPEED_DIST)
			{
				graph[current_graph].resize_axis_x(0, robotItf->control_usb_data_count * 5);
			}

			if( current_graph != GRAPH_TABLE)
			{
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				glOrtho(graph[current_graph].plot_xmin, graph[current_graph].plot_xmax, graph[current_graph].plot_ymin, graph[current_graph].plot_ymax, 0, 1);
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();
				glDisable(GL_LIGHTING);
				glDisable(GL_DEPTH_TEST);
			}

			switch(current_graph)
			{
				default:
				case GRAPH_TABLE:
					break;
				case GRAPH_HOKUYO_HIST:
					plot_hokuyo_hist(&graph[current_graph]);
					break;
				case GRAPH_SPEED_DIST:
					plot_speed_dist(&graph[current_graph]);
					break;
			}

			pthread_mutex_unlock(&robotItf->mutex);
		}
	}

	glColor3f(0,0,0);
	plot_axes(&graph[current_graph]);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(graph[current_graph].roi_xmin, graph[current_graph].roi_xmax, graph[current_graph].roi_ymin, graph[current_graph].roi_ymax, 0, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if( glplot_show_legend )
	{
		plot_legende(&graph[current_graph]);
	}

	if( drawing_zoom_selection )
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0, screen_width, screen_height, 0, 0, 1);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		glColor3f(0,0,0);

		glBegin(GL_LINE_STRIP);
		glVertex2f(mouse_x1, mouse_y1);
		glVertex2f(mouse_x2, mouse_y1);
		glVertex2f(mouse_x2, mouse_y2);
		glVertex2f(mouse_x1, mouse_y2);
		glVertex2f(mouse_x1, mouse_y1);
		glEnd();
	}
}

static void mouse_press(GtkWidget* widget, GdkEventButton* event)
{
	if(event->button == 1 )
	{
		if( current_graph == GRAPH_TABLE )
		{
			tableScene.mouseSelect(event->x, event->y, &graph[GRAPH_TABLE]);
		}
		else
		{
			drawing_zoom_selection = 1;
		}
		mouse_x1 = event->x;
		mouse_y1 = event->y;
		mouse_x2 = mouse_x1;
		mouse_y2 = mouse_y1;
	}
	else if(event->button == 2)
	{
		mouse_scroll_x1 = event->x;
		mouse_scroll_y1 = event->y;
	}
	else
	{
		graph[current_graph].reset_roi();
	}
	gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
}

static void mouse_release(GtkWidget* widget, GdkEventButton* event)
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
	else if(event->button == 2)
	{
		mouse_scroll_x1 = 0;
		mouse_scroll_y1 = 0;
	}
}

static void mouse_move(GtkWidget* widget, GdkEventMotion* event)
{
	static VectPlan opponentPos = tableScene.getOpponentPosition();
	if(event->state & GDK_BUTTON1_MASK)
	{
		mouse_x2 = event->x;
		mouse_y2 = event->y;
		if(current_graph == GRAPH_TABLE)
		{
			tableScene.mouseMoveSelection(event->x, event->y);
			VectPlan newOpponentPos = tableScene.getOpponentPosition();
			if(simulation && (newOpponentPos.x != opponentPos.x || newOpponentPos.y != opponentPos.y || newOpponentPos.theta != opponentPos.theta))
			{
				Vect2 origin(opponentPos.x, opponentPos.theta);
				qemu->move_object(QEMU_OPPONENT_ID, origin, newOpponentPos - opponentPos);
				opponentPos = newOpponentPos;
			}
		}
		gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
	}
	else if(event->state & GDK_BUTTON2_MASK)
	{
		if( current_graph == GRAPH_TABLE )
		{
			float dx = (event->x - mouse_scroll_x1) / graph[current_graph].screen_width;
			float dy = -(event->y - mouse_scroll_y1) / graph[current_graph].screen_height;
			tableScene.rotateView(dx, dy);
			mouse_scroll_x1 = event->x;
			mouse_scroll_y1 = event->y;
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
			tableScene.translateView(0, 0, 200);
		}
		else if( event->direction == GDK_SCROLL_DOWN )
		{
			tableScene.translateView(0, 0, -200);
		}
	}
	gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
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
			tableScene.translateView(40, 0, 0);
			break;
		case GDK_Left:
			tableScene.translateView(-40, 0, 0);
			break;
		case GDK_Up:
			tableScene.translateView(0, 0, -40);
			break;
		case GDK_Down:
			tableScene.translateView(0, 0, 40);
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

static void toggle_color(GtkWidget* /*widget*/, gpointer /*arg*/)
{
	static int color = COLOR_GREEN;
	static bool ioColor = false;

	if( color == COLOR_GREEN )
	{
		color = COLOR_YELLOW;
	}
	else
	{
		color = COLOR_GREEN;
	}

	if(qemu)
	{
		// simulation : on change l'io
		ioColor = !ioColor;
		VectPlan pos(1250, 0, M_PI/2);
		pos.symetric(color);
		qemu->setPosition(pos);
		qemu->set_io(GPIO_MASK(IO_COLOR), ioColor);
	}
	else
	{
		// en reel, on passe par l'interface de com
		robotItf->color(color);
	}
}

static void toggle_go(GtkWidget* /*widget*/, gpointer /*arg*/)
{
	static bool go = false;
	if(qemu)
	{
		// simulation
		go = !go;
		qemu->set_io(GPIO_MASK_IN_GO, go);
	}
	else
	{
		robotItf->go();
	}
}

static void joystick_event(int event, float val)
{
	if(event & JOYSTICK_BTN_BASE)
	{
		//int v = rint(val);
		// c'est un bouton
		switch(event & ~JOYSTICK_BTN_BASE)
		{
			case 0: // A (xbox)
				break;
			case 1: // B (xbox)
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
