#include <gtk/gtk.h>
#ifndef GTK3
#include <gtkgl/gtkglarea.h>
#endif
#include <gdk/gdkkeysyms.h>

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
#include "kernel/math/matrix_homogeneous.h"
#include "opengl/main_shader.h"
#include "point_texture.h"
#include "linux/tools/Robot.h"

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

static MainShader shader;
static GlFont glfont;
static char fontName[] = "DejaVuSerif.ttf";
static int fontSize = 12;
static int screen_width = 0;
static int screen_height = 0;
static Robot* robot;
static int robotCount = 1;
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
static bool glplot_show_legend = false;
static TableScene tableScene;
Graphique graph[GRAPH_NUM];
struct joystick joystick;
static int glplot_init_done = 0;
#if 1
static VectPlan qemuStartPos(1000, -600, -M_PI/2);
#else
static VectPlan qemuStartPos(1200, 0, 0);
#endif
static GlObjectBasic selectionObject;
static GlObjectBasic axisObject;
static GlObjectBasic graphPointObject;
static int color = COLOR_GREEN;
static bool ioColor = true;
static GLuint pointTextureId;

static void select_graph(GtkWidget* widget, gpointer arg);
static void select_configuration(GtkWidget* widget, gpointer arg);
static void show_legend(GtkWidget* widget, gpointer arg);
static void select_active_courbe(GtkWidget* widget, gpointer arg);
static void init(GtkGLArea *widget);
static gboolean config(GtkWidget* widget, GdkEventConfigure* ev, gpointer arg);
static gboolean render(GtkWidget* widget, GdkEventExpose* ev, gpointer arg);
static void mouse_press(GtkWidget* widget, GdkEventButton* event);
static void mouse_release(GtkWidget* widget, GdkEventButton* event);
static gboolean keyboard_press(GtkWidget* widget, GdkEventKey* event, gpointer arg);
static gboolean keyboard_release(GtkWidget* widget, GdkEventKey* event, gpointer arg);
static void toggle_color(GtkWidget* widget, gpointer arg);
static void toggle_go(GtkWidget* widget, gpointer arg);
static void reboot_robot(GtkWidget* widget, gpointer arg);
static void joystick_event(int event, float val);
static void mouse_move(GtkWidget* widget, GdkEventMotion* event);
static void mouse_scroll(GtkWidget* widget, GdkEventScroll* event);
static void plot_legende(Graphique* graph);
static void plot_hokuyo_hist(Graphique* graph);
static void plot_speed_dist(Graphique* graph);
static void plot_axes_text(Graphique* graph);
static void plot_axes_lines(Graphique* graph);

static void qemu_set_parameters();
static void gtk_end();

int glplot_main(bool cli, Robot* _robot, int RobotCount)
{
	robot = _robot;
	robotCount = RobotCount;

	graph[GRAPH_TABLE].init("Table", -1600, 1600, -1100, 1100, 800, 600, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_ROBOT, "Robot", 1, 0, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_OPPONENT_ROBOT, "Opponent Robot", 1, 0, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_STATIC_ELM, "Table 2d du robot", 1, 0, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_TABLE3D, "Table 3d simu", 1, 0, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_ELM3D, "Elements 3d simu", 1, 0, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_HOKUYO1, "Hokuyo 1", 1, 1, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_HOKUYO1_SEG, "Hokuyo 1 - poly", 1, 0, 1, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_HOKUYO2, "Hokuyo 2", 1, 0.5, 0.5, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_RPLIDAR, "Rplidar", 1, 1, 0, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_POS_CONS, "Position (consigne)", 1, 0, 0, 1);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_POS_MES, "Position (mesure)", 1, 0, 1, 0);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_GRAPH, "Graph", 1, 1, 1, 1);
	graph[GRAPH_TABLE].add_courbe(SUBGRAPH_TABLE_GRAPH_LINK, "Graph links", 1, 0, 1, 1);

	graph[GRAPH_HOKUYO_HIST].init("Hokuyo", 0, 682, 0, 4100, 800, 600, 0, 0);
	graph[GRAPH_HOKUYO_HIST].add_courbe(GRAPH_HOKUYO1_HIST, "Hokuyo 1", 1, 1, 0, 0);
	graph[GRAPH_HOKUYO_HIST].add_courbe(GRAPH_HOKUYO2_HIST, "Hokuyo 2", 1, 0, 0, 1);

	graph[GRAPH_SPEED_DIST].init("Control", 0, 90000, -3100, 3100, 800, 600, 0, 0);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_SPEED_DIST_MES, "Vitesse d'avance mesuree", 1, 0, 1, 0);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_SPEED_DIST_CONS, "Vitesse d'avance de consigne", 1, 0, 0, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_SPEED_ROT_MES, "Vitesse de rotation mesuree", 1, 0.5, 0.5, 0);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_SPEED_ROT_CONS, "Vitesse de rotation de consigne", 1, 1, 0, 0);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_V1, "v1", 0, 1, 0, 0);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_V2, "v2", 0, 0, 1, 0);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_V1_MES, "v1_mes", 0, 0, 0, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_V2_MES, "v2_mes", 0, 1, 0, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_VBAT, "vBat", 0, 1, 0, 0);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_I1, "i1", 0, 0, 0, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_I2, "i2", 0, 0, 0, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_I3, "i3", 0, 0, 0, 1);
	graph[GRAPH_SPEED_DIST].add_courbe(SUBGRAPH_MOTION_I4, "i4", 0, 0, 0, 1);

	gtk_init(0, NULL);
#ifndef GTK3
	gdk_gl_query();
#endif
	// création de la fenêtre
	main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title(GTK_WINDOW(main_window),"USB Interface");
	gtk_window_set_default_size(GTK_WINDOW(main_window), 400, 300);

	g_signal_connect(G_OBJECT(main_window), "delete_event", G_CALLBACK(gtk_end), NULL);
	g_signal_connect(G_OBJECT(main_window), "destroy", G_CALLBACK(gtk_end), NULL);

	// fenetre opengl
	// config de opengl
#ifndef GTK3
	int attribute[] = { GDK_GL_RGBA, GDK_GL_DOUBLEBUFFER, GDK_GL_DEPTH_SIZE, 1, GDK_GL_STENCIL_SIZE, 1, GDK_GL_NONE };
	opengl_window = gtk_gl_area_share_new (attribute, (GtkGLArea *) opengl_window);
#else
	opengl_window = gtk_gl_area_new();
	gtk_gl_area_set_has_depth_buffer( GTK_GL_AREA(opengl_window), true);
	//gtk_gl_area_set_has_alpha( GTK_GL_AREA(opengl_window), true);
	gtk_gl_area_set_has_stencil_buffer( GTK_GL_AREA(opengl_window), true);
#endif
	gtk_widget_set_size_request(opengl_window, 800, 600);

	gtk_widget_add_events(opengl_window, GDK_VISIBILITY_NOTIFY_MASK | GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK);

	g_signal_connect_after(G_OBJECT(opengl_window), "realize", G_CALLBACK(init), NULL);
#ifndef GTK3
	g_signal_connect(G_OBJECT(opengl_window), "configure_event", G_CALLBACK(config), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "expose_event", G_CALLBACK(render), NULL);
#else
	g_signal_connect(G_OBJECT(opengl_window), "resize", G_CALLBACK(config), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "render", G_CALLBACK(render), NULL);
#endif
	g_signal_connect(G_OBJECT(opengl_window), "button_press_event", G_CALLBACK(mouse_press), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "button_release_event", G_CALLBACK(mouse_release), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "motion_notify_event", G_CALLBACK(mouse_move), NULL);
	g_signal_connect(G_OBJECT(opengl_window), "scroll-event", G_CALLBACK(mouse_scroll), NULL);
	g_signal_connect_swapped(G_OBJECT(main_window), "key_press_event", G_CALLBACK(keyboard_press), opengl_window);
	g_signal_connect_swapped(G_OBJECT(main_window), "key_release_event", G_CALLBACK(keyboard_release), opengl_window);


	GtkWidget* menu0 = gtk_menu_bar_new();	// menu "niveau 0" (ie: barre de menu)
	GtkWidget* menu1 = NULL;	// menu "niveau 1"
	GtkWidget* menuObj;

	// menu courbe
	menu1 = gtk_menu_new();	// menu "niveau 1"
	menuObj = gtk_menu_item_new_with_label("Courbe");
	gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu0), menuObj);

	GSList *group = NULL;
	for(long i = 0; i < GRAPH_NUM; i++)
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

	for(long i = 0 ; i < GRAPH_NUM; i++)
	{
		menu1 = gtk_menu_new();	// menu "niveau 1"
		menuObj = gtk_menu_item_new_with_label( graph[i].name);
		gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);
		gtk_menu_shell_append(GTK_MENU_SHELL(menu0), menuObj);
		for(long j = 0; j < MAX_COURBES; j++)
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
	gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

	menu1 = gtk_menu_new();	// menu "niveau 1"
	menuObj = gtk_menu_item_new_with_label("Configuration");
	gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu0), menuObj);
	group = NULL;
	for(long i = 0; i < 5; i++)
	{
		char buffer[16];
		snprintf(buffer, sizeof(buffer), "%d", (int)i+1);
		menuObj = gtk_radio_menu_item_new_with_label(group, buffer);
		group = gtk_radio_menu_item_get_group (GTK_RADIO_MENU_ITEM (menuObj));
		g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(select_configuration), (void*)(i+1));
		gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);
		if( i == 0 )
		{
			gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM (menuObj), TRUE);
		}
	}

	GtkWidget* toolbar = gtk_toolbar_new();
	gtk_toolbar_set_style(GTK_TOOLBAR(toolbar), GTK_TOOLBAR_ICONS);
	gtk_container_set_border_width(GTK_CONTAINER(toolbar), 0);
	gtk_orientable_set_orientation(GTK_ORIENTABLE(toolbar), GTK_ORIENTATION_VERTICAL);
	gtk_toolbar_set_icon_size(GTK_TOOLBAR(toolbar), GTK_ICON_SIZE_SMALL_TOOLBAR);
	GtkToolItem* toolBarBtn = gtk_tool_button_new(NULL, NULL);
	gtk_tool_button_set_icon_name( GTK_TOOL_BUTTON(toolBarBtn), "application-exit");
	g_signal_connect(G_OBJECT(toolBarBtn), "clicked", G_CALLBACK(gtk_end), NULL);
	gtk_toolbar_insert(GTK_TOOLBAR(toolbar), toolBarBtn, -1);

	GtkWidget* reloadBtn = gtk_button_new_with_label("Recharger");
	g_signal_connect(G_OBJECT(reloadBtn), "clicked", G_CALLBACK(reboot_robot), NULL);
	gtk_button_set_relief(GTK_BUTTON(reloadBtn), GTK_RELIEF_NONE);

#ifndef GTK3
	GdkColor green = {0, 0, 65535, 0};
	GtkWidget* switchColorBtn = gtk_color_button_new_with_color(&green);
#else
	GdkRGBA green = {0, 1, 0, 1};
	GtkWidget* switchColorBtn = gtk_color_button_new_with_rgba(&green);
#endif
	GtkColorButtonClass* switchColorBtnClass = GTK_COLOR_BUTTON_GET_CLASS(switchColorBtn);
	GtkButtonClass* switchColorBtnClass2 = GTK_BUTTON_CLASS(switchColorBtnClass);
	switchColorBtnClass2->clicked = NULL; // on vire le handler par defaut qui affiche une popup de selection de couleur
	g_signal_connect(G_OBJECT(switchColorBtn), "clicked", G_CALLBACK(toggle_color), NULL);
	gtk_button_set_relief(GTK_BUTTON(switchColorBtn), GTK_RELIEF_NONE);

	GtkWidget* goBtn = gtk_button_new_with_label("Go");
	g_signal_connect(G_OBJECT(goBtn), "clicked", G_CALLBACK(toggle_go), NULL);
	gtk_button_set_relief(GTK_BUTTON(goBtn), GTK_RELIEF_NONE);

	// rangement des éléments dans la fenetre
	// vbox la fenetre principale : menu + fenetre opengl
#ifndef GTK3
	GtkWidget* main_vbox = gtk_vbox_new(FALSE, 0);
	GtkWidget* main_hbox = gtk_hbox_new(FALSE, 0);
	GtkWidget* main_vboxToolBar = gtk_vbox_new(FALSE, 0);
#else
	GtkWidget* main_vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
	GtkWidget* main_hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
	GtkWidget* main_vboxToolBar = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
#endif
	gtk_container_add(GTK_CONTAINER(main_window), main_hbox);
	gtk_box_pack_start(GTK_BOX(main_hbox), main_vboxToolBar, FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(main_vboxToolBar), reloadBtn, FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(main_vboxToolBar), switchColorBtn, FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(main_vboxToolBar), goBtn, FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(main_vboxToolBar), toolbar, TRUE, TRUE, 0);

	gtk_box_pack_start(GTK_BOX(main_hbox), main_vbox, TRUE, TRUE, 0);

	gtk_box_pack_start(GTK_BOX(main_vbox), menu0, FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(main_vbox), opengl_window, TRUE, TRUE, 0);

	gtk_widget_show_all(main_window);

	joystick_init(&joystick, "/dev/input/js0", joystick_event);

	if( cli )
	{
		Qemu* qemu = NULL;
		if( robot[ROBOT_SELECTED].m_qemu.isInitDone() )
		{
			qemu = &robot[ROBOT_SELECTED].m_qemu;
		}
		cmd_init(&robot[ROBOT_SELECTED].m_robotItf, qemu, gtk_end);
	}

	qemu_set_parameters();

	gtk_main();

	joystick_destroy(&joystick);

	return 0;
}

void qemu_set_parameters()
{
	for(int i = 0; i < robotCount ; i++)
	{
		if( robot[i].m_simulation )
		{
			robot[i].m_qemu.setPosition(qemuStartPos.symetric(color));
			robot[i].m_qemu.setIo(GPIO_MASK(IO_COLOR), ioColor);
		}
	}

	tableScene.initQemuObjects();
}

static gboolean gtk_end_from_gtk_thread(gpointer /*data*/)
{
	gtk_main_quit();
	return G_SOURCE_REMOVE;
}

void gtk_end()
{
	gdk_threads_add_idle(gtk_end_from_gtk_thread, NULL);
}

static gboolean redraw(gpointer data)
{
	GtkWidget* widget = (GtkWidget*)data;
	gtk_widget_queue_draw(widget);
	return G_SOURCE_REMOVE;
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
			gdk_threads_add_idle(redraw, opengl_window);
			last_plot = current;
		}
	}
}

static void select_graph(GtkWidget* widget, gpointer arg)
{
	(void) widget;

	unsigned long id = (unsigned long) arg;
	if(id < GRAPH_NUM)
	{
		current_graph = id;
	}
	gtk_widget_queue_draw(opengl_window);
}

static void select_configuration(GtkWidget* widget, gpointer arg)
{
	(void) widget;

	unsigned long id = (unsigned long) arg;
	tableScene.selectConfiguration(id);
	gtk_widget_queue_draw(opengl_window);
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

	gtk_widget_queue_draw(opengl_window);
}

static void init(GtkGLArea *area)
{
	int i;
#ifndef GTK3
	if(!gtk_gl_area_make_current (area)) return;
#else
	gtk_gl_area_make_current(area);
	if (gtk_gl_area_get_error (area) )
	{
		return;
	}
#endif
	int res = glfont.init(fontName, fontSize);
	if( res )
	{
		exit(-1);
	}

	for(i = 0; i < GRAPH_NUM; i++)
	{
		graph[i].set_border(10 * glfont.width, glfont.height*3);
	}

	res = shader.init();
	if( res )
	{
		exit(-1);
	}

	res = tableScene.init(&glfont, robot, robotCount, &shader);
	if( ! res )
	{
		exit(-1);
	}

	float sel[] =
	{
		mouse_x1, mouse_y1,
		mouse_x2, mouse_y1,
		mouse_x2, mouse_y2,
		mouse_x1, mouse_y2,
		mouse_x1, mouse_y1
	};

	selectionObject.init(sel, 2, 5, &shader);

	float axis[] =
	{
		1, 0,
		0, 0,
		0, 1
	};
	axisObject.init(axis, 2, 3, &shader);

	float plus_pt[2*CONTROL_USB_DATA_MAX];
	memset(plus_pt, 0, sizeof(plus_pt));
	graphPointObject.init(plus_pt, 2, CONTROL_USB_DATA_MAX, &shader, true);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POINT_SPRITE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &pointTextureId);
	glBindTexture(GL_TEXTURE_2D, pointTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, pointTexture.width, pointTexture.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pointTexture.pixel_data);

	glplot_init_done = 1;
}

static gboolean config(GtkWidget* widget, GdkEventConfigure* ev, gpointer arg)
{
	(void) ev;
	(void) arg;

#ifndef GTK3
	if(!gtk_gl_area_make_current (GTK_GL_AREA (widget)))
	{
		return FALSE;
	}
#else
	gtk_gl_area_make_current(GTK_GL_AREA(widget));
	if (gtk_gl_area_get_error( GTK_GL_AREA(widget) ) )
	{
		return FALSE;
	}
#endif

	GtkAllocation alloc;
	gtk_widget_get_allocation(widget, &alloc);
	screen_width = alloc.width;
	screen_height = alloc.height;

	int i;
	for( i = 0; i < GRAPH_NUM; i++)
	{
		graph[i].resize_screen(screen_width, screen_height);
	}

	glViewport(0, 0, screen_width, screen_height);

	return TRUE;
}

static gboolean render(GtkWidget* widget, GdkEventExpose* /*ev*/, gpointer /*arg*/)
{
	Graphique* g = &graph[current_graph];
	RobotInterface* robotItf = &robot[ROBOT_SELECTED].m_robotItf;

	// on efface le frame buffer
	glClearStencil(0);
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);

	shader.use();
	shader.setSprite(0);
	if( current_graph == GRAPH_TABLE)
	{
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_STENCIL_TEST);
		glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
		glStencilFunc(GL_ALWAYS, 0, ~0);
		tableScene.draw(g);
	}
	else
	{

		int res = pthread_mutex_lock(&robotItf->mutex);
		if(res == 0)
		{
			if(current_graph == GRAPH_SPEED_DIST)
			{
				g->resize_axis_x(0, robotItf->control_usb_data_count * CONTROL_DT * 1000);
			}

			glm::mat4 projection = glm::ortho(g->plot_xmin, g->plot_xmax, g->plot_ymin, g->plot_ymax, 0.0f, 1.0f);
			shader.setProjection(projection);
			shader.setModelView(glm::mat4(1.0f));
			glDisable(GL_DEPTH_TEST);
			glDisable(GL_CULL_FACE);

			switch(current_graph)
			{
				default:
				case GRAPH_TABLE:
					break;
				case GRAPH_HOKUYO_HIST:
					plot_hokuyo_hist(g);
					break;
				case GRAPH_SPEED_DIST:
					plot_speed_dist(g);
					break;
			}

			pthread_mutex_unlock(&robotItf->mutex);
		}
	}

	shader.setColor(0, 0, 0);
	plot_axes_lines(g);

	if( drawing_zoom_selection )
	{
		glm::mat4 oldProjection = shader.getProjection();
		glm::mat4 oldModelView = shader.getModelView();
		glm::mat4 projection = glm::ortho(0.0f, (float)screen_width, (float)screen_height, 0.0f, 0.0f, 1.0f);
		shader.setProjection(projection);
		shader.setModelView(glm::mat4(1.0f));
		shader.setColor(0, 0, 0);

		float sel[] =
		{
			mouse_x1, mouse_y1,
			mouse_x2, mouse_y1,
			mouse_x2, mouse_y2,
			mouse_x1, mouse_y2,
			mouse_x1, mouse_y1
		};
		selectionObject.update(sel, sizeof(sel) / (2*sizeof(sel[0])));
		selectionObject.render(GL_LINE_STRIP);
		shader.setProjection(oldProjection);
		shader.setModelView(oldModelView);
	}

	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// affichage texte
	glfont.m_textShader.use();

	glfont.m_textShader.setProjection(shader.getProjection() * shader.getModelView());
	glfont.m_textShader.setColor(0,0,0,1);
	plot_axes_text(g);

	glm::mat4 projection = glm::ortho(g->plot_xmin, g->plot_xmax, g->plot_ymin, g->plot_ymax, 0.0f, 1.0f);
	glfont.m_textShader.setProjection(projection);

	if( current_graph == GRAPH_TABLE)
	{
		int res = pthread_mutex_lock(&robotItf->mutex);
		if(res == 0)
		{
			tableScene.printInfos(g);
			pthread_mutex_unlock(&robotItf->mutex);
		}
	}

	if( glplot_show_legend )
	{
		plot_legende(g);
	}

#ifndef GTK3
	gtk_gl_area_swap_buffers(GTK_GL_AREA(widget));
#else
	(void) widget;
#endif

	return TRUE;
}

static void plot_axes_lines(Graphique* graph)
{
	static float pt[2048];
	unsigned int ptCount = 0;
	float roi_xmin = graph->roi_xmin;
	float roi_xmax = graph->roi_xmax;
	float roi_ymin = graph->roi_ymin;
	float roi_ymax = graph->roi_ymax;

	float axis[] =
	{
		roi_xmax, roi_ymin,
		roi_xmin, roi_ymin,
		roi_xmin, roi_ymax,
	};
	axisObject.update(axis, sizeof(axis) / (2 * sizeof(axis[0])));
	axisObject.render(GL_LINE_STRIP);

	// axe x
	float dx = graph->tics_dx;
	float x;
	for(x = 0; x <= roi_xmax && ptCount < sizeof(pt)/(2*sizeof(pt[0])); x+=dx)
	{
		pt[2*ptCount] = x;
		pt[2*ptCount+1] = roi_ymin;
		ptCount++;
	}
	for(x = -dx; x >= roi_xmin && ptCount < sizeof(pt)/(2*sizeof(pt[0])); x-=dx)
	{
		pt[2*ptCount] = x;
		pt[2*ptCount+1] = roi_ymin;
		ptCount++;
	}

	// axe y
	float dy = graph->tics_dy;
	float y;
	for(y = 0; y <= roi_ymax && ptCount < sizeof(pt)/(2*sizeof(pt[0])); y+=dy)
	{
		pt[2*ptCount] = roi_xmin;
		pt[2*ptCount+1] = y;
		ptCount++;
	}
	for(y = -dy; y >= roi_ymin && ptCount < sizeof(pt)/(2*sizeof(pt[0])); y-=dy)
	{
		pt[2*ptCount] = roi_xmin;
		pt[2*ptCount+1] = y;
		ptCount++;
	}

	shader.setSprite(pointTexture.width);
	glBindTexture(GL_TEXTURE_2D, pointTextureId);
	graphPointObject.update(pt, ptCount);
	graphPointObject.render(GL_POINTS);
	shader.setSprite(0);
}

static void plot_axes_text(Graphique* graph)
{
	float roi_xmin = graph->roi_xmin;
	float roi_xmax = graph->roi_xmax;
	float roi_ymin = graph->roi_ymin;
	float roi_ymax = graph->roi_ymax;

	float ratio_x = graph->ratio_x;
	float ratio_y = graph->ratio_y;

	// axe x
	float dx = graph->tics_dx;
	float x;
	for(x = 0; x <= roi_xmax; x+=dx)
	{
		glfont.glPrintf_xcenter_yhigh2(x, roi_ymin, ratio_x, ratio_y, "%g", x);
	}
	for(x = -dx; x >= roi_xmin; x-=dx)
	{
		glfont.glPrintf_xcenter_yhigh2(x, roi_ymin, ratio_x, ratio_y, "%g", x);
	}

	// axe y
	float dy = graph->tics_dy;
	float y;
	for(y = 0; y <= roi_ymax; y+=dy)
	{
		glfont.glPrintf_xright2_ycenter(roi_xmin, y, ratio_x, ratio_y, "%g", y);
	}
	for(y = -dy; y >= roi_ymin; y-=dy)
	{
		glfont.glPrintf_xright2_ycenter(roi_xmin, y, ratio_x, ratio_y, "%g", y);
	}
}

static void plot_legende(Graphique* graph)
{
	int i = 0;
	int dy = 0;

	for(i = 0; i < MAX_COURBES; i++)
	{
		if( graph->courbes_activated[i] )
		{
			glfont.setColor(graph->color[3*i], graph->color[3*i+1], graph->color[3*i+2], 1);
			glfont.glPrintf_xright2_yhigh(graph->roi_xmax, graph->roi_ymax + dy, graph->ratio_x, graph->ratio_y, "%s", graph->courbes_names[i]);
			dy -= 2*glfont.digitHeight * graph->ratio_y;
		}
	}
}

static void plot_hokuyo_hist(Graphique* graph)
{
	static float pt[2*HOKUYO_NUM_POINTS];
	for(int i=0; i < HOKUYO_NUM_POINTS; i++)
	{
		pt[2*i] = i;
	}
	RobotInterface* robotItf = &robot[0].m_robotItf;

	shader.setSprite(pointTexture.width);
	glBindTexture(GL_TEXTURE_2D, pointTextureId);

	if( graph->courbes_activated[GRAPH_HOKUYO1_HIST] )
	{
		shader.setColor3f(&graph->color[3*GRAPH_HOKUYO1_HIST]);
		for(int i = 0; i < HOKUYO_NUM_POINTS; i++)
		{
			pt[2*i+1] = robotItf->hokuyo_scan[HOKUYO1].distance[i];
		}
		graphPointObject.update(pt, HOKUYO_NUM_POINTS);
		graphPointObject.render(GL_POINTS);
	}

	if( graph->courbes_activated[GRAPH_HOKUYO2_HIST] )
	{
		shader.setColor3f(&graph->color[3*GRAPH_HOKUYO2_HIST]);
		for(int i = 0; i < HOKUYO_NUM_POINTS; i++)
		{
			pt[2*i+1] = robotItf->hokuyo_scan[HOKUYO2].distance[i];
		}
		graphPointObject.update(pt, HOKUYO_NUM_POINTS);
		graphPointObject.render(GL_POINTS);
	}
	shader.setSprite(0);
}

static void plot_speed_dist(Graphique* graph)
{
	RobotInterface* robotItf = &robot[ROBOT_SELECTED].m_robotItf;

	static Vect2 pt[CONTROL_USB_DATA_MAX];
	for(int i=1; i < robotItf->control_usb_data_count; i++)
	{
		pt[i-1].x = 1000 * CONTROL_DT * i;
	}

	shader.setSprite(pointTexture.width);
	glBindTexture(GL_TEXTURE_2D, pointTextureId);

	if( graph->courbes_activated[SUBGRAPH_MOTION_SPEED_DIST_CONS] && robotItf->control_usb_data_count > 1)
	{
		shader.setColor3f(&graph->color[3*SUBGRAPH_MOTION_SPEED_DIST_CONS]);
		VectPlan old_cons = robotItf->control_usb_data[0].cons;
		for(int i=1; i < robotItf->control_usb_data_count; i++)
		{
			VectPlan cons = robotItf->control_usb_data[i].cons;
			VectPlan v = (cons - old_cons) / CONTROL_DT;
			pt[i-1].y = v.norm();
			old_cons = cons;
		}
		graphPointObject.update((float*)pt, robotItf->control_usb_data_count-1);
		graphPointObject.render(GL_POINTS);
	}

	if( graph->courbes_activated[SUBGRAPH_MOTION_SPEED_ROT_CONS] )
	{
		shader.setColor3f(&graph->color[3*SUBGRAPH_MOTION_SPEED_ROT_CONS]);
		VectPlan old_cons = robotItf->control_usb_data[0].cons;
		for(int i=1; i < robotItf->control_usb_data_count; i++)
		{
			VectPlan cons = robotItf->control_usb_data[i].cons;
			VectPlan v = (cons - old_cons) / CONTROL_DT;
			pt[i-1].y = v.theta*1000.0f;
			old_cons = cons;
		}
		graphPointObject.update((float*)pt, robotItf->control_usb_data_count-1);
		graphPointObject.render(GL_POINTS);
	}

	if( graph->courbes_activated[SUBGRAPH_MOTION_SPEED_DIST_MES] )
	{
		shader.setColor3f(&graph->color[3*SUBGRAPH_MOTION_SPEED_DIST_MES]);
		VectPlan old_pos = robotItf->control_usb_data[0].pos;
		for(int i=1; i < robotItf->control_usb_data_count; i++)
		{
			VectPlan pos = robotItf->control_usb_data[i].pos;
			VectPlan v = (pos - old_pos) / CONTROL_DT;
			pt[i-1].y = v.norm();
			old_pos = pos;
		}
		graphPointObject.update((float*)pt, robotItf->control_usb_data_count-1);
		graphPointObject.render(GL_POINTS);
	}

	if( graph->courbes_activated[SUBGRAPH_MOTION_SPEED_ROT_MES] )
	{
		shader.setColor3f(&graph->color[3*SUBGRAPH_MOTION_SPEED_ROT_MES]);
		VectPlan old_pos = robotItf->control_usb_data[0].pos;
		for(int i=1; i < robotItf->control_usb_data_count; i++)
		{
			VectPlan pos = robotItf->control_usb_data[i].pos;
			VectPlan v = (pos - old_pos) / CONTROL_DT;
			pt[i-1].y = v.theta*1000.0f;
			old_pos = pos;
		}
		graphPointObject.update((float*)pt, robotItf->control_usb_data_count-1);
		graphPointObject.render(GL_POINTS);
	}

	for(int j = 0; j < 2; j++)
	{
		if( graph->courbes_activated[SUBGRAPH_MOTION_V1 + j] )
		{
			shader.setColor3f(&graph->color[3*(SUBGRAPH_MOTION_V1 + j)]);
			for(int i=0; i < robotItf->control_usb_data_count; i++)
			{
				pt[i-1].y = robotItf->control_usb_data[i].cons_motors_v[j];
			}
			graphPointObject.update((float*)pt, robotItf->control_usb_data_count-1);
			graphPointObject.render(GL_POINTS);
		}
		if( graph->courbes_activated[SUBGRAPH_MOTION_V1_MES + j] )
		{
			shader.setColor3f(&graph->color[3*(SUBGRAPH_MOTION_V1_MES + j)]);
			for(int i=0; i < robotItf->control_usb_data_count; i++)
			{
				pt[i-1].y = robotItf->control_usb_data[i].mes_motors[j].v;
			}
			graphPointObject.update((float*)pt, robotItf->control_usb_data_count-1);
			graphPointObject.render(GL_POINTS);
		}
	}

	if( graph->courbes_activated[SUBGRAPH_MOTION_VBAT] )
	{
		shader.setColor3f(&graph->color[3*SUBGRAPH_MOTION_VBAT]);
		for(int i=1; i < robotItf->control_usb_data_count; i++)
		{
			pt[i-1].y = robotItf->control_usb_data[i].vBat;
		}
		graphPointObject.update((float*)pt, robotItf->control_usb_data_count-1);
		graphPointObject.render(GL_POINTS);
	}

	for(int j = 0; j < 4; j++)
	{
		if( graph->courbes_activated[SUBGRAPH_MOTION_I1+j] )
		{
			shader.setColor3f(&graph->color[3*(SUBGRAPH_MOTION_I1+j)]);
			for(int i=1; i < robotItf->control_usb_data_count; i++)
			{
				pt[i-1].y = 1000 * robotItf->control_usb_data[i].iPwm[j];
			}
			graphPointObject.update((float*)pt, robotItf->control_usb_data_count-1);
			graphPointObject.render(GL_POINTS);
		}
	}
	shader.setSprite(0);
}

static void mouse_press(GtkWidget* widget, GdkEventButton* event)
{
	if(event->button == 1 )
	{
		if( current_graph == GRAPH_TABLE )
		{
#ifndef GTK3
			if(!gtk_gl_area_make_current (GTK_GL_AREA (widget)))
			{
				return;
			}
#else
			gtk_gl_area_make_current(GTK_GL_AREA (widget));
			if (gtk_gl_area_get_error (GTK_GL_AREA (widget)) )
			{
				return;
			}
#endif
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
	gtk_widget_queue_draw(widget);
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
		gtk_widget_queue_draw(widget);
	}
	else if(event->button == 2)
	{
		mouse_scroll_x1 = 0;
		mouse_scroll_y1 = 0;
	}
}

static void mouse_move(GtkWidget* widget, GdkEventMotion* event)
{
	if(event->state & GDK_BUTTON1_MASK)
	{
		mouse_x2 = event->x;
		mouse_y2 = event->y;
		if(current_graph == GRAPH_TABLE)
		{
			tableScene.mouseMoveSelection(event->x, event->y);
		}
		gtk_widget_queue_draw(widget);
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
			gtk_widget_queue_draw(widget);
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
	gtk_widget_queue_draw(widget);
}

static gboolean keyboard_press(GtkWidget* widget, GdkEventKey* event, gpointer arg)
{
	(void) widget;
	(void) arg;
	int res;

	switch(event->keyval)
	{
		case GDK_KEY_Escape:
			drawing_zoom_selection = 0;
			break;
		case GDK_KEY_KP_Subtract:
		case GDK_KEY_minus:
			graph[current_graph].zoomf(2);
			break;
		case GDK_KEY_KP_Add:
		case GDK_KEY_plus:
			graph[current_graph].zoomf(0.5);
			break;
		case GDK_KEY_Right:
			tableScene.translateView(40, 0, 0);
			break;
		case GDK_KEY_Left:
			tableScene.translateView(-40, 0, 0);
			break;
		case GDK_KEY_Up:
			tableScene.translateView(0, 0, -40);
			break;
		case GDK_KEY_Down:
			tableScene.translateView(0, 0, 40);
			break;
		case GDK_KEY_r:
			for(int i = 0; i < robotCount; i++)
			{
				res = pthread_mutex_lock(&robot[i].m_robotItf.mutex);
				if(res == 0)
				{
					robot[i].m_robotItf.control_usb_data_count = 0;
					pthread_mutex_unlock(&robot[i].m_robotItf.mutex);
				}
			}
			break;
		case GDK_KEY_u:
			graph[current_graph].reset_roi();
			break;
	}

	gtk_widget_queue_draw(opengl_window);

	return TRUE;
}

static gboolean keyboard_release(GtkWidget* widget, GdkEventKey* event, gpointer arg)
{
	(void) widget;
	(void) event;
	(void) arg;
	return TRUE;
}

static void toggle_color(GtkWidget* widget, gpointer /*arg*/)
{
	printf("toggle color!!\n");

#ifndef GTK3
	if(!gtk_gl_area_make_current (GTK_GL_AREA (opengl_window)))
	{
		return;
	}
#else
	gtk_gl_area_make_current(GTK_GL_AREA(opengl_window));
	if (gtk_gl_area_get_error( GTK_GL_AREA(opengl_window) ) )
	{
		return;
	}
#endif

	if( color == COLOR_GREEN )
	{
		color = COLOR_PURPLE;
	}
	else
	{
		color = COLOR_GREEN;
	}

	tableScene.setColor(color);

	ioColor = !ioColor;
	for(int i = 0; i < robotCount; i++)
	{
		if(robot[i].m_simulation)
		{
			// simulation : on change l'io
			if( robot[i].m_robotItf.last_control_usb_data.pos.x == 0 && robot[i].m_robotItf.last_control_usb_data.pos.y == 0 && ! robot[i].m_robotItf.get_gpio(1 << (GPIO_IN_GO+1)))
			{
				robot[i].m_qemu.setPosition(qemuStartPos.symetric(color));
			}
			robot[i].m_qemu.setIo(GPIO_MASK(IO_COLOR), ioColor);
		}
		else
		{
			// en reel, on passe par l'interface de com
			robot[i].m_robotItf.color(color);
		}
	}

	GtkColorButton* switchColorBtn = (GtkColorButton*) widget;

	if( ioColor )
	{
#ifdef GTK3
		GdkRGBA green = {0, 1, 0, 1};
		gtk_color_chooser_set_rgba((GtkColorChooser*)switchColorBtn, &green);
#else
		GdkColor green = {0, 0, 65535, 0};
		gtk_color_button_set_color(switchColorBtn, &green);
#endif
	}
	else
	{
#ifdef GTK3
		GdkRGBA purple = {1, 0, 1, 1};
		gtk_color_chooser_set_rgba((GtkColorChooser*)switchColorBtn, &purple);
#else
		GdkColor purple = {0, 65535, 0, 65535};
		gtk_color_button_set_color(switchColorBtn, &purple);
#endif
	}
}

static void toggle_go(GtkWidget* /*widget*/, gpointer /*arg*/)
{
	static bool go = false;
	go = !go;

	for(int i = 0; i < robotCount; i++)
	{
		if(robot[i].m_simulation)
		{
			// simulation
			robot[i].m_qemu.setIo(GPIO_MASK_IN_GO, go);
		}
		else
		{
			robot[i].m_robotItf.go();
		}
	}
}

static void reboot_robot(GtkWidget* /*widget*/, gpointer /*arg*/)
{
#ifndef GTK3
	if(!gtk_gl_area_make_current (GTK_GL_AREA (opengl_window)))
	{
		return;
	}
#else
	gtk_gl_area_make_current(GTK_GL_AREA(opengl_window));
	if (gtk_gl_area_get_error( GTK_GL_AREA(opengl_window) ) )
	{
		return;
	}
#endif

	for(int i = 0; i < robotCount; i++)
	{
		if(robot[i].m_simulation)
		{
			// simulation
			robot[i].m_robotItf.current_time = 0;
			robot[i].m_robotItf.start_time = 0;
			robot[i].m_robotItf.control_usb_data_count = 0;
			robot[i].m_qemu.reboot();
		}
		else
		{
			robot[i].m_robotItf.reboot();
		}
	}
	qemu_set_parameters();
	tableScene.setColor(color);
}

static void joystick_event(int event, float /*val*/)
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
				//robotItf->motion_set_speed(VectPlan(0,0,1), val);
				break;
			case 2: // gachette gauche (xbox)
				//robotItf->motion_set_speed(VectPlan(1,0,0), val*1000);
				break;
			case 5: // gachette droite (xbox)
				//robotItf->motion_set_speed(VectPlan(0,1,0), val*1000);
				break;
			default:
				break;
		}
		//log_info("axe %d : %f", event, val);
	}
}
