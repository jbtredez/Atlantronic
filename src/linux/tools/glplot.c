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
#define MAX_FPS    20

static GLuint font_base;
static char font_name[] = "fixed";
static int font_height = 0;
static int font_digit_height = 0;
static int font_width = 0;
static XFontStruct* font_info = NULL;
static int screen_width = 0;
static int screen_height = 0;
static struct foo_interface foo;
static float bordure_pixel_x = 0;
static float bordure_pixel_y = 0;
static float mouse_x1 = 0;
static float mouse_y1 = 0;
static float mouse_x2 = 0;
static float mouse_y2 = 0;

enum
{
	GRAPH_TABLE = 0,
	GRAPH_NUM,
};

struct graph graph[GRAPH_NUM];

static void close_gtk(GtkWidget* widget, gpointer arg);
static void init(GtkWidget* widget, gpointer arg);
static gboolean config(GtkWidget* widget, GdkEventConfigure* ev, gpointer arg);
static gboolean afficher(GtkWidget* widget, GdkEventExpose* ev, gpointer arg);
static void mounse_press(GtkWidget* widget, GdkEventButton* event);
static void mounse_release(GtkWidget* widget, GdkEventButton* event);
static gboolean keyboard_press(GtkWidget* widget, GdkEventKey* event, gpointer arg);
static gboolean keyboard_release(GtkWidget* widget, GdkEventKey* event, gpointer arg);
static void mouse_move(GtkWidget* widget, GdkEventMotion* event);
static int init_font(GLuint base, char* f);
static void glPrintf(float x, float y, GLuint base, char* s, ...) __attribute__(( format(printf, 4, 5) ));
static void draw_plus(float x, float y, float rx, float ry);
static void glPrintf_xright2_ycenter(float x, float y, float x_ratio, float y_ratio, GLuint base, char* s, ...) __attribute__(( format(printf, 6, 7) ));
static void glPrintf_xcenter_yhigh2(float x, float y, float x_ratio, float y_ratio, GLuint base, char* s, ...) __attribute__(( format(printf, 6, 7) ));
static void glprint(float x, float y, GLuint base, char* buffer, int size);
void read_callback();

int main(int argc, char *argv[])
{
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
	GtkWidget* opengl_window = gtk_drawing_area_new();
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

	menuObj = gtk_menu_item_new_with_label("Quitter");
	g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(close_gtk), (GtkWidget*) main_window);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

	menuObj = gtk_menu_item_new_with_label("Fichier");
	gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);

	// ajout du menu1 que l'on vient de créer dans le menu0
	gtk_menu_shell_append(GTK_MENU_SHELL(menu0),menuObj);

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

static void init(GtkWidget* widget, gpointer arg)
{
	(void) arg;

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

	graph_init(&graph[GRAPH_TABLE], -1500, 1500, -1000, 1000);

	bordure_pixel_x = 10 * font_width;
	bordure_pixel_y = font_height*3;

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

void plot_table()
{
	float roi_xmin = graph[GRAPH_TABLE].roi_xmin;
	float roi_xmax = graph[GRAPH_TABLE].roi_xmax;
	float roi_ymin = graph[GRAPH_TABLE].roi_ymin;
	float roi_ymax = graph[GRAPH_TABLE].roi_ymax;

	float ratio_x = (roi_xmax - roi_xmin) / (screen_width - 2 * bordure_pixel_x);
	float ratio_y = (roi_ymax - roi_ymin) / (screen_height - 2 * bordure_pixel_y);

	float bordure_x = bordure_pixel_x * ratio_x;
	float bordure_y = bordure_pixel_y * ratio_y;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(roi_xmin - bordure_x, roi_xmax + bordure_x, roi_ymin - bordure_y, roi_ymax + bordure_y, 0, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glColor3f(0,0,0);

	glBegin(GL_LINE_STRIP);
	glVertex2f(roi_xmin, roi_ymin);
	glVertex2f(roi_xmax, roi_ymin);
	glVertex2f(roi_xmax, roi_ymax);
	glVertex2f(roi_xmin, roi_ymax);
	glVertex2f(roi_xmin, roi_ymin);
	glEnd();

	// axe x
	float dx = graph[GRAPH_TABLE].tics_dx;
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
	float dy = graph[GRAPH_TABLE].tics_dy;
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

	struct vect_pos pos_hokuyo = {0, 0, 0, 1, 0};
	struct vect_pos pos_table = {0, 0, 0, 1, 0};

	int i;
	glColor3f(1,0,0);
	for(i=0; i < 682; i++)
	{
		pos_hokuyo.x = foo.hokuyo_x[i];
		pos_hokuyo.y = foo.hokuyo_y[i];
		pos_hokuyo_to_table(&foo.hokuyo_scan.pos, &pos_hokuyo, &pos_table);
		draw_plus(pos_table.x, pos_table.y, 0.5*font_width*ratio_x, 0.5*font_width*ratio_y);
	}

	glColor3f(0,0,1);
	int max = foo.control_usb_data_count % CONTROL_USB_DATA_MAX;
	for(i=0; i< max; i++)
	{
		if(foo.control_usb_data[i].control_state != CONTROL_READY_ASSER && foo.control_usb_data[i].control_state != CONTROL_READY_FREE)
		{
			draw_plus(foo.control_usb_data[i].control_cons_x, foo.control_usb_data[i].control_cons_y, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
		}
	}

	glColor3f(0,1,0);
	for(i=0; i < max; i++)
	{
		draw_plus(foo.control_usb_data[i].control_pos_x, foo.control_usb_data[i].control_pos_y, 0.25*font_width*ratio_x, 0.25*font_width*ratio_y);
	}

	// affichage du repère robot
	glColor3f(0,0,0);
	struct vect_pos pos_robot;
	pos_robot.x = foo.control_usb_data[max-1].control_pos_x;
	pos_robot.y = foo.control_usb_data[max-1].control_pos_y;
	pos_robot.alpha = foo.control_usb_data[max-1].control_pos_alpha;
	pos_robot.ca = cos(pos_robot.alpha);
	pos_robot.sa = sin(pos_robot.alpha);

	glRotatef(pos_robot.alpha * 180 / M_PI, 0, 0, 1);
	glBegin(GL_LINES);
	glVertex2f(pos_robot.x, pos_robot.y);
	glVertex2f(pos_robot.x + 5 * font_height, pos_robot.y);
	glVertex2f(pos_robot.x, pos_robot.y);
	glVertex2f(pos_robot.x, pos_robot.y + 5 * font_height);
	glEnd();

	glBegin(GL_LINE_STRIP);
	glVertex2f(PARAM_NP_X, -150);
	glVertex2f(PARAM_NP_X, 150);
	glVertex2f(300 + PARAM_NP_X, 150);
	glVertex2f(300 + PARAM_NP_X, -150);
	glVertex2f(PARAM_NP_X, -150);
	glEnd();

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

	glBegin(GL_LINE_STRIP);
	glVertex2f(mouse_x1, mouse_y1);
	glVertex2f(mouse_x2, mouse_y1);
	glVertex2f(mouse_x2, mouse_y2);
	glVertex2f(mouse_x1, mouse_y2);
	glVertex2f(mouse_x1, mouse_y1);
	glEnd();

	int res = pthread_mutex_lock(&foo.mutex);
	if(res == 0)
	{
		plot_table();
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
		mouse_x1 = event->x;
		mouse_y1 = event->y;
		mouse_x2 = mouse_x1;
		mouse_y2 = mouse_y1;
	}
	else
	{
		graph_reset_roi(&graph[GRAPH_TABLE]);
	}
	gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
}

static void mounse_release(GtkWidget* widget, GdkEventButton* event)
{
	if(event->button == 1)
	{
		if( mouse_x1 != mouse_x2 && mouse_y1 != mouse_y2)
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

			float zx1 = (mouse_x1 - bordure_pixel_x) / (screen_width - 2 * bordure_pixel_x);
			float zx2 = (mouse_x2 - bordure_pixel_x) / (screen_width - 2 * bordure_pixel_x);
			float zy1 = (screen_height - mouse_y2 - bordure_pixel_y) / (screen_height - 2 * bordure_pixel_y);
			float zy2 = (screen_height - mouse_y1 - bordure_pixel_y) / (screen_height - 2 * bordure_pixel_y);
			graph_zoom(&graph[GRAPH_TABLE], zx1, zx2, zy1, zy2);
		}

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

static void glPrintf(float x, float y, GLuint base, char* s, ...)
{
	va_list arglist;
	va_start(arglist, s);
	char buffer[1024];
	int size = vsnprintf(buffer, sizeof(buffer), s, arglist);
	va_end(arglist);

	glprint(x, y, base, buffer, size);
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
