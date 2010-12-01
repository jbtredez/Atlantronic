#include <stdio.h>
#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <gdk/gdkkeysyms.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "vecteur.h"
#include "3ds.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

modele3DS table;
modele3DS tomate;

//! @struct Vue
//! @brief structure définissant une vue
typedef struct
{
	const char* nom; //!< nom du menu
	vecteur3f position; //!< position de la vue
	vecteur3f orientation; //!< orientation de la vue
}Vue;

//!
//! il suffit d'ajouter une ligne { nomDeLaVue, position, orientation } pour ajouter une vue
Vue vues[ ] = {
	{"Vue d'origine", vecteur3f(0,-2775,2400), vecteur3f(90,-31,0)},
	{"Vue de dessus", vecteur3f(0,0,5000),vecteur3f(90,-89.99,0)}
}; //!< vues prédéfinies

Vue vueCam = vues[0]; //!< vue de la camera

int xold = 0;//!< dernière position de la souris (bouton appuyé) suivant l'axe x
int yold = 0; //!< dernière position de la souris (bouton appuyé) suivant l'axe y

static gboolean afficher(GtkWidget* widget, GdkEventExpose* event, gpointer data); //!< fonction d'affichage openGL
static gboolean config(GtkWidget* widget, GdkEventConfigure* event, gpointer data); //!< s'occupe de reconfigurer openGL en cas de changement de taille de la fenêtre
static void init(GtkWidget* widget, gpointer data); //!< s'occupe de l'initialisation d'openGL
static gboolean clavierPress(GtkWidget* widget, GdkEventKey* event, gpointer data); //!< s'occupe de la gestion du clavier "touche pressée"
static gboolean clavierRelease(GtkWidget* widget, GdkEventKey* event, gpointer data); //!< s'occupe de la gestion du clavier "touche relachée"
static void sourisClic(GtkWidget* widget, GdkEventButton* event); //!< s'occupe de la gestion des clic de souris
static void sourisBouge(GtkWidget* widget, GdkEventMotion* event); //!< s'occupe de la gestion du mouvement de la souris

static void aPropos(GtkWidget* widget, gpointer data); //!< affiche et s'occupe de la fenêtre "a propos"
static void infoCG(GtkWidget* widget, gpointer data); //!< affiche et s'occupe de la fenêtre "information carte graphique"

static int fd_to_qemu;
static int fd_to_simu;

// tests
#if 0
#include <pthread.h>
static pthread_t id;

#define ACK               1
#define WRITE_MEMORY      2
#define READ_MEMORY       3

struct atlantronic_memory_io
{
	uint8_t cmd;
	uint64_t offset;
	uint32_t val;
};


void* lecture(void* arg)
{
	struct atlantronic_memory_io io;
	int n;
	uint8_t ack = ACK;

	while(1)
	{
		n = read(fd_to_simu, &io, sizeof(io) );
		if( n == sizeof(io))
		{
			if(io.cmd == WRITE_MEMORY)
			{
				write(fd_to_qemu, &ack, sizeof(ack));
				printf("commande : %i, offset : %#.4x val : %#.2x\n", io.cmd, io.offset, io.val);
			}
			else if(io.cmd == READ_MEMORY)
			{
				io.val = 10;
				write(fd_to_qemu, &io, sizeof(io));
			}
			else
			{
				printf("erreur protocole\n");
				return NULL;
			}
		}
		else if(n < 0)
		{
			perror("read");
			return NULL;
		}
		else
		{
			printf("erreur protocole\n");
			return NULL;
		}
	}

	return NULL;
}
#endif
int main(int argc, char** argv)
{
#if 0
	mkfifo("/tmp/to_qemu", 0666);
	mkfifo("/tmp/to_simu", 0666);

	fd_to_qemu = open("/tmp/to_qemu", O_WRONLY);
	fd_to_simu = open("/tmp/to_simu", O_RDONLY);

	pthread_create(&id, NULL, lecture, NULL);
#endif
	if( ! g_thread_supported() )
	{
		g_thread_init(NULL);
	}
	else
	{
//		meslog(_erreur_, "pas de support des g_thread, risque de bug (non prévu et non testé)");
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
		// meslog(_info_,"double buffer non géré");
		glconfig = gdk_gl_config_new_by_mode((GdkGLConfigMode)(GDK_GL_MODE_RGB | GDK_GL_MODE_DEPTH));
		if(glconfig == NULL)
		{
			// meslog(_erreur_,"opengl non géré, abandon");
			return 0;
		}
	}

	table.chargement("table.3ds");
	tomate.chargement("tomate.3ds");
	tomate.translation(0,0,100);

	// création de la fenêtre
	GtkWidget* fenetrePrincipale = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title(GTK_WINDOW(fenetrePrincipale),"Interface de Hercule");
	gtk_window_set_default_size(GTK_WINDOW(fenetrePrincipale), 400, 300);

	g_signal_connect(G_OBJECT(fenetrePrincipale), "delete_event", G_CALLBACK(gtk_main_quit), NULL);
	gtk_signal_connect(GTK_OBJECT(fenetrePrincipale), "destroy", GTK_SIGNAL_FUNC(gtk_main_quit), NULL);

	// objet que l'on met dans la fenetre principale dans lequel on mettra le menu et la hbox
	GtkWidget* vbox = gtk_vbox_new(FALSE, 0);
	gtk_container_add(GTK_CONTAINER(fenetrePrincipale), vbox);

	GtkWidget* hbox = gtk_hbox_new(FALSE, 0);

	// fenetre opengl
	GtkWidget* fenetreOpenGL = gtk_drawing_area_new();
	gtk_widget_set_size_request(fenetreOpenGL, 400, 300);
	gtk_widget_set_gl_capability(fenetreOpenGL, glconfig, NULL, TRUE, GDK_GL_RGBA_TYPE);

	gtk_widget_add_events(fenetreOpenGL, GDK_VISIBILITY_NOTIFY_MASK | GDK_BUTTON_PRESS_MASK | GDK_POINTER_MOTION_MASK);
	g_signal_connect_after(G_OBJECT(fenetreOpenGL), "realize", G_CALLBACK(init), NULL);
	g_signal_connect(G_OBJECT(fenetreOpenGL), "configure_event", G_CALLBACK(config), NULL);
	g_signal_connect(G_OBJECT(fenetreOpenGL), "expose_event", G_CALLBACK(afficher), NULL);
	g_signal_connect(G_OBJECT(fenetreOpenGL), "button_press_event", G_CALLBACK(sourisClic), NULL);
	g_signal_connect(G_OBJECT(fenetreOpenGL), "motion_notify_event", G_CALLBACK(sourisBouge), NULL);
	g_signal_connect_swapped(G_OBJECT(fenetrePrincipale), "key_press_event", G_CALLBACK(clavierPress), fenetreOpenGL);
	g_signal_connect_swapped(G_OBJECT(fenetrePrincipale), "key_release_event", G_CALLBACK(clavierRelease), fenetreOpenGL);

	// création du menu
	GtkWidget* menu0 = gtk_menu_bar_new();	// menu "niveau 0" (ie: barre de menu)
	GtkWidget* menu1 = gtk_menu_new();	// menu "niveau 1"
    GtkWidget* menuObj;

	// contenu du menu1 et def du nom à la fin
	menuObj = gtk_menu_item_new_with_label("Connexion localhost:4000\t1");
//    g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(connexionLocaleDef), (GtkWidget*) fenetreOpenGL);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

	menuObj = gtk_menu_item_new_with_label("Connexion 192.168.1.2:4000\t2");
//    g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(connexionRobotDef), (GtkWidget*) fenetreOpenGL);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

	menuObj = gtk_menu_item_new_with_label("Connexion...");
//    g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(connexion), (GtkWidget*) fenetreOpenGL);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

	menuObj = gtk_menu_item_new_with_label("Deconnexion");
//    g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(deconnexion), (GtkWidget*) fenetreOpenGL);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

	menuObj = gtk_menu_item_new_with_label("Quitter");
//    g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(fermetureGTK), (GtkWidget*) fenetrePrincipale);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

	menuObj = gtk_menu_item_new_with_label("Controle");
	gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);

	// ajout du menu1 que l'on vient de créer dans le menu0
	gtk_menu_shell_append(GTK_MENU_SHELL(menu0),menuObj);

	// nouveau menu de "niveau 1"
	menu1 = gtk_menu_new();

	// contenu du menu1 et def du nom à la fin

	menuObj = gtk_menu_item_new_with_label("Constantes du robot");
//    g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(modifierConstantes), (GtkWidget*) fenetrePrincipale);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

	menuObj = gtk_menu_item_new_with_label("Calibration de la camera du robot");
//    g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(calibrationCamera), (GtkWidget*) fenetrePrincipale);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

	menuObj = gtk_menu_item_new_with_label("Paramétrage");
	gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);

	// ajout du menu1 que l'on vient de créer dans le menu0
	gtk_menu_shell_append(GTK_MENU_SHELL(menu0),menuObj);

	// nouveau menu de "niveau 1"
	menu1 = gtk_menu_new();

	for(unsigned int i=0;i<(sizeof(vues)/sizeof(Vue));i++){
		menuObj = gtk_menu_item_new_with_label(vues[i].nom);
//		g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(menuVue), fenetreOpenGL);
		gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);
	}

	menuObj = gtk_menu_item_new_with_label("Vues");
	gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);

	// ajout du menu1 que l'on vient de créer dans le menu0
	gtk_menu_shell_append(GTK_MENU_SHELL(menu0),menuObj);

	// nouveau menu de "niveau 1"
	menu1 = gtk_menu_new();
	menuObj = gtk_menu_item_new_with_label("Infos carte graphique");
    g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(infoCG), (GtkWidget*) fenetrePrincipale);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

	menuObj = gtk_menu_item_new_with_label("A propos");
    g_signal_connect(G_OBJECT(menuObj), "activate", G_CALLBACK(aPropos), (GtkWidget*) fenetrePrincipale);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu1), menuObj);

	menuObj = gtk_menu_item_new_with_label("Aide");
	gtk_menu_item_set_submenu(GTK_MENU_ITEM(menuObj), menu1);

	gtk_menu_shell_append(GTK_MENU_SHELL(menu0),menuObj);

	GtkWidget* vbox2 = gtk_vbox_new(FALSE, 0);

	// rangement des éléments dans la fenetre
    gtk_box_pack_start(GTK_BOX(vbox), menu0, FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox), fenetreOpenGL, TRUE, TRUE, 0);
	gtk_box_pack_start(GTK_BOX(hbox), vbox2, FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(vbox), hbox, TRUE, TRUE, 0);

	gtk_widget_show_all(fenetrePrincipale);

	gtk_main();
	gdk_threads_leave();

/*
	char c;
	printf("je suis lancé\n");
	c = getchar();
	printf("avec %c\n", c);
*/
	return 0;
}

static void init(GtkWidget* widget, gpointer )
{
	GdkGLContext* glcontext = gtk_widget_get_gl_context(widget);
	GdkGLDrawable* gldrawable = gtk_widget_get_gl_drawable(widget);

	if(!gdk_gl_drawable_gl_begin(gldrawable, glcontext))
	{
		return;
	}

	table.compiler();
	tomate.compiler();

//	glGenTextures(1,&idTexture);
	glEnable(GL_TEXTURE_2D);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glEnable(GL_ALPHA_TEST);
	glEnable (GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glPolygonMode(GL_BACK,GL_LINE);
	glPolygonMode(GL_FRONT,GL_FILL);

	float posLum0[4] = {0,0,3000,0};
	glLightfv(GL_LIGHT0,GL_POSITION,posLum0);
	glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION,  1.0f);
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.2f);
    glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.08f);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_CULL_FACE);

	gdk_gl_drawable_gl_end(gldrawable);
}

static gboolean afficher(GtkWidget* widget, GdkEventExpose* , gpointer )
{
	GdkGLContext* glcontext = gtk_widget_get_gl_context(widget);
	GdkGLDrawable* gldrawable = gtk_widget_get_gl_drawable(widget);

	if(!gdk_gl_drawable_gl_begin(gldrawable, glcontext)) return false;

	glClearColor(0,0,0,1.0); 		// selectionne la couleur noire (qui est celle par défaut)
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 	// efface le frame buffer

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(vueCam.position.x,vueCam.position.y,vueCam.position.z,
			vueCam.position.x + cos(M_PI/180*vueCam.orientation.x)*cos(M_PI/180*vueCam.orientation.y),
			vueCam.position.y+sin(M_PI/180*vueCam.orientation.x)*cos(M_PI/180*vueCam.orientation.y),
			vueCam.position.z+sin(M_PI/180*vueCam.orientation.y),
			0,0,1);

	table.afficher();
	tomate.afficher();

	if(gdk_gl_drawable_is_double_buffered(gldrawable))
	{
		gdk_gl_drawable_swap_buffers(gldrawable);
	}
	else
	{
		glFlush();
	}

	gdk_gl_drawable_gl_end(gldrawable);

	return true;
}

static gboolean config(GtkWidget* widget, GdkEventConfigure* , gpointer )
{
	GdkGLContext* glcontext = gtk_widget_get_gl_context(widget);
	GdkGLDrawable* gldrawable = gtk_widget_get_gl_drawable(widget);

	GLfloat ratio = (GLfloat) (widget->allocation.width) / (GLfloat) (widget->allocation.height);

	if(!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
	{
		return false;
	}

	glViewport(0, 0, widget->allocation.width, widget->allocation.height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0,(float) ratio,100,100000);

//	glFrustum (-1.0, 1.0, -1/ratio, 1/ratio, 5.0, 60.0);

	gdk_gl_drawable_gl_end(gldrawable);

	return true;
}

static gboolean clavierPress(GtkWidget* widget, GdkEventKey* event, gpointer data)
{
	static float dl = 50;
	switch(event->keyval)
	{
		case GDK_z:
			vueCam.position.x += dl*cos(M_PI/180*vueCam.orientation.x)*cos(M_PI/180*vueCam.orientation.y);
			vueCam.position.y += dl*sin(M_PI/180*vueCam.orientation.x)*cos(M_PI/180*vueCam.orientation.y);
			vueCam.position.z += dl*sin(M_PI/180*vueCam.orientation.y);
			break;
		case GDK_s:
			vueCam.position.x -= dl*cos(M_PI/180*vueCam.orientation.x)*cos(M_PI/180*vueCam.orientation.y);
			vueCam.position.y -= dl*sin(M_PI/180*vueCam.orientation.x)*cos(M_PI/180*vueCam.orientation.y);
			vueCam.position.z -= dl*sin(M_PI/180*vueCam.orientation.y);
			break;
		case GDK_d:
			vueCam.position.x += dl*sin(M_PI/180*vueCam.orientation.x)*cos(M_PI/180*vueCam.orientation.y);
			vueCam.position.y -= dl*cos(M_PI/180*vueCam.orientation.x)*cos(M_PI/180*vueCam.orientation.y);
			break;
		case GDK_q:
			vueCam.position.x -= dl*sin(M_PI/180*vueCam.orientation.x)*cos(M_PI/180*vueCam.orientation.y);
			vueCam.position.y += dl*cos(M_PI/180*vueCam.orientation.x)*cos(M_PI/180*vueCam.orientation.y);
			break;
		case GDK_Escape:
			gtk_main_quit();
			return true;	// pour ne pas appeler gdk_window_invalidate_rect après destruction
			break;
		default:
			break;
	}

	gdk_window_invalidate_rect(widget->window, &widget->allocation, false);

	return true;
}

static gboolean clavierRelease(GtkWidget* widget, GdkEventKey* event, gpointer data)
{
	switch(event->keyval)
	{
		case GDK_Up:
			tomate.translation(0,100,0);
			break;
		case GDK_Down:
			tomate.translation(0,-100,0);
			break;
		case GDK_Left:
			tomate.translation(-100,0,0);
			break;
		case GDK_Right:
			tomate.translation(100,0,0);
			break;
		default:
			break;
	}

	gdk_window_invalidate_rect(widget->window, &widget->allocation, false);

	return true;
}

static void sourisClic(GtkWidget* , GdkEventButton* event)
{
	if(event->button == 1){
		xold = event->x;
		yold = event->y;
	}
}

static void sourisBouge(GtkWidget* widget, GdkEventMotion* event)
{
	if(event->state & GDK_BUTTON1_MASK){
		vueCam.orientation.x -= 0.1 * (event->x - xold);
		if(vueCam.orientation.x >= 360)	vueCam.orientation.x -= 360;
		if(vueCam.orientation.x < 0) vueCam.orientation.x += 360;
		vueCam.orientation.y -= 0.1 * (event->y - yold);
		if(vueCam.orientation.y >= 90 - 0.0001) vueCam.orientation.y = 90 - 0.0001;
		if(vueCam.orientation.y < -90 + 0.0001) vueCam.orientation.y = -90 + 0.0001;

		yold = event->y;
		xold = event->x;

		gdk_window_invalidate_rect(widget->window, &widget->allocation, false);
	}
}

void infoCG(GtkWidget* , gpointer data)
{
	GtkWidget* fenetre = gtk_message_dialog_new(GTK_WINDOW(data),
		GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_OK,
		"Informations sur le rendu opengl:\n"
		"\n"
		"GL_RENDERER   = %s\n"
		"GL_VERSION    = %s\n"
		"GL_VENDOR     = %s\n"
//		"GL_EXTENSIONS = %s\n"
		,glGetString(GL_RENDERER), glGetString(GL_VERSION),
		glGetString(GL_VENDOR)
//,	glGetString(GL_EXTENSIONS)
		);

	gtk_window_set_title(GTK_WINDOW(fenetre),"Infos carte graphique");

	gtk_dialog_run(GTK_DIALOG(fenetre));
	gtk_widget_destroy(fenetre);
}

void aPropos(GtkWidget* , gpointer data)
{
	GtkWidget* fenetre = gtk_message_dialog_new(GTK_WINDOW(data),
		GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_OK,
		"Interface de simulation du robot\n"
		"\n"
		"Réalisée par Atlantronic"
		);

	gtk_window_set_title(GTK_WINDOW(fenetre),"A propos");

	gtk_dialog_run(GTK_DIALOG(fenetre));
	gtk_widget_destroy(fenetre);
}

