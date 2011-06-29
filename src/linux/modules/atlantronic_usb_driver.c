//! @file atlantronic_usb_driver.c
//! @brief Code du driver USB des cartes Atlantronic
//! @author Atlantronic

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/usb.h>
#include <linux/kref.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/slab.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
#error pas teste avec une version < 2.6.39
#endif

MODULE_AUTHOR("Atlantronic"); //!< auteur du module (visible par modinfo)
MODULE_DESCRIPTION("Module USB pour communiquer avec les cartes électroniques du robot"); //!< description du module (visible par modinfo)
MODULE_SUPPORTED_DEVICE("Cartes usb du robot (Foo, Bar)"); //!< appareils supportés (visible par modinfo)
MODULE_LICENSE("GPL");		//!< licence "GPL"

// valeurs de l'id du périphérique
#define ATLANTRONIC_ID              0x1818   //!< id du vendeur
#define ATLANTRONIC_FOO_ID          0x0001   //!< id de foo
#define ATLANTRONIC_BAR_ID          0x0002   //!< id de bar

#define ATLANTRONIC_LOG_SUBCLASS      0x00   //!< interface de log
#define ATLANTRONIC_HOKUYO_SUBCLASS   0x01   //!< interface hokuyo

#define info(format, arg...) printk(KERN_INFO KBUILD_MODNAME ": "	format "\n" , ## arg)	//!< macro d'info formatée

// prototype des fonctions utilisées
static int atlantronic_probe(struct usb_interface *interface, const struct usb_device_id *id);
static void atlantronic_disconnect(struct usb_interface *interface);
static int __init atlantronic_init(void); //!< fonction d'initialisation du module (appelée à l'insertion du module dans le noyau)
static void __exit atlantronic_exit(void); //!< fonction de fermeture appelée lors du retrait du module du noyau

//! @brief table des id des périphériques gérés par le pilote
static struct usb_device_id atlantronic_device_id [] =
{
	{ USB_DEVICE(ATLANTRONIC_ID, ATLANTRONIC_FOO_ID) },
	{ USB_DEVICE(ATLANTRONIC_ID, ATLANTRONIC_BAR_ID) },
	{ }
};

MODULE_DEVICE_TABLE(usb, atlantronic_device_id); //!< ajout de la table des périphériques gérés par le module

static struct usb_driver atlantronic_driver =
{
    .name = "Atlantronic",
    .id_table = atlantronic_device_id,
    .probe = atlantronic_probe,
    .disconnect = atlantronic_disconnect,
	.supports_autosuspend = 1,
};//!< choix des fonctions à appeler lors de la connexion ou de la déconnexion d'un périphérique usb

struct atlantronic_data
{
	struct usb_device*		udev; 					//!< udev
	struct usb_interface*	interface;				//!< interface
	unsigned char           interface_subclass;     //!< type d'interface
	struct kref				kref;					//!< compteur pour libérer la mémoire quand il le faut
};

static void atlantronic_delete(struct kref *kref)
{
	struct atlantronic_data* dev;

	dev = container_of(kref, struct atlantronic_data, kref);
	usb_put_dev(dev->udev);
	kfree(dev);
}

//! @param interface interface usb fournie par usbcore
//! @param id id du périphérique qui a été branché
//!
//! la fonction s'occupe de vérifier que l'on a connecté le bon périphérique (vérifie la présence des endpoints)
//! La fonction s'occupe allocation des ressources nécessaires pour la gestion du périphérique
static int atlantronic_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	int rep = -ENOMEM;
	struct atlantronic_data* dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;

    info("atlantronic_probe");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if( ! dev )
	{
		err("Out of memory");
		goto error;
	}

	kref_init(&dev->kref);
	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	iface_desc = interface->cur_altsetting;

	dev->interface_subclass = iface_desc->desc.bInterfaceSubClass;
	if( dev->interface_subclass == ATLANTRONIC_LOG_SUBCLASS && iface_desc->desc.bNumEndpoints == 1)
	{
		endpoint = &iface_desc->endpoint[0].desc;
		if( usb_endpoint_is_int_in(endpoint) )
		{
			info("log interface detected");
		}
		else
		{
			err("unknown endpoint: subclass=%i endPointAddr=%i", dev->interface_subclass, endpoint->bEndpointAddress);
		}
	}
	else if( dev->interface_subclass == ATLANTRONIC_HOKUYO_SUBCLASS && iface_desc->desc.bNumEndpoints == 1)
	{
		endpoint = &iface_desc->endpoint[0].desc;
		if( usb_endpoint_is_bulk_in(endpoint) )
		{
			info("hokuyo interface detected");
		}
		else
		{
			err("unknown endpoint: subclass=%i endPointAddr=%i", dev->interface_subclass, endpoint->bEndpointAddress);
		}
	}
	else
	{
		err("unknown interface: subclass=%i NumEndpoints=%i", dev->interface_subclass, iface_desc->desc.bNumEndpoints);
	}
	
	// sauvegarde du pointeur dans l'interface
	usb_set_intfdata(interface, dev);

	return 0;

error:
	if( dev )
	{
		kref_put(&dev->kref, atlantronic_delete);
	}

	return rep;
}

//! @param interface interface usb qui a été déconnectée et qui est gérée par ce module (ie: fonction robot_probe qui a retourné 0)
static void atlantronic_disconnect(struct usb_interface *interface)
{
	struct atlantronic_data* dev;

    info("atlantronic_disconnect");

	dev = usb_get_intfdata(interface);

	kref_put(&dev->kref, atlantronic_delete);
}

static int __init atlantronic_init(void)
{
	int rep = 0;

    info("Atlantronic : init");

	// enregistrement du pilote
	rep = usb_register(&atlantronic_driver);
	if(rep)
	{
		err("usb_register(): error %d", rep);
	}

    return rep;
}

static void __exit atlantronic_exit(void)
{
    info("Atlantronic : exit");
	usb_deregister(&atlantronic_driver);
}

// points d'entrées dans le noyau
module_init( atlantronic_init );
module_exit( atlantronic_exit );

