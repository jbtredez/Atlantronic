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
#include <linux/ctype.h>

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

#define ATLANTRONIC_MAX_PRODUCT_NAME_SIZE       0x10
#define ATLANTRONIC_MAX_INTERFACE_NAME_SIZE     0x10

#undef err
#define err(format, arg...) printk(KERN_ERR KBUILD_MODNAME ":%s:%d: " format "\n" , __FUNCTION__, __LINE__, ## arg)	//!< macro d'erreur formatée
#define info(format, arg...) printk(KERN_INFO KBUILD_MODNAME ": " format "\n" , ## arg) //!< macro d'info formatée

// prototype des fonctions utilisées
static int atlantronic_log_release(struct inode *inode, struct file *file);
static int atlantronic_log_open(struct inode *inode, struct file *file);
static ssize_t atlantronic_log_read(struct file *file, char *buffer, size_t count, loff_t *ppos);

static void atlantronic_log_urb_callback(struct urb *urb);

static int atlantronic_log_probe(struct usb_interface *interface, const struct usb_device_id *id);
static void atlantronic_log_disconnect(struct usb_interface *interface);

static int __init atlantronic_log_init(void); //!< fonction d'initialisation du module (appelée à l'insertion du module dans le noyau)
static void __exit atlantronic_log_exit(void); //!< fonction de fermeture appelée lors du retrait du module du noyau
static void atlantronic_log_delete(struct kref *kref);

//! @brief table des id des périphériques gérés par le pilote
static struct usb_device_id atlantronic_log_device_id [] =
{
	{ USB_DEVICE(ATLANTRONIC_ID, ATLANTRONIC_FOO_ID) },
	{ USB_DEVICE(ATLANTRONIC_ID, ATLANTRONIC_BAR_ID) },
	{ }
};

MODULE_DEVICE_TABLE(usb, atlantronic_log_device_id); //!< ajout de la table des périphériques gérés par le module

static struct usb_driver atlantronic_log_driver =
{
    .name = "Atlantronic",
    .id_table = atlantronic_log_device_id,
    .probe = atlantronic_log_probe,
    .disconnect = atlantronic_log_disconnect,
	.supports_autosuspend = 1,
};//!< choix des fonctions à appeler lors de la connexion ou de la déconnexion d'un périphérique usb

struct atlantronic_log_data
{
	struct usb_device*		udev;                        //!< udev
	struct usb_interface*	interface;                   //!< interface
	unsigned char           interface_subclass;          //!< type d'interface
	struct usb_class_driver class;                       //!< class driver
	struct urb*		        in_urb;                      //!< urb
	unsigned char           log_buffer[4096];            //!< log buffer
	int                     log_buffer_end;              //!< indice de fin des log dans le buffer
	struct kref				kref;                        //!< compteur pour libérer la mémoire quand il le faut
};

static const struct file_operations atlantronic_log_fops =
{
	.owner =	THIS_MODULE,
	.read =		atlantronic_log_read,
	.write =    NULL,
	.open =		atlantronic_log_open,
	.release =	atlantronic_log_release,
	.flush =	NULL,
	.llseek =	NULL,
};

static void atlantronic_log_delete(struct kref *kref)
{
	struct atlantronic_log_data* dev;

	dev = container_of(kref, struct atlantronic_log_data, kref);
	usb_put_dev(dev->udev);

	if( dev->in_urb )
	{
		usb_kill_urb(dev->in_urb);
		usb_free_urb(dev->in_urb);
	}

	if( dev->class.name )
	{
		kfree(dev->class.name);
	}

	kfree(dev);
}

static int atlantronic_log_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int atlantronic_log_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t atlantronic_log_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	return 0;
}

static void atlantronic_log_urb_callback(struct urb *urb)
{
	struct atlantronic_log_data* dev;
	int rep;
	int i;
	dev = urb->context;

	// TODO
	switch(urb->status)
	{
		case 0:
			break;
		case -ETIMEDOUT:
			err("time out urb");
			return;
			break;
		case -ECONNRESET:
		case -ENOENT:
		case -ESHUTDOWN:
			err("arrêt urb: %d", urb->status);
			return;
			break;
		default:
			err("status urb non nul: %d", urb->status);
			goto exit;	// on tente de renvoyer quand même
			break;
	}

//	#if DEBUG_USB >=3
	printk(KERN_INFO "Message reçu sur interrupt in: %d octets : ", urb->actual_length);
	for(i = 0;i< urb->actual_length;i++)
	{
		printk("%#.2x ",((__u8*)urb->transfer_buffer)[i]);
	}
	printk("\n");
//	#endif

exit:	
	rep = usb_submit_urb(urb, GFP_ATOMIC);

	if( rep )
	{
		err("usb_submit_urb");
	}
}

//! @param interface interface usb fournie par usbcore
//! @param id id du périphérique qui a été branché
//!
//! la fonction s'occupe de vérifier que l'on a connecté le bon périphérique (vérifie la présence des endpoints)
//! La fonction s'occupe allocation des ressources nécessaires pour la gestion du périphérique
static int atlantronic_log_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	int rep = -ENOMEM;
	struct atlantronic_log_data* dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int len;
	int pipe;

    info("atlantronic_log_probe");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if( ! dev )
	{
		err("Out of memory");
		goto error;
	}

	kref_init(&dev->kref);
	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	dev->class.name = kmalloc( ATLANTRONIC_MAX_PRODUCT_NAME_SIZE + ATLANTRONIC_MAX_INTERFACE_NAME_SIZE, GFP_KERNEL);
	if( ! dev->class.name )
	{
		err("Out of memory");
		goto error;
	}

	len = strlen(dev->udev->product);
	if( len > ATLANTRONIC_MAX_PRODUCT_NAME_SIZE )
	{
		len = ATLANTRONIC_MAX_PRODUCT_NAME_SIZE;
	}

	strncpy(dev->class.name, dev->udev->product, len);

	iface_desc = interface->cur_altsetting;

	dev->interface_subclass = iface_desc->desc.bInterfaceSubClass;
	if( dev->interface_subclass != ATLANTRONIC_LOG_SUBCLASS)
	{
		rep = -ENODEV;
		goto error;
	}

	if( iface_desc->desc.bNumEndpoints != 1)
	{
		err("unknown log interface: subclass=%i NumEndpoints=%i", dev->interface_subclass, iface_desc->desc.bNumEndpoints);
		rep = -ENODEV;
		goto error;	
	}

	endpoint = &iface_desc->endpoint[0].desc;
	if( ! usb_endpoint_is_int_in(endpoint) )
	{
		err("wrong endpoint: subclass=%i endPointAddr=%i", dev->interface_subclass, endpoint->bEndpointAddress);
		rep = -ENODEV;
		goto error;		
	}

	info("log interface detected");
	dev->class.fops = &atlantronic_log_fops;
	strcpy(dev->class.name + len, "_log%d");

	dev->class.name[0] = tolower(dev->class.name[0]);

	// allocation des urb
	dev->in_urb = usb_alloc_urb(0, GFP_KERNEL);

	if( ! dev->in_urb )
	{
		err("Out of memory");
		goto error;
	}

	pipe = usb_rcvintpipe(dev->udev, endpoint->bEndpointAddress);
	usb_fill_int_urb(dev->in_urb, dev->udev, pipe, dev->log_buffer, le16_to_cpu(endpoint->wMaxPacketSize), atlantronic_log_urb_callback, dev, endpoint->bInterval);

	// sauvegarde du pointeur dans l'interface
	usb_set_intfdata(interface, dev);

	// enregistrement auprès de usb_core
	rep = usb_register_dev(interface, &dev->class);

	if( rep )
	{
		// on n'a pas pu enregistrer le matériel
		err("usb_register_dev failed");
		goto error_intfdata;	
	}

	rep = usb_submit_urb( dev->in_urb, GFP_ATOMIC);
	if( rep )
	{
		err("usb_submit_urb failed");
		goto error;
	}

	info("interface attached to log%d", interface->minor);

	return 0;

error_intfdata:
	usb_set_intfdata(interface, NULL);

error:
	if( dev )
	{
		kref_put(&dev->kref, atlantronic_log_delete);
	}

	return rep;
}

//! @param interface interface usb qui a été déconnectée et qui est gérée par ce module (ie: fonction robot_probe qui a retourné 0)
static void atlantronic_log_disconnect(struct usb_interface *interface)
{
	struct atlantronic_log_data* dev;

    info("atlantronic_log_disconnect");

	dev = usb_get_intfdata(interface);

	usb_deregister_dev(interface, &dev->class);

	kref_put(&dev->kref, atlantronic_log_delete);
}

static int __init atlantronic_log_init(void)
{
	int rep = 0;

    info("Atlantronic : init");

	// enregistrement du pilote
	rep = usb_register(&atlantronic_log_driver);
	if(rep)
	{
		err("usb_register(): error %d", rep);
	}

    return rep;
}

static void __exit atlantronic_log_exit(void)
{
    info("Atlantronic : exit");
	usb_deregister(&atlantronic_log_driver);
}

// points d'entrées dans le noyau
module_init( atlantronic_log_init );
module_exit( atlantronic_log_exit );

