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
#include <linux/kthread.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
#error pas teste avec une version < 2.6.39
#endif

MODULE_AUTHOR("Atlantronic"); //!< auteur du module (visible par modinfo)
MODULE_DESCRIPTION("Module USB pour communiquer avec les cartes électroniques du robot"); //!< description du module (visible par modinfo)
MODULE_SUPPORTED_DEVICE("Cartes usb du robot (Foo, Bar)"); //!< appareils supportés (visible par modinfo)
MODULE_LICENSE("GPL");		//!< licence "GPL"

#define DEBUG_USB                        3

// valeurs de l'id du périphérique
#define ATLANTRONIC_ID              0x1818   //!< id du vendeur
#define ATLANTRONIC_FOO_ID          0x0001   //!< id de foo
#define ATLANTRONIC_BAR_ID          0x0002   //!< id de bar

#define ATLANTRONIC_LOG_SUBCLASS      0x00   //!< interface de log
#define ATLANTRONIC_HOKUYO_SUBCLASS   0x01   //!< interface hokuyo

#define ATLANTRONIC_MAX_PRODUCT_NAME_SIZE       0x10
#define ATLANTRONIC_MAX_INTERFACE_NAME_SIZE     0x10

#define ATLANTRONIC_LOG_BUFFER_SIZE             4096

#define ATLANTRONIC_MAX_IN_URB                    10

#undef err
#define err(format, arg...) printk(KERN_ERR KBUILD_MODNAME ":%s:%d: " format "\n" , __FUNCTION__, __LINE__, ## arg)	//!< macro d'erreur formatée
#define info(format, arg...) printk(KERN_INFO KBUILD_MODNAME ": " format "\n" , ## arg) //!< macro d'info formatée

#define debug(niveau, format, arg...) do	\
{	\
	if( (niveau) <= DEBUG_USB )		\
		printk(KERN_INFO KBUILD_MODNAME ":%s:%d: " format "\n" , __FUNCTION__, __LINE__, ## arg);	\
}while(0)	//!< macro de debug formatée

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

static int atlantronic_log_task(void* arg);

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
	struct usb_device*		  udev;                //!< udev
	struct usb_interface*	  interface;           //!< interface
	unsigned char             interface_subclass;  //!< type d'interface
	struct usb_class_driver   class;               //!< class driver
	struct task_struct*       log_task;            //!< tache de recuperation des log
	struct completion         in_completion;       //!< completion
	volatile char             thread_stop_req;     //!< request thread to stop
	spinlock_t                err_lock;            //!< lock sur error
	int                       error;               //!< erreur urb
	int                       open_count;          //!< nombre d'ouvertures
	struct mutex              io_mutex;            //!< mutex i/o
	wait_queue_head_t         wait;                //!< liste de processus en attente dans le read
	struct urb*   in_urb[ATLANTRONIC_MAX_IN_URB];            //!< urb
	unsigned char log_buffer[ATLANTRONIC_LOG_BUFFER_SIZE];   //!< log buffer
	int          log_buffer_begin; //!< indice de fin des log dans le buffer
	int          log_buffer_end;   //!< indice de fin des log dans le buffer
	struct kref  kref;             //!< compteur de references
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
	int i;

	debug(1, "atlantronic_log_delete");

	dev = container_of(kref, struct atlantronic_log_data, kref);

	usb_put_dev(dev->udev);

	for(i = 0; i < ATLANTRONIC_MAX_IN_URB; i++)
	{
		if( dev->in_urb[i] )
		{
			usb_kill_urb(dev->in_urb[i]);
			usb_free_urb(dev->in_urb[i]);
		}
	}

	if( dev->class.name )
	{
		kfree(dev->class.name);
	}

	kfree(dev);
}

static int atlantronic_log_open(struct inode *inode, struct file *file)
{
	struct atlantronic_log_data* dev;
	struct usb_interface* interface;
	int subminor;
	int rep = 0;

	debug(1, "atlantronic_log_open");

	subminor = iminor(inode);
	// récupération de l'interface liée au périphérique
	interface = usb_find_interface(&atlantronic_log_driver, subminor);
	if(!interface)
	{
		err("usb_find_interface error subminor=%d", subminor);
		rep = -ENODEV;
		goto error;
	}

	// récupération des infos liées à l'interface.
	dev = usb_get_intfdata(interface);
	if(!dev)
	{
		rep = -ENODEV;
		goto error;
	}

	// on met à jour le compteur de pointeur sur la structure
	kref_get(&dev->kref);

	mutex_lock(&dev->io_mutex);

	if(!dev->open_count++)
	{
		rep = usb_autopm_get_interface(interface);
		if(rep)
		{
			goto error_unlock;
		}
	}

	file->private_data = dev;

	mutex_unlock(&dev->io_mutex);

	return 0;

error_unlock:
	dev->open_count--;
	mutex_unlock(&dev->io_mutex);
	kref_put(&dev->kref, atlantronic_log_delete);
error:
	return rep;
}

static int atlantronic_log_release(struct inode *inode, struct file *file)
{
	struct atlantronic_log_data* dev = file->private_data;

	debug(1, "atlantronic_log_release");

	if(dev == NULL)
	{
		return -ENODEV;
	}

	mutex_lock(&dev->io_mutex);

	if (!--dev->open_count && dev->interface)
	{
		usb_autopm_put_interface(dev->interface);
	}
	mutex_unlock(&dev->io_mutex);

	kref_put(&dev->kref, atlantronic_log_delete);

	return 0;
}

static ssize_t atlantronic_log_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct atlantronic_log_data* dev = file->private_data;
	int size = 0;
	int rep;
	int n_cpy = 0;

	debug(1, "atlantronic_log_read");

	if(dev == NULL)
	{
		rep = -ENODEV;
		goto error;
	}

	do
	{
		if( wait_event_interruptible(dev->wait, dev->log_buffer_begin != dev->log_buffer_end))
		{
			debug(2, "interruption par un signal");
			rep = -ERESTARTSYS;
			goto error;
		}

		mutex_lock(&dev->io_mutex);

		size = dev->log_buffer_end - dev->log_buffer_begin;
		if(size < 0)
		{
			size = ATLANTRONIC_LOG_BUFFER_SIZE - dev->log_buffer_begin;
			if(size > count)
			{
				size = count;
			}

			if( copy_to_user(buffer, dev->log_buffer + dev->log_buffer_begin, size) )
			{
				err("copy_to_user failed (size = %d)", size);
				rep = -EFAULT;
				goto error_unlock;
			}

			n_cpy = size;
			dev->log_buffer_begin = (dev->log_buffer_begin + size) % ATLANTRONIC_LOG_BUFFER_SIZE;
		}

		size = dev->log_buffer_end - dev->log_buffer_begin;
		if(size > 0)
		{
			if(size > count - n_cpy)
			{
				size = count - n_cpy;
			}

			if( copy_to_user(buffer + n_cpy, dev->log_buffer + dev->log_buffer_begin, size) )
			{
				err("copy_to_user failed (size = %d)", size);
				rep = -EFAULT;
				goto error_unlock;
			}

			n_cpy += size;
			dev->log_buffer_begin = (dev->log_buffer_begin + size) % ATLANTRONIC_LOG_BUFFER_SIZE;
		}

		mutex_unlock(&dev->io_mutex);
	}while( n_cpy == 0);

	return n_cpy;

error_unlock:
	mutex_unlock(&dev->io_mutex);
error:
	return rep;
}

static void atlantronic_log_urb_callback(struct urb *urb)
{
	struct atlantronic_log_data* dev = urb->context;

	spin_lock(&dev->err_lock);
	switch(urb->status)
	{
		case 0:
			break;
		case -ETIMEDOUT:
			err("time out urb");
			break;
		case -ECONNRESET:
		case -ENOENT:
		case -ESHUTDOWN:
			err("arrêt urb: %d", urb->status);
			dev->error = urb->status;
			break;
		default:
			err("status urb non nul: %d", urb->status);
			break;
	}
	spin_unlock(&dev->err_lock);

	complete(&dev->in_completion);
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
	int i;

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
	init_completion(&dev->in_completion);
	mutex_init(&dev->io_mutex);
	init_waitqueue_head(&dev->wait);

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
	if( ! usb_endpoint_is_bulk_in(endpoint) )
	{
		err("wrong endpoint: subclass=%i endPointAddr=%i", dev->interface_subclass, endpoint->bEndpointAddress);
		rep = -ENODEV;
		goto error;		
	}

	info("log interface detected");
	dev->class.fops = &atlantronic_log_fops;
	strcpy(dev->class.name + len, "_log%d");

	dev->class.name[0] = tolower(dev->class.name[0]);

	dev->log_buffer_begin = 0;
	dev->log_buffer_end = 0;

	// allocation des urb
	for(i = 0; i < ATLANTRONIC_MAX_IN_URB; i++)
	{
		dev->in_urb[i] = usb_alloc_urb(0, GFP_KERNEL);

		if( ! dev->in_urb[i] )
		{
			err("Out of memory");
			goto error;
		}

		pipe = usb_rcvbulkpipe(dev->udev, endpoint->bEndpointAddress);
		usb_fill_bulk_urb(dev->in_urb[i], dev->udev, pipe, dev->log_buffer, le16_to_cpu(endpoint->wMaxPacketSize), atlantronic_log_urb_callback, dev);
	}

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

	dev->thread_stop_req = 0;
	dev->log_task = kthread_run(atlantronic_log_task, dev, "atlantronic_log%d", interface->minor);

	if( IS_ERR(dev->log_task) )
	{
		dev->log_task = NULL;
		goto error_deregister;	
	}

	for(i = 0; i<ATLANTRONIC_MAX_IN_URB ; i++)
	{
		rep = usb_submit_urb( dev->in_urb[i], GFP_ATOMIC);
		if( rep )
		{
			err("usb_submit_urb failed");
			goto error_deregister;
		}
	}

	info("interface attached to log%d", interface->minor);

	return 0;

error_deregister:
	usb_deregister_dev(interface, &dev->class);

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

    info("Atlantronic_log_disconnect");

	dev = usb_get_intfdata(interface);

	usb_deregister_dev(interface, &dev->class);

	dev->thread_stop_req = 1;

	complete(&dev->in_completion);
	if( dev->log_task)
	{
		wake_up_process(dev->log_task);
	}

	kref_put(&dev->kref, atlantronic_log_delete);
}

static int atlantronic_log_task(void* arg)
{
	struct atlantronic_log_data* dev = arg;
	struct urb* urb;
	int urb_id = 0;
	int rep;
	int error;
	int i;
	int len;

	kref_get(&dev->kref);

	while( 1 )
	{
		wait_for_completion_interruptible(&dev->in_completion);

		if( dev->thread_stop_req )
		{
			debug(1, "stop thread request");
			goto exit;
		}

		spin_lock_irq(&dev->err_lock);
		error = dev->error;
		spin_unlock_irq(&dev->err_lock);

		urb = dev->in_urb[urb_id];
		if(error)
		{
			len = 0;
		}
		else
		{
			len = urb->actual_length;
		}

#if DEBUG_USB >=3
		if(len)
		{
			printk(KERN_INFO "Log reçu (urb %i): %d octets : ", urb_id, len);
			for(i = 0; i< len; i++)
			{
				printk("%#.2x ", ((__u8*)urb->transfer_buffer)[i]);
			}
			printk("\n");
		}
#endif

		mutex_lock(&dev->io_mutex);
		for(i = 0; i< len; i++)
		{
			dev->log_buffer[dev->log_buffer_end] = ((__u8*)urb->transfer_buffer)[i];
			dev->log_buffer_end = (dev->log_buffer_end + 1) % ATLANTRONIC_LOG_BUFFER_SIZE;
			if( dev->log_buffer_end == dev->log_buffer_begin )
			{
				// buffer circulaire plein
				dev->log_buffer_begin = (dev->log_buffer_begin + 1) % ATLANTRONIC_LOG_BUFFER_SIZE;
			}
		}
		mutex_unlock(&dev->io_mutex);

		rep = usb_submit_urb(dev->in_urb[urb_id], GFP_ATOMIC);
		if( rep )
		{
			err("usb_submit_urb");
		}

		urb_id = (urb_id + 1) % ATLANTRONIC_MAX_IN_URB;
		wake_up_interruptible(&dev->wait);
	}

exit:
	debug(1, "stop log thread");
	kref_put(&dev->kref, atlantronic_log_delete);
	return 0;
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

