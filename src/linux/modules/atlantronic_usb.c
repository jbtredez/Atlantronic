//! @file atlantronic_usb.c
//! @brief Code du driver USB, interface des cartes Atlantronic
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

#define DEBUG_USB                        0

// valeurs de l'id du périphérique
#define ATLANTRONIC_ID              0x1818   //!< id du vendeur
#define ATLANTRONIC_FOO_ID          0x0001   //!< id de foo
#define ATLANTRONIC_BAR_ID          0x0002   //!< id de bar

#define ATLANTRONIC_SUBCLASS          0x00   //!< interface

#define ATLANTRONIC_MAX_PRODUCT_NAME_SIZE       0x40

#define ATLANTRONIC_BUFFER_SIZE              1024000

#define ATLANTRONIC_MAX_IN_URB                    10
#define ATLANTRONIC_MAX_OUT_URB                   10
#define ATLANTRONIC_MAX_WRITE_SIZE                64

#undef err
#define err(format, arg...) printk(KERN_ERR KBUILD_MODNAME ":%s:%d: " format "\n" , __FUNCTION__, __LINE__, ## arg)	//!< macro d'erreur formatée
#define info(format, arg...) printk(KERN_INFO KBUILD_MODNAME ": " format "\n" , ## arg) //!< macro d'info formatée

#define debug(niveau, format, arg...) do	\
{	\
	if( (niveau) <= DEBUG_USB )		\
		printk(KERN_INFO KBUILD_MODNAME ":%s:%d: " format "\n" , __FUNCTION__, __LINE__, ## arg);	\
}while(0)	//!< macro de debug formatée

// prototype des fonctions utilisées
static int atlantronic_release(struct inode *inode, struct file *file);
static int atlantronic_open(struct inode *inode, struct file *file);
static ssize_t atlantronic_read(struct file *file, char *buffer, size_t count, loff_t *ppos);
static ssize_t atlantronic_write(struct file *file, const char *user_buffer, size_t count, loff_t *ppos);

static void atlantronic_urb_in_callback(struct urb *urb);
static void atlantronic_urb_out_callback(struct urb *urb);

static int atlantronic_probe(struct usb_interface *interface, const struct usb_device_id *id);
static void atlantronic_disconnect(struct usb_interface *interface);

static int __init atlantronic_init(void); //!< fonction d'initialisation du module (appelée à l'insertion du module dans le noyau)
static void __exit atlantronic_exit(void); //!< fonction de fermeture appelée lors du retrait du module du noyau
static void atlantronic_delete(struct kref *kref);

static int atlantronic_task(void* arg);

//! @brief table des id des périphériques gérés par le pilote
static struct usb_device_id atlantronic_device_id [] =
{
	{ USB_DEVICE(ATLANTRONIC_ID, ATLANTRONIC_FOO_ID) },
	{ USB_DEVICE(ATLANTRONIC_ID, ATLANTRONIC_BAR_ID) },
	{ }
};

MODULE_DEVICE_TABLE(usb, atlantronic_device_id); //!< ajout de la table des périphériques gérés par le module

struct atlantronic_id
{
	struct list_head list;
	int id;
};

// TODO mutex
static struct atlantronic_id atlantronic_foo_list;
static struct atlantronic_id atlantronic_bar_list;

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
	struct usb_device*		  udev;                //!< udev
	struct usb_interface*	  interface;           //!< interface
	unsigned char             interface_subclass;  //!< type d'interface
	struct atlantronic_id     id;                  //!< id dans la liste foo ou bar
	struct usb_class_driver   class;               //!< class driver
	struct task_struct*       task;                //!< tache de recuperation des donnees
	struct completion         in_completion;       //!< completion
	volatile char             thread_stop_req;     //!< request thread to stop
	spinlock_t                err_lock;            //!< lock sur error
	int                       error;               //!< erreur urb
	int                       open_count;          //!< nombre d'ouvertures
	struct mutex              io_mutex;            //!< mutex i/o
	struct semaphore          write_limit_sem;     //!< semaphore de limitation du nombre d'urb en cours d'envoi
	wait_queue_head_t         wait;                //!< liste de processus en attente dans le read
	unsigned char             out_ep_addr;         //!< adresse de l'endpoint OUT2
	struct usb_anchor         urb_submitted;       //!< permet d'annuler les urb en cours d'envoi si necessaire
	struct urb*   in_urb[ATLANTRONIC_MAX_IN_URB];    //!< urb endpoint IN1
	unsigned char buffer[ATLANTRONIC_BUFFER_SIZE];   //!< buffer circulaire
	int           buffer_begin;     //!< indice de debut du buffer circulaire
	int           buffer_end;       //!< indice de fin du buffer circulaire
	struct kref   kref;             //!< compteur de references
};

static const struct file_operations atlantronic_fops =
{
	.owner =	THIS_MODULE,
	.read =		atlantronic_read,
	.write =    atlantronic_write,
	.open =		atlantronic_open,
	.release =	atlantronic_release,
	.flush =	NULL,
	.llseek =	NULL,
};

static void atlantronic_delete(struct kref *kref)
{
	struct atlantronic_data* dev;
	int i;

	debug(1, "atlantronic_delete");

	dev = container_of(kref, struct atlantronic_data, kref);

	usb_put_dev(dev->udev);

	for(i = 0; i < ATLANTRONIC_MAX_IN_URB; i++)
	{
		if( dev->in_urb[i] )
		{
			usb_kill_urb(dev->in_urb[i]);
			if(dev->in_urb[i]->transfer_buffer)
			{
				kfree(dev->in_urb[i]->transfer_buffer);
			}
			usb_free_urb(dev->in_urb[i]);
		}
	}

	if( dev->class.name )
	{
		kfree(dev->class.name);
	}

	list_del(&dev->id.list);

	kfree(dev);
}

static unsigned int atlantronic_get_min_id(struct atlantronic_id* id_list)
{
	struct list_head *pos, *q;
	struct atlantronic_id* tmp;
	int max = -1;

	list_for_each_safe(pos, q, &id_list->list)
	{
		tmp = list_entry(pos, struct atlantronic_id, list);
		if(tmp->id > max)
		{
			max = tmp->id;
		}
	}

	max++;

	if(max < 0)
	{
		max = 0;
	}
	return max;
}

static int atlantronic_open(struct inode *inode, struct file *file)
{
	struct atlantronic_data* dev;
	struct usb_interface* interface;
	int subminor;
	int rep = 0;

	debug(1, "atlantronic_open");

	subminor = iminor(inode);

	// récupération de l'interface liée au périphérique
	interface = usb_find_interface(&atlantronic_driver, subminor);
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
	kref_put(&dev->kref, atlantronic_delete);
error:
	return rep;
}

static int atlantronic_release(struct inode *inode, struct file *file)
{
	struct atlantronic_data* dev = file->private_data;

	debug(1, "atlantronic_release");

	if(dev == NULL)
	{
		return -ENODEV;
	}

	mutex_lock(&dev->io_mutex);

	if(dev->interface)
	{
		if (!--dev->open_count && dev->interface)
		{
			usb_autopm_put_interface(dev->interface);
		}
	}
	mutex_unlock(&dev->io_mutex);

	kref_put(&dev->kref, atlantronic_delete);

	return 0;
}

static ssize_t atlantronic_write(struct file *file, const char *user_buffer, size_t count, loff_t *ppos)
{
	int rep = 0;
	struct atlantronic_data* dev = file->private_data;
	struct urb *urb = NULL;
	char* buf = NULL;
	size_t writesize = min(count, (size_t)ATLANTRONIC_MAX_WRITE_SIZE);

	debug(1, "atlantronic_write");

	if(dev == NULL)
	{
		rep = -ENODEV;
		goto end;
	}

	if( count == 0)
	{
		goto end;
	}

	// limitation du nombre d'urb en cours d'envoi
	if (!(file->f_flags & O_NONBLOCK))
	{
		if (down_interruptible(&dev->write_limit_sem))
		{
			rep = -ERESTARTSYS;
			goto end;
		}
	}
	else
	{
		if (down_trylock(&dev->write_limit_sem))
		{
			rep = -EAGAIN;
			goto end;
		}
	}

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
	{
		rep = -ENOMEM;
		goto error_up_sem;
	}

	buf = usb_alloc_coherent(dev->udev, writesize, GFP_KERNEL, &urb->transfer_dma);

	if (!buf)
	{
		rep = -ENOMEM;
		goto error_free_urb;
	}

	if (copy_from_user(buf, user_buffer, writesize))
	{
		rep = -EFAULT;
		goto error_free_buf;
	}

	mutex_lock(&dev->io_mutex);
	if (!dev->interface)
	{
		mutex_unlock(&dev->io_mutex);
		rep = -ENODEV;
		goto error_free_buf;
	}

	usb_fill_bulk_urb(urb, dev->udev, usb_sndbulkpipe(dev->udev, dev->out_ep_addr), buf, writesize, atlantronic_urb_out_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->urb_submitted);

	rep = usb_submit_urb(urb, GFP_KERNEL);
	mutex_unlock(&dev->io_mutex);

	if(rep)
	{
		err("failed submitting write urb, error %d", rep);
		goto error_unanchor;
	}

	// on libere notre ref. Sera libere plus tard par usbcore quand il en aura termine avec l'urb
	usb_free_urb(urb);

	return writesize;

error_unanchor:
	usb_unanchor_urb(urb);
error_free_buf:
	usb_free_coherent(dev->udev, writesize, buf, urb->transfer_dma);
error_free_urb:
	usb_free_urb(urb);
error_up_sem:
	up(&dev->write_limit_sem);
end:
	return rep;
}

void atlantronic_urb_out_callback(struct urb *urb)
{
	struct atlantronic_data* dev = urb->context;

//	spin_lock(&dev->err_lock);
//	dev->error = urb->status;
//	spin_unlock(&dev->err_lock);

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
			break;
		default:
			err("status urb non nul: %d - taille = %d", urb->status, urb->actual_length);
			break;
	}

	usb_free_coherent(urb->dev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
	up(&dev->write_limit_sem);
}

static ssize_t atlantronic_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct atlantronic_data* dev = file->private_data;
	int size = 0;
	int rep;
	int n_cpy = 0;

	debug(1, "atlantronic_read");

	if(dev == NULL)
	{
		rep = -ENODEV;
		goto error;
	}

	while( n_cpy == 0 && count > 0)
	{
		if( wait_event_interruptible(dev->wait, dev->buffer_begin != dev->buffer_end || dev->thread_stop_req))
		{
			debug(2, "interruption par un signal");
			rep = -ERESTARTSYS;
			goto error;
		}

		if( dev->thread_stop_req )
		{
			// usb deconnecte, on retourne 0 (fin du fichier)
			rep = 0;
			goto error;
		}

		mutex_lock(&dev->io_mutex);

		size = dev->buffer_end - dev->buffer_begin;
		if(size < 0)
		{
			size = ATLANTRONIC_BUFFER_SIZE - dev->buffer_begin;
			if(size > count)
			{
				size = count;
			}

			if( copy_to_user(buffer, dev->buffer + dev->buffer_begin, size) )
			{
				err("copy_to_user failed (size = %d)", size);
				rep = -EFAULT;
				goto error_unlock;
			}

			n_cpy = size;
			dev->buffer_begin = (dev->buffer_begin + size) % ATLANTRONIC_BUFFER_SIZE;
		}

		size = dev->buffer_end - dev->buffer_begin;
		if(size > 0)
		{
			if(size > count - n_cpy)
			{
				size = count - n_cpy;
			}

			if( copy_to_user(buffer + n_cpy, dev->buffer + dev->buffer_begin, size) )
			{
				err("copy_to_user failed (size = %d)", size);
				rep = -EFAULT;
				goto error_unlock;
			}

			n_cpy += size;
			dev->buffer_begin = (dev->buffer_begin + size) % ATLANTRONIC_BUFFER_SIZE;
		}

		mutex_unlock(&dev->io_mutex);
	}

	return n_cpy;

error_unlock:
	mutex_unlock(&dev->io_mutex);
error:
	return rep;
}

static void atlantronic_urb_in_callback(struct urb *urb)
{
	struct atlantronic_data* dev = urb->context;

	spin_lock(&dev->err_lock);
	dev->error = urb->status;
	spin_unlock(&dev->err_lock);

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
			break;
		default:
			err("status urb non nul: %d - taille = %d", urb->status, urb->actual_length);
			break;
	}

	complete(&dev->in_completion);
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
	struct usb_endpoint_descriptor *endpointIn;
	struct usb_endpoint_descriptor *endpointOut;
	int pipe;
	int i;

    info("atlantronic_probe");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if( ! dev )
	{
		err("Out of memory");
		goto error;
	}

	kref_init(&dev->kref);
	sema_init(&dev->write_limit_sem, ATLANTRONIC_MAX_OUT_URB);
	init_usb_anchor(&dev->urb_submitted);
	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;
	init_completion(&dev->in_completion);
	mutex_init(&dev->io_mutex);
	init_waitqueue_head(&dev->wait);

	dev->class.name = kmalloc( ATLANTRONIC_MAX_PRODUCT_NAME_SIZE, GFP_KERNEL);
	if( ! dev->class.name )
	{
		err("Out of memory");
		goto error;
	}

	INIT_LIST_HEAD(&dev->id.list);
	dev->id.id = -1;

	iface_desc = interface->cur_altsetting;

	dev->interface_subclass = iface_desc->desc.bInterfaceSubClass;
	if( dev->interface_subclass != ATLANTRONIC_SUBCLASS)
	{
		rep = -ENODEV;
		goto error;
	}

	dev->class.fops = &atlantronic_fops;

	if(id->idProduct == ATLANTRONIC_FOO_ID)
	{
		dev->id.id = atlantronic_get_min_id(&atlantronic_foo_list);
		list_add(&dev->id.list, &atlantronic_foo_list.list);
		snprintf(dev->class.name, ATLANTRONIC_MAX_PRODUCT_NAME_SIZE - 1, "foo%d", dev->id.id);
	}
	else if(id->idProduct == ATLANTRONIC_BAR_ID)
	{
		dev->id.id = atlantronic_get_min_id(&atlantronic_bar_list);
		list_add(&dev->id.list, &atlantronic_bar_list.list);
		snprintf(dev->class.name, ATLANTRONIC_MAX_PRODUCT_NAME_SIZE - 1, "bar%d", dev->id.id);
	}
	else
	{
		rep = -ENODEV;
		goto error;
	}

	dev->class.name[ATLANTRONIC_MAX_PRODUCT_NAME_SIZE - 1] = 0;

	if( iface_desc->desc.bNumEndpoints != 2)
	{
		err("unknown interface: subclass=%i NumEndpoints=%i", dev->interface_subclass, iface_desc->desc.bNumEndpoints);
		rep = -ENODEV;
		goto error;
	}

	endpointIn = &iface_desc->endpoint[0].desc;
	if( ! usb_endpoint_is_bulk_in(endpointIn) )
	{
		err("wrong endpoint: subclass=%i endPointAddr=%i", dev->interface_subclass, endpointIn->bEndpointAddress);
		rep = -ENODEV;
		goto error;
	}

	endpointOut = &iface_desc->endpoint[1].desc;
	if( ! usb_endpoint_is_bulk_out(endpointOut) )
	{
		err("wrong endpoint: subclass=%i endPointAddr=%i", dev->interface_subclass, endpointOut->bEndpointAddress);
		rep = -ENODEV;
		goto error;
	}

	dev->out_ep_addr = endpointOut->bEndpointAddress;
	dev->buffer_begin = 0;
	dev->buffer_end = 0;

	// allocation des urb
	for(i = 0; i < ATLANTRONIC_MAX_IN_URB; i++)
	{
		dev->in_urb[i] = usb_alloc_urb(0, GFP_KERNEL);

		if( ! dev->in_urb[i] )
		{
			err("Out of memory");
			goto error;
		}

		dev->in_urb[i]->transfer_buffer = kmalloc(le16_to_cpu(endpointIn->wMaxPacketSize), GFP_KERNEL);
		if( ! dev->in_urb[i]->transfer_buffer )
		{
			err("Put of memory");
			goto error;
		}

		pipe = usb_rcvbulkpipe(dev->udev, endpointIn->bEndpointAddress);
		usb_fill_bulk_urb(dev->in_urb[i], dev->udev, pipe, dev->in_urb[i]->transfer_buffer, le16_to_cpu(endpointIn->wMaxPacketSize), atlantronic_urb_in_callback, dev);
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
	dev->task = kthread_run(atlantronic_task, dev, "atlantronic%d", interface->minor);

	if( IS_ERR(dev->task) )
	{
		dev->task = NULL;
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

	info("interface attached to %s", dev->class.name);

	return 0;

error_deregister:
	usb_deregister_dev(interface, &dev->class);

error_intfdata:
	usb_set_intfdata(interface, NULL);

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

	mutex_lock(&dev->io_mutex);
	dev->interface = NULL;
	mutex_unlock(&dev->io_mutex);

	usb_kill_anchored_urbs(&dev->urb_submitted);

	usb_deregister_dev(interface, &dev->class);

	dev->thread_stop_req = 1;

	complete(&dev->in_completion);
	if( dev->task)
	{
		wake_up_process(dev->task);
	}

	// debloquage du read
	wake_up_interruptible(&dev->wait);

	kref_put(&dev->kref, atlantronic_delete);
}

static int atlantronic_task(void* arg)
{
	struct atlantronic_data* dev = arg;
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
			dev->buffer[dev->buffer_end] = ((__u8*)urb->transfer_buffer)[i];
			dev->buffer_end = (dev->buffer_end + 1) % ATLANTRONIC_BUFFER_SIZE;
			if( dev->buffer_end == dev->buffer_begin )
			{
				// buffer circulaire plein
				dev->buffer_begin = (dev->buffer_begin + 1) % ATLANTRONIC_BUFFER_SIZE;
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
	kref_put(&dev->kref, atlantronic_delete);
	return 0;
}

static int __init atlantronic_init(void)
{
	int rep = 0;

	info("Atlantronic_log : init");

	INIT_LIST_HEAD(&atlantronic_foo_list.list);
	atlantronic_foo_list.id = -1;

	INIT_LIST_HEAD(&atlantronic_bar_list.list);
	atlantronic_bar_list.id = -1;

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
	info("Atlantronic_log : exit");
	usb_deregister(&atlantronic_driver);
}

// points d'entrées dans le noyau
module_init( atlantronic_init );
module_exit( atlantronic_exit );
