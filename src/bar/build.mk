obj-bar-us_hokuyo += kernel/main.o
obj-bar-us_hokuyo += kernel/asm/isr.o
obj-bar-us_hokuyo += kernel/tasks.o
obj-bar-us_hokuyo += kernel/list.o
obj-bar-us_hokuyo += kernel/queue.o
obj-bar-us_hokuyo += kernel/port.o
obj-bar-us_hokuyo += kernel/systick.o
obj-bar-us_hokuyo += kernel/heap_3.o
obj-bar-us_hokuyo += kernel/syscalls.o
obj-bar-us_hokuyo += kernel/rcc.o
obj-bar-us_hokuyo += kernel/log.o
obj-bar-us_hokuyo += kernel/driver/can.o
obj-bar-us_hokuyo += kernel/driver/usart.o
obj-bar-us_hokuyo += kernel/driver/hokuyo.o
obj-bar-us_hokuyo += kernel/hokuyo_tools.o
obj-bar-us_hokuyo += kernel/vect_pos.o
obj-bar-us_hokuyo += kernel/math/trigo.o
obj-bar-us_hokuyo += kernel/utf8.o
obj-bar-us_hokuyo += kernel/driver/usb/usb.o
obj-bar-us_hokuyo += kernel/driver/usb/usb_init.o
obj-bar-us_hokuyo += kernel/driver/usb/usb_prop.o
obj-bar-us_hokuyo += kernel/driver/usb/usb_desc.o
obj-bar-us_hokuyo += kernel/driver/usb/usb_core.o
obj-bar-us_hokuyo += kernel/driver/usb/usb_pwr.o
obj-bar-us_hokuyo += kernel/driver/usb/usb_istr.o
obj-bar-us_hokuyo += kernel/driver/usb/usb_sil.o
obj-bar-us_hokuyo += kernel/driver/usb/otgd_fs_dev.o
obj-bar-us_hokuyo += kernel/driver/usb/otgd_fs_cal.o
obj-bar-us_hokuyo += kernel/driver/usb/otgd_fs_int.o
obj-bar-us_hokuyo += kernel/driver/usb/otgd_fs_pcd.o
#obj-bar-us_hokuyo += kernel/end.o
obj-bar-us_hokuyo += bar/us.o
obj-bar-us_hokuyo += kernel/error.o
obj-bar-us_hokuyo += bar/hokuyo_to_can.o
obj-bar-us_hokuyo += bar/gpio.o
bin-bar += us_hokuyo
