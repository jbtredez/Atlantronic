obj-bar-test_hokuyo += kernel/main.o
obj-bar-test_hokuyo += kernel/asm/isr.o
obj-bar-test_hokuyo += kernel/tasks.o
obj-bar-test_hokuyo += kernel/list.o
obj-bar-test_hokuyo += kernel/queue.o
obj-bar-test_hokuyo += kernel/port.o
obj-bar-test_hokuyo += kernel/systick.o
obj-bar-test_hokuyo += kernel/heap_3.o
obj-bar-test_hokuyo += kernel/syscalls.o
obj-bar-test_hokuyo += kernel/rcc.o
obj-bar-test_hokuyo += kernel/driver/usart.o
obj-bar-test_hokuyo += kernel/error.o
obj-bar-test_hokuyo += bar/gpio.o
obj-bar-test_hokuyo += kernel/driver/hokuyo.o
obj-bar-test_hokuyo += kernel/utf8.o
obj-bar-test_hokuyo += kernel/driver/usb/usb.o
obj-bar-test_hokuyo += kernel/driver/usb/usb_init.o
obj-bar-test_hokuyo += kernel/driver/usb/usb_prop.o
obj-bar-test_hokuyo += kernel/driver/usb/usb_desc.o
obj-bar-test_hokuyo += kernel/driver/usb/usb_core.o
obj-bar-test_hokuyo += kernel/driver/usb/usb_pwr.o
obj-bar-test_hokuyo += kernel/driver/usb/usb_istr.o
obj-bar-test_hokuyo += kernel/driver/usb/usb_sil.o
obj-bar-test_hokuyo += kernel/driver/usb/otgd_fs_dev.o
obj-bar-test_hokuyo += kernel/driver/usb/otgd_fs_cal.o
obj-bar-test_hokuyo += kernel/driver/usb/otgd_fs_int.o
obj-bar-test_hokuyo += kernel/driver/usb/otgd_fs_pcd.o
obj-bar-test_hokuyo += kernel/hokuyo_tools.o
obj-bar-test_hokuyo += kernel/vect_pos.o
obj-bar-test_hokuyo += bar/test/test_hokuyo.o
bin-bar += test_hokuyo

obj-bar-test_can_us += kernel/main.o
obj-bar-test_can_us += kernel/asm/isr.o
obj-bar-test_can_us += kernel/tasks.o
obj-bar-test_can_us += kernel/list.o
obj-bar-test_can_us += kernel/queue.o
obj-bar-test_can_us += kernel/port.o
obj-bar-test_can_us += kernel/systick.o
obj-bar-test_can_us += kernel/heap_3.o
obj-bar-test_can_us += kernel/syscalls.o
obj-bar-test_can_us += kernel/rcc.o
obj-bar-test_can_us += kernel/error.o
obj-bar-test_can_us += bar/gpio.o
bin-bar += test_can_us




