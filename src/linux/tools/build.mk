obj-linux-usb_interface += linux/tools/usb_interface.o
obj-linux-usb_interface += kernel/hokuyo_tools.o
obj-linux-usb_interface += kernel/vect_pos.o
lib-linux-usb_interface += -lm
bin-linux += usb_interface
