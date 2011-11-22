obj-linux-usb_interface += linux/tools/usb_interface.o
obj-linux-usb_interface += linux/tools/com.o
obj-linux-usb_interface += kernel/hokuyo_tools.o
obj-linux-usb_interface += kernel/vect_pos.o
obj-linux-usb_interface += linux/tools/cli.o
obj-linux-usb_interface += linux/tools/cmd.o
lib-linux-usb_interface += -lm -lreadline
bin-linux += usb_interface

obj-linux-glplot += linux/tools/glplot.o
cflags-linux-linux/tools/glplot.o+=$(shell pkg-config --cflags gtk+-2.0 gtkglext-1.0)
lib-linux-glplot+=$(shell pkg-config --libs gtk+-2.0 gtkglext-1.0)
bin-linux += glplot
