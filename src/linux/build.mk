###
# Fichiers interface USB communs au simulateur et au programme USB interface
###
obj-linux-itf-core += linux/usb_interface_common/com/com.o
obj-linux-itf-core += linux/usb_interface_common/com/com_usb.o
obj-linux-itf-core += linux/usb_interface_common/com/com_tcp.o
obj-linux-itf-core += linux/usb_interface_common/com/rs.o
obj-linux-itf-core += linux/usb_interface_common/com/com_xbee.o
obj-linux-itf-core += linux/usb_interface_common/cli.o
obj-linux-itf-core += linux/usb_interface_common/cmd.o
obj-linux-itf-core += linux/usb_interface_common/robot_interface.o
obj-linux-itf-core += linux/usb_interface_common/qemu.o
obj-linux-itf-core += linux/usb_interface_common/ring_buffer.o
obj-linux-itf-core += linux/usb_interface_common/server_tcp.o
obj-linux-itf-core += linux/usb_interface_common/Robot.o
obj-linux-itf-core += middleware/hokuyo_tools.o
obj-linux-itf-core += kernel/math/regression.o
obj-linux-itf-core += kernel/math/VectPlan.o
obj-linux-itf-core += kernel/math/matrix_homogeneous.o
obj-linux-itf-core += disco/table.o
obj-linux-itf-core += middleware/trajectory/Graph.o

###
# Programme principal USB interface
###
obj-linux-usb_interface += $(obj-linux-itf-core)
obj-linux-usb_interface += linux/usb_interface/usb_interface.o
lib-linux-usb_interface += -lm -lreadline
bin-linux += usb_interface


###
# Interface graphique du simulateur
###
obj-linux-glplot += $(obj-linux-itf-core)
obj-linux-glplot += linux/simulator_ui/glplot.o
obj-linux-glplot += linux/simulator_ui/opengl/Object3dBasic.o
obj-linux-glplot += linux/simulator_ui/opengl/object3d.o
obj-linux-glplot += linux/simulator_ui/opengl/table3d.o
obj-linux-glplot += linux/simulator_ui/opengl/StarRobot3d.o
obj-linux-glplot += linux/simulator_ui/opengl/GateRobot3d.o
obj-linux-glplot += linux/simulator_ui/opengl/gl_font.o
obj-linux-glplot += linux/simulator_ui/opengl/shader.o
obj-linux-glplot += linux/simulator_ui/opengl/main_shader.o
obj-linux-glplot += linux/simulator_ui/opengl/text_shader.o
obj-linux-glplot += linux/simulator_ui/opengl/table_scene.o
obj-linux-glplot += linux/simulator_ui/point_texture.o
obj-linux-glplot += linux/simulator_ui/glplot_main.o
obj-linux-glplot += linux/simulator_ui/graphique.o
obj-linux-glplot += linux/simulator_ui/joystick.o


GTK3_16?=$(shell if [ $$(pkg-config --modversion gtk+-3.0 | cut -d. -f2) -ge 16 ] ; then echo 1; else echo 0; fi 2> /dev/null)
ifeq ($(GTK3_16),1)
# gtk 3.16 ou plus
cxxflags-linux-linux/simulator_ui/glplot.o+=$(shell pkg-config --cflags gtk+-3.0) -DGTK3
lib-linux-glplot+=$(shell pkg-config --libs gtk+-3.0) -lreadline -lm -lassimp -lepoxy -lfreetype -lfontconfig
else
# utilisation gtk2
cxxflags-linux-linux/simulator_ui/glplot.o+=$(shell pkg-config --cflags gtk+-2.0) -I/usr/include/gtkgl-2.0
lib-linux-glplot+=$(shell pkg-config --libs gtk+-2.0) -lreadline -lm -lassimp -lepoxy -lfreetype -lfontconfig -lgtkgl-2.0
endif

cxxflags-linux-linux/simulator_ui/opengl/gl_font.o+=-I/usr/include/freetype2
cxxflags-linux-linux/simulator_ui/opengl/table_scene.o+=-I/usr/include/freetype2
bin-linux += glplot

