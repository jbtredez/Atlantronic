obj-linux-itf-core += linux/tools/com/com.o
obj-linux-itf-core += linux/tools/com/com_usb.o
obj-linux-itf-core += linux/tools/com/com_tcp.o
obj-linux-itf-core += linux/tools/com/rs.o
obj-linux-itf-core += linux/tools/com/com_xbee.o
obj-linux-itf-core += linux/tools/cli.o
obj-linux-itf-core += linux/tools/cmd.o
obj-linux-itf-core += linux/tools/robot_interface.o
obj-linux-itf-core += linux/tools/qemu.o
obj-linux-itf-core += linux/tools/ring_buffer.o
obj-linux-itf-core += linux/tools/server_tcp.o
obj-linux-itf-core += linux/tools/Robot.o
obj-linux-itf-core += middleware/hokuyo_tools.o
obj-linux-itf-core += kernel/math/regression.o
obj-linux-itf-core += kernel/math/VectPlan.o
obj-linux-itf-core += kernel/math/matrix_homogeneous.o
obj-linux-itf-core += disco/table.o
obj-linux-itf-core += middleware/trajectory/Graph.o

obj-linux-usb_interface += $(obj-linux-itf-core)
obj-linux-usb_interface += linux/tools/usb_interface.o
lib-linux-usb_interface += -lm -lreadline
bin-linux += usb_interface

obj-linux-glplot += $(obj-linux-itf-core)
obj-linux-glplot += linux/tools/glplot.o
obj-linux-glplot += linux/tools/opengl/Object3dBasic.o
obj-linux-glplot += linux/tools/opengl/object3d.o
obj-linux-glplot += linux/tools/opengl/table3d.o
obj-linux-glplot += linux/tools/opengl/StarRobot3d.o
obj-linux-glplot += linux/tools/opengl/GateRobot3d.o
obj-linux-glplot += linux/tools/opengl/gl_font.o
obj-linux-glplot += linux/tools/opengl/shader.o
obj-linux-glplot += linux/tools/opengl/main_shader.o
obj-linux-glplot += linux/tools/opengl/text_shader.o
obj-linux-glplot += linux/tools/opengl/table_scene.o
obj-linux-glplot += linux/tools/point_texture.o
obj-linux-glplot += linux/tools/glplot_main.o
obj-linux-glplot += linux/tools/graphique.o
obj-linux-glplot += linux/tools/joystick.o


GTK3_16?=$(shell if [ $$(pkg-config --modversion gtk+-3.0 | cut -d. -f2) -ge 16 ] ; then echo 1; else echo 0; fi 2> /dev/null)
ifeq ($(GTK3_16),1)
# gtk 3.16 ou plus
cxxflags-linux-linux/tools/glplot.o+=$(shell pkg-config --cflags gtk+-3.0) -DGTK3
lib-linux-glplot+=$(shell pkg-config --libs gtk+-3.0) -lreadline -lm -lassimp -lepoxy -lfreetype -lfontconfig
else
# utilisation gtk2
cxxflags-linux-linux/tools/glplot.o+=$(shell pkg-config --cflags gtk+-2.0) -I/usr/include/gtkgl-2.0
lib-linux-glplot+=$(shell pkg-config --libs gtk+-2.0) -lreadline -lm -lassimp -lepoxy -lfreetype -lfontconfig -lgtkgl-2.0
endif

cxxflags-linux-linux/tools/opengl/gl_font.o+=-I/usr/include/freetype2
cxxflags-linux-linux/tools/opengl/table_scene.o+=-I/usr/include/freetype2
bin-linux += glplot

obj-linux-sin_table_gen += linux/tools/sin_table_gen.o
lib-linux-sin_table_gen += -lm
bin-linux += sin_table_gen

obj-linux-graph_gen += linux/tools/graph_gen.o
lib-linux-graph_gen += -lm
bin-linux += graph_gen
