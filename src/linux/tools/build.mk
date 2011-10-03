obj-linux-plot_hokuyo += linux/tools/plot_hokuyo.o
obj-linux-plot_hokuyo += kernel/hokuyo_tools.o
obj-linux-plot_hokuyo += kernel/vect_pos.o
lib-linux-plot_hokuyo += -lm

bin-linux += plot_hokuyo
