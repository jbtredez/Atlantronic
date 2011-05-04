obj-linux-test_hokuyo += linux/test/hokuyo/test_hokuyo.o
obj-linux-test_hokuyo += kernel/hokuyo_tools.o

lib-linux-test_hokuyo += -lm

bin-linux += test_hokuyo

