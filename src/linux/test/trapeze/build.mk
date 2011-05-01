obj-linux-test_trapeze += kernel/trapeze.o
obj-linux-test_trapeze += linux/test/trapeze/test_trapeze.o

lib-linux-test_trapeze += -lm

bin-linux += test_trapeze
