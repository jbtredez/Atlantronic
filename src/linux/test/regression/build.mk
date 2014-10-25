obj-linux-test_regression += kernel/math/regression.o
obj-linux-test_regression += kernel/math/vect2.o
obj-linux-test_regression += linux/test/regression/test_regression.o

lib-linux-test_regression += -lm

bin-linux += test_regression
