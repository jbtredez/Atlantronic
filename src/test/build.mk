ifeq ($(SIMU),1)
obj-test_trapeze += test/test_trapeze.o
obj-test_trapeze += trapeze.o
lib-test_trapeze += -lm
BIN += test_trapeze
endif

