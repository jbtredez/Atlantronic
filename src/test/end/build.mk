ifeq ($(SIMU),0)
obj-test_end += main.o
obj-test_end += end.o
obj-test_end += syscalls.o
obj-test_end += test/end/task1.o
obj-test_end += io/$(ARCH)/rcc.o
obj-test_end += arch/$(ARCH)/gpio.o
obj-test_end += arch/$(ARCH)/adc.o
obj-test_end += io/$(ARCH)/systick.o
obj-test_end += rtos/list.o
obj-test_end += rtos/tasks.o
obj-test_end += rtos/portable/MemMang/heap_2.o
obj-test_end += $(OBJ-PORT)
lib-led += -lm
BIN += test_end
endif
