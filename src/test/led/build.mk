obj-test_led += main.o
obj-test_led += syscalls.o
obj-test_led += test/led/task1.o
obj-test_led += io/$(ARCH)/rcc.o
obj-test_led += arch/$(ARCH)/gpio.o
obj-test_led += io/$(ARCH)/systick.o
obj-test_led += rtos/list.o
obj-test_led += rtos/tasks.o
obj-test_led += rtos/portable/MemMang/heap_2.o
obj-test_led += $(OBJ-PORT)
lib-led += -lm
BIN-arm_cm3 += test_led

