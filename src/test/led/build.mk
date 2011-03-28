obj-test_led += main.o
obj-test_led += syscalls.o
obj-test_led += test/led/task1.o
obj-test_led += io/rcc.o
obj-test_led += arch/gpio.o
obj-test_led += io/systick.o
obj-test_led += rtos/list.o
obj-test_led += rtos/tasks.o
obj-test_led += rtos/portable/MemMang/heap_2.o
obj-test_led += rtos/portable/GCC/ARM_CM3/port.o
obj-test_led += arch/isr.o

lib-led += -lm
BIN-arm_cm3 += test_led

