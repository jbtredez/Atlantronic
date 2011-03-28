obj-test_end += main.o
obj-test_end += end.o
obj-test_end += syscalls.o
obj-test_end += test/end/task1.o
obj-test_end += io/rcc.o
obj-test_end += io/gpio.o
obj-test_end += io/adc.o
obj-test_end += io/systick.o
obj-test_end += rtos/list.o
obj-test_end += rtos/tasks.o
obj-test_end += rtos/portable/MemMang/heap_2.o
obj-test_end += rtos/portable/GCC/ARM_CM3/port.o
obj-test_end += asm/isr.o
lib-led += -lm
BIN-arm_cm3 += test_end

