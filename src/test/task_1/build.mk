obj-test_task_1 += main.o
obj-test_task_1 += syscalls.o
obj-test_task_1 += test/task_1/task1.o
obj-test_task_1 += test/task_1/task2.o
obj-test_task_1 += io/rcc.o
obj-test_task_1 += arch/gpio.o
obj-test_task_1 += io/systick.o
obj-test_task_1 += rtos/list.o
obj-test_task_1 += rtos/tasks.o
obj-test_task_1 += rtos/portable/MemMang/heap_2.o
obj-test_task_1 += rtos/portable/GCC/ARM_CM3/port.o
obj-test_task_1 += arch/isr.o

lib-test_task_1 += -lm
BIN-arm_cm3 += test_task_1

