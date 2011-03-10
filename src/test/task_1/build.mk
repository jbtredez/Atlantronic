ifeq ($(SIMU),0)
obj-test_task_1 += main.o
obj-test_task_1 += syscalls.o
obj-test_task_1 += test/task_1/task1.o
obj-test_task_1 += test/task_1/task2.o
obj-test_task_1 += io/$(ARCH)/rcc.o
obj-test_task_1 += arch/$(ARCH)/gpio.o
obj-test_task_1 += io/$(ARCH)/systick.o
obj-test_task_1 += rtos/list.o
obj-test_task_1 += rtos/tasks.o
obj-test_task_1 += rtos/portable/MemMang/heap_2.o
obj-test_task_1 += $(OBJ-PORT)
lib-test_task_1 += -lm
BIN += test_task_1
endif

