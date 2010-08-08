obj-rtos += main.o
obj-rtos += time2.o
obj-rtos += log.o
obj-rtos += rtos/list.o
obj-rtos += rtos/queue.o
obj-rtos += rtos/tasks.o
obj-rtos += rtos/croutine.o
obj-rtos += rtos/portable/MemMang/heap_3.o
obj-rtos += $(OBJ-PORT)
BIN+= rtos

