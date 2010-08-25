obj-rtos += main.o
obj-rtos += control.o
obj-rtos += odometry.o
obj-rtos += beacon.o
obj-rtos += io/encoders.o
obj-rtos += io/pwm.o
obj-rtos += time2.o
obj-rtos += log.o
obj-rtos += rtos/list.o
obj-rtos += rtos/queue.o
obj-rtos += rtos/tasks.o
obj-rtos += rtos/portable/MemMang/heap_3.o
obj-rtos += $(OBJ-PORT)
lib-rtos += -lm
BIN+= rtos

