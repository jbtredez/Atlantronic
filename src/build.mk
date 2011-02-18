obj-rtos += main.o
obj-rtos += control/control.o
obj-rtos += control/pid.o
obj-rtos += trapeze.o
obj-rtos += location/location.o
obj-rtos += location/odometry.o
obj-rtos += location/beacon.o
obj-rtos += io/$(ARCH)/rcc.o
obj-rtos += io/$(ARCH)/gpio.o
obj-rtos += io/$(ARCH)/usart.o
obj-rtos += io/$(ARCH)/encoders.o
obj-rtos += io/$(ARCH)/systick.o
obj-rtos += io/ax12.o
obj-rtos += io/current.o
obj-rtos += io/$(ARCH)/pwm.o
obj-rtos += end.o
obj-rtos += log.o
obj-rtos += rtos/list.o
obj-rtos += rtos/queue.o
obj-rtos += rtos/tasks.o
obj-rtos += rtos/portable/MemMang/heap_3.o
obj-rtos += $(OBJ-PORT)
lib-rtos += -lm
BIN+= rtos

ifeq ($(SIMU),0)
obj-rtos += syscalls.o
endif

ifeq ($(SIMU),1)
obj-rtos += simu/model.o
obj-rtos += simu/model_motor.o
endif
