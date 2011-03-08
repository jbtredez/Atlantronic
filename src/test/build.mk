ifeq ($(SIMU),1)
obj-test_trapeze += test/test_trapeze.o
obj-test_trapeze += trapeze.o
lib-test_trapeze += -lm
BIN += test_trapeze
endif

obj-test_control += main.o
obj-test_control += test/strategy_test_control.o
obj-test_control += control/control.o
obj-test_control += control/pid.o
obj-test_control += trapeze.o
obj-test_control += location/odometry.o
obj-test_control += location/location.o
obj-test_control += location/beacon.o
obj-test_control += io/$(ARCH)/rcc.o
obj-test_control += io/$(ARCH)/encoders.o
obj-test_control += arch/$(ARCH)/gpio.o
obj-test_control += io/$(ARCH)/usart.o
obj-test_control += io/$(ARCH)/systick.o
obj-test_control += end.o
obj-test_control += io/current.o
obj-test_control += io/ax12.o
obj-test_control += io/$(ARCH)/pwm.o
obj-test_control += log.o
obj-test_control += rtos/list.o
obj-test_control += rtos/queue.o
obj-test_control += rtos/tasks.o
obj-test_control += rtos/portable/MemMang/heap_3.o
obj-test_control += $(OBJ-PORT)
lib-test_control += -lm
BIN += test_control

ifeq ($(SIMU),0)
obj-test_control += syscalls.o
endif

ifeq ($(SIMU),1)
obj-test_control += simu/model.o
obj-test_control += simu/model_motor.o
endif
