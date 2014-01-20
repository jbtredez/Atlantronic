obj-discovery-test += kernel/main.o
obj-discovery-test += kernel/asm/isr_stm32f4xx.o
obj-discovery-test += kernel/rcc.o
obj-discovery-test += kernel/systick.o
obj-discovery-test += kernel/tasks.o
obj-discovery-test += kernel/queue.o
obj-discovery-test += kernel/port.o
obj-discovery-test += kernel/list.o
obj-discovery-test += kernel/heap_1.o
obj-discovery-test += kernel/log.o
obj-discovery-test += kernel/fault.o
obj-discovery-test += kernel/driver/spi.o
obj-discovery-test += kernel/driver/usb/usb.o
obj-discovery-test += kernel/driver/usb/usb_descriptor.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usb_core.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usb_dcd.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usb_dcd_int.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usbd_core.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usbd_ioreq.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usbd_req.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usbd_atlantronic_core.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usbd_usr.o
obj-discovery-test += kernel/driver/usb/stm32f4xx/usbd_desc.o
obj-discovery-test += kernel/driver/can.o
obj-discovery-test += kernel/canopen.o
obj-discovery-test += kernel/can_motor.o
obj-discovery-test += kernel/math/vect_plan.o
obj-discovery-test += kernel/math/vect2.o
obj-discovery-test += kernel/control/kinematics.o
obj-discovery-test += kernel/geometric_model/geometric_model.o
obj-discovery-test += kernel/location/location.o
obj-discovery-test += discovery/gpio.o
obj-discovery-test += discovery/control.o
obj-discovery-test += kernel/end.o

bin-discovery += test

obj-discovery-baz = $(obj-discovery-test)
obj-discovery-baz += kernel/driver/usart.o
obj-discovery-baz += kernel/driver/hokuyo.o
obj-discovery-baz += kernel/hokuyo_tools.o
obj-discovery-baz += kernel/driver/dynamixel.o
obj-discovery-baz += kernel/driver/pwm.o
obj-discovery-baz += kernel/driver/encoder.o
obj-discovery-baz += kernel/driver/adc.o
obj-discovery-baz += kernel/math/regression.o
obj-discovery-baz += discovery/graph.o
#obj-discovery-baz += discovery/trajectory.o
#obj-discovery-baz += discovery/detection.o
obj-discovery-baz += discovery/table.o

bin-discovery += baz
