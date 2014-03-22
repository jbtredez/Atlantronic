obj-discovery-core += kernel/main.o
obj-discovery-core += kernel/asm/isr_stm32f4xx.o
obj-discovery-core += kernel/rcc.o
obj-discovery-core += kernel/systick.o
obj-discovery-core += kernel/tasks.o
obj-discovery-core += kernel/queue.o
obj-discovery-core += kernel/port.o
obj-discovery-core += kernel/list.o
obj-discovery-core += kernel/heap_1.o
obj-discovery-core += kernel/log.o
obj-discovery-core += kernel/fault.o
obj-discovery-core += kernel/driver/usb/usb.o
obj-discovery-core += kernel/driver/usb/usb_descriptor.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usb_core.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usb_dcd.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usb_dcd_int.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usbd_core.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usbd_ioreq.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usbd_req.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usbd_atlantronic_core.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usbd_usr.o
obj-discovery-core += kernel/driver/usb/stm32f4xx/usbd_desc.o
obj-discovery-core += kernel/driver/usart.o
obj-discovery-core += kernel/end.o
obj-discovery-core += discovery/gpio.o

obj-discovery-gyro += $(obj-discovery-core)
obj-discovery-gyro += kernel/driver/spi.o
obj-discovery-gyro += kernel/driver/accelero.o
obj-discovery-gyro += kernel/driver/gyro.o
obj-discovery-gyro += kernel/math/simpson_integrator.o

obj-discovery-dynamixel += $(obj-discovery-core)
obj-discovery-dynamixel += kernel/driver/dynamixel.o

obj-discovery-baz += $(obj-discovery-core)
obj-discovery-baz += kernel/driver/spi.o
obj-discovery-baz += kernel/driver/accelero.o
obj-discovery-baz += kernel/driver/gyro.o
obj-discovery-baz += kernel/driver/can.o
obj-discovery-baz += kernel/driver/hokuyo.o
obj-discovery-baz += kernel/driver/dynamixel.o
obj-discovery-baz += kernel/driver/pwm.o
obj-discovery-baz += kernel/driver/encoder.o
obj-discovery-baz += kernel/driver/adc.o
obj-discovery-baz += kernel/canopen.o
obj-discovery-baz += kernel/can_motor.o
obj-discovery-baz += kernel/math/vect_plan.o
obj-discovery-baz += kernel/math/vect2.o
obj-discovery-baz += kernel/math/simpson_integrator.o
obj-discovery-baz += kernel/control/kinematics.o
obj-discovery-baz += kernel/geometric_model/geometric_model.o
obj-discovery-baz += kernel/location/location.o
obj-discovery-baz += kernel/hokuyo_tools.o
obj-discovery-baz += discovery/control.o


#obj-discovery-baz += kernel/math/regression.o
#obj-discovery-baz += discovery/graph.o
#obj-discovery-baz += discovery/trajectory.o
#obj-discovery-baz += discovery/detection.o
#obj-discovery-baz += discovery/table.o
#obj-discovery-baz += discovery/test.o

bin-discovery += core
bin-discovery += gyro
bin-discovery += dynamixel
bin-discovery += baz
