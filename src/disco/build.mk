core_os += kernel/main.o
core_os += kernel/asm/isr_stm32f4xx.o
core_os += kernel/rcc.o
core_os += kernel/systick.o
core_os += kernel/tasks.o
core_os += kernel/queue.o
core_os += kernel/port.o
core_os += kernel/list.o
core_os += kernel/heap_1.o
core_os += kernel/fault.o
core_os += kernel/driver/gpio.o
core_os += kernel/log.o

core_usb += kernel/driver/usb/usb.o
core_usb += kernel/driver/usb/usb_descriptor.o
core_usb += kernel/driver/usb/stm32f4xx/stm32f4xx_hal_pcd.o
core_usb += kernel/driver/usb/stm32f4xx/stm32f4xx_hal_pcd_ex.o
core_usb += kernel/driver/usb/stm32f4xx/stm32f4xx_ll_usb.o
core_usb += kernel/driver/usb/stm32f4xx/usbd_core.o
core_usb += kernel/driver/usb/stm32f4xx/usbd_ctlreq.o
core_usb += kernel/driver/usb/stm32f4xx/usbd_ioreq.o
core_usb += kernel/driver/usb/stm32f4xx/usbd_conf.o

core_cpu_board += kernel/driver/sdram.o
core_cpu_board += kernel/driver/spi.o
core_cpu_board += kernel/driver/gyro/l3gd20.o
core_cpu_board += kernel/driver/lcd/lcd.o
core_cpu_board += kernel/driver/i2c.o
core_cpu_board += kernel/driver/exti.o
core_cpu_board += kernel/driver/ts/stmpe811.o

obj-disco-core_os += ${core_os}
bin-disco += core_os

obj-disco-core_cpu_board += ${core_os}
obj-disco-core_cpu_board += ${core_usb}
obj-disco-core_cpu_board += ${core_cpu_board}
bin-disco += core_cpu_board

obj-disco-core += ${core_os}
obj-disco-core += ${core_usb}
obj-disco-core += ${core_cpu_board}
obj-disco-core += kernel/driver/pwm.o
obj-disco-core += kernel/driver/encoder.o
obj-disco-core += kernel/driver/power.o
obj-disco-core += kernel/driver/adc.o
obj-disco-core += kernel/driver/io.o
obj-disco-core += kernel/match.o
obj-disco-core += kernel/control.o
obj-disco-core += kernel/location/location.o
obj-disco-core += kernel/math/vect_plan.o
obj-disco-core += kernel/math/vect2.o
obj-disco-core += kernel/math/regression.o
obj-disco-core += kernel/math/matrix_homogeneous.o
obj-disco-core += kernel/geometric_model/geometric_model_diff.o
obj-disco-core += kernel/control/kinematics.o
obj-disco-core += kernel/driver/usart.o
obj-disco-core += kernel/driver/hokuyo.o
obj-disco-core += kernel/driver/dynamixel.o
obj-disco-core += kernel/driver/xbee.o
obj-disco-core += kernel/hokuyo_tools.o
#obj-disco-core += discovery/detection.o
#obj-disco-core += discovery/table.o

bin-disco += core
