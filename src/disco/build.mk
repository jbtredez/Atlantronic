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

core_baz += ${core_os}
core_baz += ${core_usb}
core_baz += ${core_cpu_board}
core_baz += kernel/driver/pwm.o
core_baz += kernel/driver/encoder.o
core_baz += kernel/driver/power.o
core_baz += kernel/driver/adc.o
core_baz += kernel/driver/io.o
core_baz += kernel/match.o
core_baz += kernel/control.o
core_baz += kernel/location/location.o
core_baz += kernel/math/vect_plan.o
core_baz += kernel/math/vect2.o
core_baz += kernel/math/regression.o
core_baz += kernel/math/matrix_homogeneous.o
core_baz += kernel/math/segment_intersection.o
core_baz += kernel/kinematics_model/kinematics_model_diff.o
core_baz += kernel/control/kinematics.o
core_baz += kernel/driver/usart.o
core_baz += kernel/driver/hokuyo.o
core_baz += kernel/driver/dynamixel.o
core_baz += kernel/driver/xbee.o
core_baz += kernel/hokuyo_tools.o
core_baz += kernel/driver/can.o
core_baz += kernel/detection.o
core_baz += kernel/table.o
#core_baz += kernel/driver/gyro/adxrs453.o
#core_baz += kernel/math/simpson_integrator.o
core_baz += kernel/state_machine/state_machine.o
#core_baz += kernel/canopen.o
core_baz += kernel/can_mip.o
core_baz += kernel/can_motor_mip.o
#core_baz += kernel/can_motor_faulhaber.o
core_baz += kernel/motion/motion.o
core_baz += kernel/motion/graph.o
core_baz += kernel/motion/trajectory.o
#core_baz += kernel/pump.o
#core_baz += kernel/arm.o

core_robot += kernel/driver/stepper_driver.o
core_robot += disco/elevator.o
core_robot += disco/finger.o
core_robot += disco/wing.o

obj-disco-core_os += ${core_os}
bin-disco += core_os

obj-disco-core_cpu_board += ${core_os}
obj-disco-core_cpu_board += ${core_usb}
obj-disco-core_cpu_board += ${core_cpu_board}
bin-disco += core_cpu_board

obj-disco-core += ${core_baz}
bin-disco += core

obj-disco-homologation += $(core_baz)
obj-disco-homologation += $(core_robot)
obj-disco-homologation += disco/strat_homol.o
bin-disco += homologation


