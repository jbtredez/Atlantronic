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

core_baz += kernel/driver/pwm.o
core_baz += kernel/driver/encoder.o
core_baz += kernel/driver/power.o
core_baz += kernel/driver/adc.o
core_baz += kernel/driver/io.o
core_baz += kernel/match.o
core_baz += kernel/driver/usart.o
core_baz += kernel/driver/hokuyo.o
core_baz += kernel/driver/dynamixel.o
core_baz += kernel/driver/xbee.o
core_baz += kernel/driver/can.o

core_robot += kernel/control.o
core_robot += kernel/location/location.o
core_robot += kernel/math/vect_plan.o
core_robot += kernel/math/vect2.o
core_robot += kernel/math/regression.o
#core_robot += kernel/math/matrix_homogeneous.o
core_robot += kernel/math/segment_intersection.o
core_robot += kernel/kinematics_model/kinematics_model_diff.o
core_robot += kernel/control/kinematics.o
core_robot += kernel/hokuyo_tools.o
core_robot += kernel/detection.o
core_robot += kernel/table.o
#core_robot += kernel/driver/gyro/adxrs453.o
#core_robot += kernel/math/simpson_integrator.o
core_robot += kernel/state_machine/state_machine.o
core_robot += kernel/stratege_machine/stratege.o
core_robot += kernel/stratege_machine/action.o
#core_robot += kernel/canopen.o
core_robot += kernel/can_mip.o
core_robot += kernel/can_motor_mip.o
#core_robot += kernel/can_motor_faulhaber.o
core_robot += kernel/motion/motion.o
core_robot += kernel/motion/motion_speed_check.o
core_robot += kernel/motion/graph.o
core_robot += kernel/motion/trajectory.o
#core_robot += kernel/pump.o
#core_robot += kernel/arm.o

core_robot += kernel/driver/stepper_driver.o
core_robot += disco/elevator.o
core_robot += disco/finger.o
core_robot += disco/wing.o
core_robot += disco/recalage.o
core_robot += disco/clapet.o



obj-disco-core_os += ${core_os}
bin-disco += core_os

obj-disco-core_cpu_board += ${core_os}
obj-disco-core_cpu_board += ${core_usb}
obj-disco-core_cpu_board += ${core_cpu_board}
bin-disco += core_cpu_board

obj-disco-baz += ${core_os}
obj-disco-baz += ${core_usb}
obj-disco-baz += ${core_cpu_board}
obj-disco-baz += ${core_baz}
bin-disco += baz

obj-disco-robot += ${core_os}
obj-disco-robot += ${core_usb}
obj-disco-robot += ${core_cpu_board}
obj-disco-robot += ${core_baz}
obj-disco-robot += $(core_robot)
bin-disco += robot

obj-disco-homologation += ${core_os}
obj-disco-homologation += ${core_usb}
obj-disco-homologation += ${core_cpu_board}
obj-disco-homologation += $(core_baz)
obj-disco-homologation += $(core_robot)
obj-disco-homologation += disco/strat_simple.o
obj-disco-homologation += disco/strat_homol.o
bin-disco += homologation
