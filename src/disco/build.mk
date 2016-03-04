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
core_baz += kernel/driver/Dynamixel.o
core_baz += kernel/driver/DynamixelManager.o
core_baz += kernel/driver/xbee.o
core_baz += kernel/driver/can.o

core_robot += disco/star.o
core_robot += kernel/control.o
core_robot += kernel/location/location.o
core_robot += kernel/math/VectPlan.o
core_robot += kernel/math/Vect2.o
core_robot += kernel/math/regression.o
#core_robot += kernel/math/matrix_homogeneous.o
core_robot += kernel/math/segment_intersection.o
core_robot += kernel/kinematics_model/KinematicsModelDiff.o
core_robot += kernel/control/kinematics.o
core_robot += middleware/hokuyo_tools.o
core_robot += middleware/detection.o
core_robot += disco/table.o
#core_robot += kernel/driver/gyro/adxrs453.o
#core_robot += kernel/math/simpson_integrator.o
core_robot += middleware/state_machine/StateMachine.o
core_robot += middleware/state_machine/StateMachineState.o
core_robot += middleware/stratege_machine/stratege.o
core_robot += middleware/stratege_machine/action.o
core_robot += middleware/stratege_machine/action_composite.o

#core_robot += kernel/canopen.o
core_robot += kernel/CanMipNode.o
core_robot += kernel/CanMipMotor.o
core_robot += kernel/math/findRotation.o
core_robot += kernel/math/poly7.o
core_robot += middleware/motion/pid.o
core_robot += middleware/motion/Path.o
core_robot += middleware/motion/Motion.o
core_robot += middleware/motion/MotionMoveState.o
core_robot += middleware/motion/MotionDisabledState.o
core_robot += middleware/motion/MotionTryEnableState.o
core_robot += middleware/motion/MotionEnabledState.o
core_robot += middleware/motion/MotionActuatorKinematicsState.o
core_robot += middleware/motion/MotionSpeedState.o
core_robot += middleware/motion/MotionTrajectoryState.o
core_robot += middleware/motion/MotionInterruptingState.o
core_robot += middleware/motion/motion_speed_check.o
core_robot += middleware/trajectory/Graph.o
core_robot += middleware/trajectory/Trajectory.o
#core_robot += kernel/pump.o
#core_robot += kernel/arm.o

core_robot += kernel/driver/StepperDriver.o
core_robot += disco/elevator.o
core_robot += disco/finger.o
core_robot += disco/wing.o
core_robot += disco/carpet.o
core_robot += disco/recalage.o

core_robot += disco/robot_state.o
core_robot += disco/action/clapet.o
core_robot += disco/action/feet.o
core_robot += disco/action/feet_lateral.o
core_robot += disco/action/gobelet.o
core_robot += disco/action/Light.o
core_robot += disco/action/dropzone.o
core_robot += disco/action/Drop.o
core_robot += disco/action/MoveBackward.o
core_robot += disco/action/deposecarpette.o
core_robot += disco/action/SpotLight.o
core_robot += disco/action/Feed.o
core_robot += disco/action/Move.o



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
obj-disco-homologation += disco/strat/strat_simple.o
obj-disco-homologation += disco/strat_homol.o
bin-disco += homologation
