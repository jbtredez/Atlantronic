obj-foo-common += kernel/main.o
obj-foo-common += kernel/asm/isr_stm32f1xx.o
obj-foo-common += kernel/tasks.o
obj-foo-common += kernel/list.o
obj-foo-common += kernel/queue.o
obj-foo-common += kernel/port.o
obj-foo-common += kernel/systick.o
obj-foo-common += kernel/heap_1.o
obj-foo-common += kernel/rcc.o
obj-foo-common += kernel/log.o
obj-foo-common += kernel/utf8.o
obj-foo-common += kernel/driver/usb/usb.o
obj-foo-common += kernel/driver/usb/stm32f1xx/usb_init.o
obj-foo-common += kernel/driver/usb/stm32f1xx/usb_prop.o
obj-foo-common += kernel/driver/usb/stm32f1xx/usb_desc.o
obj-foo-common += kernel/driver/usb/usb_descriptor.o
obj-foo-common += kernel/driver/usb/stm32f1xx/usb_core.o
obj-foo-common += kernel/driver/usb/stm32f1xx/usb_pwr.o
obj-foo-common += kernel/driver/usb/stm32f1xx/usb_istr.o
obj-foo-common += kernel/driver/usb/stm32f1xx/usb_sil.o
obj-foo-common += kernel/driver/usb/stm32f1xx/otgd_fs_dev.o
obj-foo-common += kernel/driver/usb/stm32f1xx/otgd_fs_cal.o
obj-foo-common += kernel/driver/usb/stm32f1xx/otgd_fs_int.o
obj-foo-common += kernel/driver/usb/stm32f1xx/otgd_fs_pcd.o
obj-foo-common += kernel/driver/usart.o
obj-foo-common += kernel/driver/can.o
obj-foo-common += kernel/driver/hokuyo.o
obj-foo-common += kernel/math/fx_math.o
obj-foo-common += kernel/math/regression.o
obj-foo-common += kernel/math/segment_intersection.o
obj-foo-common += kernel/math/vect2.o
obj-foo-common += kernel/math/distance.o
obj-foo-common += kernel/vect_pos.o
obj-foo-common += kernel/hokuyo_tools.o
obj-foo-common += kernel/end.o
obj-foo-common += kernel/canopen.o
obj-foo-common += kernel/can_motor.o
obj-foo-common += foo/pince.o
obj-foo-common += foo/ax12.o
obj-foo-common += foo/arm.o
obj-foo-common += foo/gpio.o
obj-foo-common += foo/pwm.o
obj-foo-common += kernel/trapeze.o
obj-foo-common += foo/control/pid.o
obj-foo-common += foo/control/control.o
obj-foo-common += foo/control/trajectory.o
obj-foo-common += foo/recalage.o
obj-foo-common += foo/graph.o
obj-foo-common += foo/detection.o
obj-foo-common += foo/adc.o
obj-foo-common += foo/table.o
obj-foo-common += foo/location/odometry.o
obj-foo-common += foo/location/location.o
obj-foo-common += foo/location/beacon.o
obj-foo-common += foo/encoders.o

# TODO - a voir pour les defauts ...
#obj-foo-common += kernel/fault.o

#
# STRAT COUPE 2012
#
STRAT = 2012

obj-foo-strat_$(STRAT) += $(obj-foo-common)
obj-foo-strat_$(STRAT) += foo/strat_$(STRAT).o
bin-foo += strat_$(STRAT)