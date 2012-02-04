obj-foo-test_led += kernel/main.o
obj-foo-test_led += kernel/asm/isr.o
obj-foo-test_led += kernel/tasks.o
obj-foo-test_led += kernel/list.o
obj-foo-test_led += kernel/port.o
obj-foo-test_led += kernel/systick.o
obj-foo-test_led += kernel/heap_3.o
obj-foo-test_led += kernel/syscalls.o
obj-foo-test_led += kernel/rcc.o
obj-foo-test_led += kernel/test/led/task1.o
obj-foo-test_led += foo/gpio.o
bin-foo += test_led

obj-foo-test_tasks += kernel/main.o
obj-foo-test_tasks += kernel/asm/isr.o
obj-foo-test_tasks += kernel/tasks.o
obj-foo-test_tasks += kernel/list.o
obj-foo-test_tasks += kernel/port.o
obj-foo-test_tasks += kernel/systick.o
obj-foo-test_tasks += kernel/heap_3.o
obj-foo-test_tasks += kernel/syscalls.o
obj-foo-test_tasks += kernel/rcc.o
obj-foo-test_tasks += kernel/test/tasks/task1.o
obj-foo-test_tasks += kernel/test/tasks/task2.o
obj-foo-test_tasks += foo/gpio.o
bin-foo += test_tasks

obj-foo-test_end += kernel/main.o
obj-foo-test_end += kernel/asm/isr.o
obj-foo-test_end += kernel/tasks.o
obj-foo-test_end += kernel/list.o
obj-foo-test_end += kernel/port.o
obj-foo-test_end += kernel/systick.o
obj-foo-test_end += kernel/heap_3.o
obj-foo-test_end += kernel/syscalls.o
obj-foo-test_end += kernel/rcc.o
obj-foo-test_end += kernel/end.o
obj-foo-test_end += kernel/test/end/task1.o
obj-foo-test_end += foo/gpio.o
bin-foo += test_end

#obj-foo-test_can += kernel/main.o
#obj-foo-test_can += kernel/asm/isr.o
#obj-foo-test_can += kernel/tasks.o
#obj-foo-test_can += kernel/list.o
#obj-foo-test_can += kernel/queue.o
#obj-foo-test_can += kernel/port.o
#obj-foo-test_can += kernel/systick.o
#obj-foo-test_can += kernel/heap_3.o
#obj-foo-test_can += kernel/syscalls.o
#obj-foo-test_can += kernel/rcc.o
#obj-foo-test_can += kernel/driver/can.o
#obj-foo-test_can += kernel/error.o
#obj-foo-test_can += foo/location/location.o
#obj-foo-test_can += foo/location/odometry.o
#obj-foo-test_can += foo/location/encoder.o
#obj-foo-test_can += foo/us.o
#obj-foo-test_can += foo/gpio.o
#bin-foo += test_can

obj-foo-test_deplacement += kernel/main.o
obj-foo-test_deplacement += kernel/asm/isr.o
obj-foo-test_deplacement += kernel/tasks.o
obj-foo-test_deplacement += kernel/list.o
obj-foo-test_deplacement += kernel/queue.o
obj-foo-test_deplacement += kernel/port.o
obj-foo-test_deplacement += kernel/systick.o
obj-foo-test_deplacement += kernel/heap_3.o
obj-foo-test_deplacement += kernel/syscalls.o
obj-foo-test_deplacement += kernel/rcc.o
obj-foo-test_deplacement += kernel/end.o
obj-foo-test_deplacement += kernel/error.o
obj-foo-test_deplacement += kernel/driver/can.o
obj-foo-test_deplacement += kernel/math/regression.o
obj-foo-test_deplacement += kernel/math/segment_intersection.o
obj-foo-test_deplacement += kernel/math/trigo.o
obj-foo-test_deplacement += kernel/trapeze.o
obj-foo-test_deplacement += kernel/vect_pos.o
obj-foo-test_deplacement += kernel/driver/usart.o
obj-foo-test_deplacement += kernel/utf8.o
obj-foo-test_deplacement += kernel/driver/usb/usb.o
obj-foo-test_deplacement += kernel/driver/usb/usb_init.o
obj-foo-test_deplacement += kernel/driver/usb/usb_prop.o
obj-foo-test_deplacement += kernel/driver/usb/usb_desc.o
obj-foo-test_deplacement += kernel/driver/usb/usb_core.o
obj-foo-test_deplacement += kernel/driver/usb/usb_pwr.o
obj-foo-test_deplacement += kernel/driver/usb/usb_istr.o
obj-foo-test_deplacement += kernel/driver/usb/usb_sil.o
obj-foo-test_deplacement += kernel/driver/usb/otgd_fs_dev.o
obj-foo-test_deplacement += kernel/driver/usb/otgd_fs_cal.o
obj-foo-test_deplacement += kernel/driver/usb/otgd_fs_int.o
obj-foo-test_deplacement += kernel/driver/usb/otgd_fs_pcd.o
obj-foo-test_deplacement += foo/gpio.o
obj-foo-test_deplacement += foo/control/control.o
obj-foo-test_deplacement += foo/control/trajectory.o
obj-foo-test_deplacement += foo/control/pid.o
obj-foo-test_deplacement += foo/detection.o
obj-foo-test_deplacement += kernel/driver/hokuyo.o
obj-foo-test_deplacement += kernel/hokuyo_tools.o
obj-foo-test_deplacement += foo/pwm.o
obj-foo-test_deplacement += foo/location/odometry.o
obj-foo-test_deplacement += foo/location/location.o
obj-foo-test_deplacement += foo/location/beacon.o
obj-foo-test_deplacement += foo/encoders.o
obj-foo-test_deplacement += foo/test/test_deplacement.o
obj-foo-test_deplacement += foo/adc.o
obj-foo-test_deplacement += foo/us.o
obj-foo-test_deplacement += foo/pince.o
obj-foo-test_deplacement += foo/ax12.o
bin-foo += test_deplacement

obj-foo-test_pince += kernel/main.o
obj-foo-test_pince += kernel/asm/isr.o
obj-foo-test_pince += kernel/tasks.o
obj-foo-test_pince += kernel/list.o
obj-foo-test_pince += kernel/queue.o
obj-foo-test_pince += kernel/port.o
obj-foo-test_pince += kernel/systick.o
obj-foo-test_pince += kernel/heap_3.o
obj-foo-test_pince += kernel/syscalls.o
obj-foo-test_pince += kernel/rcc.o
obj-foo-test_pince += kernel/driver/usart.o
obj-foo-test_pince += kernel/end.o
obj-foo-test_pince += kernel/driver/can.o
obj-foo-test_pince += kernel/error.o
obj-foo-test_pince += foo/gpio.o
obj-foo-test_pince += foo/control/pid.o
obj-foo-test_pince += kernel/trapeze.o
obj-foo-test_pince += foo/pwm.o
obj-foo-test_pince += foo/encoders.o
obj-foo-test_pince += foo/ax12.o
obj-foo-test_pince += foo/pince.o
obj-foo-test_pince += foo/test/test_pince.o
obj-foo-test_pince += foo/adc.o
obj-foo-test_pince += kernel/utf8.o
obj-foo-test_pince += kernel/driver/usb/usb.o
obj-foo-test_pince += kernel/driver/usb/usb_init.o
obj-foo-test_pince += kernel/driver/usb/usb_prop.o
obj-foo-test_pince += kernel/driver/usb/usb_desc.o
obj-foo-test_pince += kernel/driver/usb/usb_core.o
obj-foo-test_pince += kernel/driver/usb/usb_pwr.o
obj-foo-test_pince += kernel/driver/usb/usb_istr.o
obj-foo-test_pince += kernel/driver/usb/usb_sil.o
obj-foo-test_pince += kernel/driver/usb/otgd_fs_dev.o
obj-foo-test_pince += kernel/driver/usb/otgd_fs_cal.o
obj-foo-test_pince += kernel/driver/usb/otgd_fs_int.o
obj-foo-test_pince += kernel/driver/usb/otgd_fs_pcd.o
bin-foo += test_pince

obj-foo-test_hokuyo += kernel/main.o
obj-foo-test_hokuyo += kernel/asm/isr.o
obj-foo-test_hokuyo += kernel/tasks.o
obj-foo-test_hokuyo += kernel/list.o
obj-foo-test_hokuyo += kernel/queue.o
obj-foo-test_hokuyo += kernel/port.o
obj-foo-test_hokuyo += kernel/systick.o
obj-foo-test_hokuyo += kernel/heap_3.o
obj-foo-test_hokuyo += kernel/syscalls.o
obj-foo-test_hokuyo += kernel/rcc.o
obj-foo-test_hokuyo += kernel/driver/usart.o
obj-foo-test_hokuyo += kernel/driver/can.o
obj-foo-test_hokuyo += kernel/end.o
obj-foo-test_hokuyo += kernel/error.o
obj-foo-test_hokuyo += foo/gpio.o
obj-foo-test_hokuyo += foo/location/odometry.o
obj-foo-test_hokuyo += foo/location/location.o
obj-foo-test_hokuyo += foo/location/beacon.o
obj-foo-test_hokuyo += foo/encoders.o
obj-foo-test_hokuyo += kernel/driver/hokuyo.o
obj-foo-test_hokuyo += kernel/hokuyo_tools.o
obj-foo-test_hokuyo += kernel/vect_pos.o
obj-foo-test_hokuyo += foo/detection.o
obj-foo-test_hokuyo += kernel/math/regression.o
obj-foo-test_hokuyo += kernel/math/segment_intersection.o
obj-foo-test_hokuyo += kernel/math/trigo.o
obj-foo-test_hokuyo += kernel/utf8.o
obj-foo-test_hokuyo += kernel/driver/usb/usb.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_init.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_prop.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_desc.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_core.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_pwr.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_istr.o
obj-foo-test_hokuyo += kernel/driver/usb/usb_sil.o
obj-foo-test_hokuyo += kernel/driver/usb/otgd_fs_dev.o
obj-foo-test_hokuyo += kernel/driver/usb/otgd_fs_cal.o
obj-foo-test_hokuyo += kernel/driver/usb/otgd_fs_int.o
obj-foo-test_hokuyo += kernel/driver/usb/otgd_fs_pcd.o
bin-foo += test_hokuyo
