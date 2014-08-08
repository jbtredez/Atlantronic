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

core_usb += kernel/driver/usb/usb.o
core_usb += kernel/driver/usb/usb_descriptor.o
core_usb += kernel/driver/usb/stm32f4xx/stm32f4xx_hal_pcd.o
core_usb += kernel/driver/usb/stm32f4xx/stm32f4xx_hal_pcd_ex.o
core_usb += kernel/driver/usb/stm32f4xx/stm32f4xx_ll_usb.o
core_usb += kernel/driver/usb/stm32f4xx/usbd_core.o
core_usb += kernel/driver/usb/stm32f4xx/usbd_ctlreq.o
core_usb += kernel/driver/usb/stm32f4xx/usbd_ioreq.o
core_usb += kernel/driver/usb/stm32f4xx/usbd_conf.o

obj-disco-core_os += ${core_os}
bin-disco += core_os

obj-disco-core += ${core_os}
obj-disco-core += ${core_usb}
obj-disco-core += kernel/log.o
obj-disco-core += kernel/driver/sdram.o
obj-disco-core += kernel/driver/spi.o
bin-disco += core

