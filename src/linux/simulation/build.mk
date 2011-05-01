obj-linux-simu += linux/simulation/simu.o
obj-linux-simu += linux/simulation/Environnement.o
obj-linux-simu += linux/simulation/ArmCm3.o
obj-linux-simu += linux/simulation/Ax12.o
obj-linux-simu += linux/simulation/CpuEmu.o
obj-linux-simu += linux/simulation/Motor.o
obj-linux-simu += linux/simulation/Robot.o
obj-linux-simu += linux/simulation/Table.o
obj-linux-simu += linux/simulation/Pion.o
obj-linux-simu += linux/simulation/log.o
obj-linux-simu += linux/simulation/rcc.o
obj-linux-simu += linux/simulation/tim.o
obj-linux-simu += linux/simulation/gpio.o
obj-linux-simu += linux/simulation/can.o
obj-linux-simu += linux/simulation/adc.o
obj-linux-simu += linux/simulation/usart.o

lib-linux-simu += -lstdc++
lib-linux-simu += -lIrrlicht
lib-linux-simu += -lNewton
lib-linux-simu += -lGL
lib-linux-simu += -lXxf86vm
lib-linux-simu += -lXext
lib-linux-simu += -lX11
lib-linux-simu += -ljpeg
lib-linux-simu += -lpng
lib-linux-simu += -lbz2

bin-linux += simu

