obj-simu += simulation/simu.o
obj-simu += simulation/Environnement.o
obj-simu += simulation/ArmCm3.o
obj-simu += simulation/Ax12.o
obj-simu += simulation/CpuEmu.o
obj-simu += simulation/Motor.o
obj-simu += simulation/Robot.o
obj-simu += simulation/Table.o
obj-simu += simulation/Pion.o
obj-simu += simulation/log.o
obj-simu += simulation/rcc.o
obj-simu += simulation/tim.o
obj-simu += simulation/gpio.o
obj-simu += simulation/can.o
obj-simu += simulation/adc.o
obj-simu += simulation/usart.o

lib-simu += -lstdc++
lib-simu += -lIrrlicht
lib-simu += -lNewton
lib-simu += -lGL
lib-simu += -lXxf86vm
lib-simu += -lXext
lib-simu += -lX11
lib-simu += -ljpeg
lib-simu += -lpng
lib-simu += -lbz2

BIN-linux += simu

