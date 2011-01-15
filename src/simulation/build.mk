ifeq ($(ARCH),gcc_posix)
obj-simu += simulation/simu.o
obj-simu += simulation/Environnement.o
obj-simu += simulation/ArmCm3.o 
obj-simu += simulation/ArmRcc.o
obj-simu += simulation/ArmTim.o
obj-simu += simulation/ArmGpio.o
obj-simu += simulation/ArmUsart.o
obj-simu += simulation/CpuEmu.o
obj-simu += simulation/Motor.o
obj-simu += simulation/Robot.o
obj-simu += simulation/Table.o
obj-simu += simulation/Pion.o
obj-simu += simulation/log.o

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

BIN+= simu
endif
