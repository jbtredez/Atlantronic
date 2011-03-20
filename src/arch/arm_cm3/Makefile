CROSSCOMPILE:=arm-elf-
#CROSSCOMPILE:=arm-none-eabi-

MCPU:=cortex-m3
MTUNE:=cortex-m3

LDSCRIPT:=scripts/ld/arm-elf.ld

DEF+=__ARM_CM3__=1

CFLAGS:=-mcpu=$(MCPU)
CFLAGS+=-msoft-float
CFLAGS+=-mfpu=vfp
CFLAGS+=-mthumb
CFLAGS+=-x c
CFLAGS+=-Wall
CFLAGS+=-Wextra
CFLAGS+=-mno-thumb-interwork
#CFLAGS+=-O1
CFLAGS+=-fomit-frame-pointer
CFLAGS+=-g3

LDFLAGS:=-mcpu=$(MCPU)
LDFLAGS+=-msoft-float
LDFLAGS+=-mfpu=vfp
LDFLAGS+=-mthumb
LDFLAGS+=-T $(LDSCRIPT)
LDFLAGS+=-lgcc
LDFLAGS+=-lc
LDFLAGS+=-lm
#LDFLAGS+=-lnosys
LDFLAGS+=-lg 
LDFLAGS+=-g3

ifeq ($(DEBUG),1)

else
DEF+= NDEBUG
#CFLAGS+=-O3
#LDFLAGS+=-O3
endif

CFLAGS+=$(addprefix -D,$(DEF))

INCLUDES+=-Isrc/rtos/portable/GCC/ARM_CM3
OBJ-PORT+=rtos/portable/GCC/ARM_CM3/port.o
OBJ-PORT+=arch/$(ARCH)/isr.o
