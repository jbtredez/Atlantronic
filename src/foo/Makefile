CROSSCOMPILE:=arm-elf-

MCPU:=cortex-m3
MTUNE:=cortex-m3

LDSCRIPT:=scripts/ld/arm-elf.ld

DEF+=__ARM_CM3__=1
DEF+=STM32F10X_CL

CFLAGS:=-mcpu=$(MCPU)
CFLAGS+=-msoft-float
CFLAGS+=-mfpu=vfp
CFLAGS+=-mthumb
CFLAGS+=-x c
CFLAGS+=-Wall
CFLAGS+=-Wextra
CFLAGS+=-Wswitch-default
CFLAGS+=-Wswitch-enum
CFLAGS+=-Wunsuffixed-float-constants
CFLAGS+=-mno-thumb-interwork
#CFLAGS+=-O2
CFLAGS+=-fomit-frame-pointer
CFLAGS+=-g3

LDFLAGS:=-mcpu=$(MCPU)
LDFLAGS+=-nostartfiles
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

DEF+=__$(ARCH)__

ifeq ($(DEBUG),1)

else
DEF+= NDEBUG
#CFLAGS+=-O3
#LDFLAGS+=-O3
endif

CFLAGS+=$(addprefix -D,$(DEF))

INCLUDES+=-Iinclude/foo

