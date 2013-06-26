CROSSCOMPILE:=arm-none-eabi-

MCPU:=cortex-m3

LDSCRIPT:=scripts/ld/stm32f10x.ld

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
CFLAGS+=-fomit-frame-pointer
CFLAGS+=-g

LDFLAGS:=-mcpu=$(MCPU)
LDFLAGS+=-nostartfiles
LDFLAGS+=-msoft-float
LDFLAGS+=-mfpu=vfp
LDFLAGS+=-mthumb
LDFLAGS+=-T $(LDSCRIPT)
LDFLAGS+=-lgcc
LDFLAGS+=-lc
LDFLAGS+=-lm
LDFLAGS+=-lg
LDFLAGS+=-g

DEF+=__$(ARCH)__

ifneq ($(DEBUG),1)
DEF+= NDEBUG
CFLAGS+=-O2
LDFLAGS+=-O2
endif

CFLAGS+=$(addprefix -D,$(DEF))

INCLUDES+=-Iinclude/$(ARCH) -Isrc/$(ARCH)
