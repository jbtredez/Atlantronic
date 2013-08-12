CROSSCOMPILE:=arm-none-eabi-

MCPU:=cortex-m3

LDSCRIPT:=scripts/ld/stm32f10x.ld

CFLAGS:=-mcpu=$(MCPU)
CFLAGS+=-msoft-float
CFLAGS+=-mfpu=vfp
CFLAGS+=-mthumb
CFLAGS+=-mno-thumb-interwork
CFLAGS+=-fomit-frame-pointer
CFLAGS+=-fno-exceptions
CFLAGS+=-fno-non-call-exceptions
CFLAGS+=-fsingle-precision-constant
CFLAGS+=-ffast-math
CFLAGS+=-Wall
CFLAGS+=-Wextra
CFLAGS+=-Wswitch-default
CFLAGS+=-Wswitch-enum
CFLAGS+=-DSTM32F10X_CL
CFLAGS+=-D__$(ARCH)__
CFLAGS+=-g3
ifneq ($(DEBUG),1)
CFLAGS+=-O2
endif

CXXFLAGS:=$(CFLAGS)
CXXFLAGS+=-fno-rtti
CXXFLAGS+=-ftemplate-depth-0
CXXFLAGS+=-Woverloaded-virtual

#cflag qu'on ne met pas pour le c++
CFLAGS+=-Wunsuffixed-float-constants

LDFLAGS:=-mcpu=$(MCPU)
LDFLAGS+=-nostartfiles
LDFLAGS+=-msoft-float
LDFLAGS+=-mfpu=vfp
LDFLAGS+=-mthumb
LDFLAGS+=-T $(LDSCRIPT)
LDFLAGS+=-lsupc++
LDFLAGS+=-lgcc
LDFLAGS+=-lc
LDFLAGS+=-lm
LDFLAGS+=-lg
LDFLAGS+=-g3

INCLUDES+=-Iinclude/$(ARCH) -Isrc/$(ARCH)
