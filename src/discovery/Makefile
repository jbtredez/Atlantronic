CROSSCOMPILE:=arm-none-eabi-

MCPU:=cortex-m4

LDSCRIPT:=scripts/ld/stm32f407.ld

CFLAGS:=-mcpu=$(MCPU)
CFLAGS+=-mfloat-abi=hard
CFLAGS+=-mfpu=fpv4-sp-d16
CFLAGS+=-mthumb
CFLAGS+=-mno-thumb-interwork
CFLAGS+=-fomit-frame-pointer
CFLAGS+=-fno-exceptions
CFLAGS+=-fno-non-call-exceptions
CFLAGS+=-fsingle-precision-constant
CFLAGS+=-Wall
CFLAGS+=-Wextra
CFLAGS+=-Wswitch-default
CFLAGS+=-Wswitch-enum
CFLAGS+=-D__$(ARCH)__
CFLAGS+=-DSTM32F4XX
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
LDFLAGS+=-mfloat-abi=hard
LDFLAGS+=-mfpu=fpv4-sp-d16
LDFLAGS+=-mthumb
LDFLAGS+=-T $(LDSCRIPT)
LDFLAGS+=-lsupc++
LDFLAGS+=-lgcc
LDFLAGS+=-lc
LDFLAGS+=-lm
LDFLAGS+=-lg
LDFLAGS+=-g3

INCLUDES+=-Iinclude/$(ARCH) -Isrc/$(ARCH)
