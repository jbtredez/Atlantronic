CROSSCOMPILE:=arm-none-eabi-

MCPU:=cortex-m4
HW_FPU=1

LDSCRIPT:=scripts/ld/stm32f429.ld

CFLAGS:=-mcpu=$(MCPU)
ifeq ($(HW_FPU),1)
CFLAGS+=-mfloat-abi=hard
CFLAGS+=-mfpu=fpv4-sp-d16
else
CFLAGS+=-msoft-float
CFLAGS+=-mfpu=vfp
endif
CFLAGS+=-mthumb
CFLAGS+=-mno-thumb-interwork
CFLAGS+=-fomit-frame-pointer
CFLAGS+=-fno-exceptions
CFLAGS+=-fno-non-call-exceptions
CFLAGS+=-fsingle-precision-constant
CFLAGS+=-ffast-math
CFLAGS+=-fno-stack-protector
CFLAGS+=-Wall
CFLAGS+=-Wextra
CFLAGS+=-Wswitch-default
CFLAGS+=-Wswitch-enum
CFLAGS+=-D__$(ARCH)__
CFLAGS+=-DARCH=\"$(ARCH)\"
CFLAGS+=-DSTM32F4XX
CFLAGS+=-DSTM32F429xx
CFLAGS+=-DSTM32F4_USB_HS
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
ifeq ($(HW_FPU),1)
LDFLAGS+=-mfloat-abi=hard
LDFLAGS+=-mfpu=fpv4-sp-d16
else
LDFLAGS+=-msoft-float
LDFLAGS+=-mfpu=vfp
endif
LDFLAGS+=-mthumb
LDFLAGS+=-T $(LDSCRIPT)
LDFLAGS+=-lsupc++
LDFLAGS+=-lgcc
LDFLAGS+=-lc
LDFLAGS+=-lm
LDFLAGS+=-lg
LDFLAGS+=-g3

INCLUDES+=-Iinclude/$(ARCH) -Isrc/$(ARCH)
