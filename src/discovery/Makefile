CROSSCOMPILE:=arm-none-eabi-

MCPU=cortex-m4

LDSCRIPT:=scripts/ld/stm32f407.ld

CFLAGS:=-mcpu=$(MCPU)
CFLAGS+=-mfloat-abi=hard
CFLAGS+=-fsingle-precision-constant
CFLAGS+=-mfpu=fpv4-sp-d16
CFLAGS+=-mthumb
CFLAGS+=-x c
CFLAGS+=-Wall
CFLAGS+=-Wextra
CFLAGS+=-Wswitch-default
CFLAGS+=-Wswitch-enum
CFLAGS+=-Wunsuffixed-float-constants
CFLAGS+=-mno-thumb-interwork
CFLAGS+=-fomit-frame-pointer
CFLAGS+=-g3

LDFLAGS:=-mcpu=$(MCPU)
LDFLAGS+=-nostartfiles
LDFLAGS+=-mfloat-abi=hard
LDFLAGS+=-mfpu=fpv4-sp-d16
LDFLAGS+=-mthumb
LDFLAGS+=-T $(LDSCRIPT)
LDFLAGS+=-lgcc
LDFLAGS+=-lc
LDFLAGS+=-lm
#LDFLAGS+=-lnosys
LDFLAGS+=-lg
LDFLAGS+=-g3

DEF+=__$(ARCH)__
DEF+=STM32F4XX

ifneq ($(DEBUG),1)
DEF+= NDEBUG
CFLAGS+=-O2
LDFLAGS+=-O2
endif

CFLAGS+=$(addprefix -D,$(DEF))

INCLUDES+=-Iinclude/$(ARCH) -Isrc/$(ARCH)

