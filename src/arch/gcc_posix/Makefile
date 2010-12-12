MARCH ?= core2
SIMU:=1

DEF+=USE_STDIO=1 __GCC_POSIX__=1
LDSCRIPT:=scripts/ld/elf_linux_$(BIT).ld

CFLAGS:=-march=$(MARCH)
CFLAGS+=-m$(BIT)
CFLAGS+=-Wall
CFLAGS+=-Wextra

LDFLAGS:=-march=$(MARCH)
LDFLAGS+=-m$(BIT)
LDFLAGS+=-T $(LDSCRIPT)
LDFLAGS+=-pthread
LDFLAGS+=-lrt

ifeq ($(DEBUG),1)
CFLAGS+=-g
LDFLAGS+=-g
else
DEF+= NDEBUG
CFLAGS+=-O3
CFLAGS+=-fomit-frame-pointer
LDFLAGS+=-O3
endif

CFLAGS+=$(addprefix -D,$(DEF))

INCLUDES+=-Isrc/rtos/portable/GCC/Posix
OBJ-PORT+=rtos/portable/GCC/Posix/port.o
