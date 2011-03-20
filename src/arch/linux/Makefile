MARCH ?= core2

DEF+=LINUX
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
CXXFLAGS:=$(CFLAGS)
