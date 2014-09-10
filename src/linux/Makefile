MARCH ?= core2

# on tente de détecter la configuration native
BIT ?= $(shell getconf LONG_BIT)

DEF+=LINUX
LDSCRIPT:=scripts/ld/elf_linux_$(BIT).ld

CFLAGS:=-march=$(MARCH)
CFLAGS+=-m$(BIT)
CFLAGS+=-Wall
CFLAGS+=-Wextra
CFLAGS+=-g

LDFLAGS:=-march=$(MARCH)
LDFLAGS+=-m$(BIT)
LDFLAGS+=-T $(LDSCRIPT)
LDFLAGS+=-pthread
LDFLAGS+=-lrt
DEF+=__disco__

ifneq ($(DEBUG),1)
DEF+= NDEBUG
CFLAGS+=-O2
CFLAGS+=-fomit-frame-pointer
endif

CFLAGS+=$(addprefix -D,$(DEF))
CXXFLAGS:=$(CFLAGS)

INCLUDES+=-Iinclude/discovery -Iinclude/foo -Isrc/linux -Isrc/kernel -Isrc/discovery -Isrc/foo -Isrc/bar
