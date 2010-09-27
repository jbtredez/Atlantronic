CC:=gcc
AS:=gcc
LD:=gcc
DOT:=dot

MARCH ?= core2
SIMU:=1

DEF+=USE_STDIO=1 __GCC_POSIX__=1
LDSCRIPT:=scripts/ld/elf_linux_$(BIT).ld

ifeq ($(DEBUG),1)
CFLAGS:=-march=$(MARCH) -m$(BIT) -g -x c $(addprefix -D,$(DEF)) -Wall -Wextra
LDFLAGS:=-march=$(MARCH) -m$(BIT) -g -T $(LDSCRIPT) -pthread -lrt -ldl
else
DEF+= NDEBUG
CFLAGS:=-march=$(MARCH) -m$(BIT) -O3 -x c $(addprefix -D,$(DEF)) -Wall -Wextra -fomit-frame-pointer
LDFLAGS:=-march=$(MARCH) -m$(BIT) -T $(LDSCRIPT) -O3 -pthread -lrt -ldl
endif

INCLUDES+=-Isrc/rtos/portable/GCC/Posix
OBJ-PORT+=rtos/portable/GCC/Posix/port.o
