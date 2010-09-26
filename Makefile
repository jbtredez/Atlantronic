# @file Makefile
# @author Jean-Baptiste Trédez
#
# TODO : vite fait - à tester et améliorer
#
# fichier de configuration : .cfg
# les fichiers objets *.o sont construits à partir des fichiers *.c ou *.S
#
# fonctionnement :
#   - recherche tout les fichiers build.mk dans les sous dossiers
#   - ces fichiers indiquent les programmes à compiler et comment les compiler :
#       - on suppose qu'on veut compiler le programme foo
#       - BIN += foo
#       - obj-foo += foo.o bar.o foo2.o
#       - lib-foo += -lncurses

# dossiers
src := src
obj := obj
bin := bin
doc := doc

# ajout de la configuration perso si elle existe
-include .cfg

INCLUDES:=-I. -Iinclude -Iinclude/rtos

SIMU ?= 1

DEBUG ?= 1

BIT ?= 32

ifeq ($(SIMU),1)
_obj:=$(obj)/gcc_posix
_bin:=$(bin)
CC:=gcc
AS:=gcc
LD:=gcc
DOT:=dot

MARCH ?= core2

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
else
_obj:=$(obj)/pic32
_bin:=$(bin)/pic32
CC:=wine pic32-gcc
AS:=wine pic32-gcc
LD:=wine pic32-gcc
HEX:=wine pic32-bin2hex

PIC:=32MX795F512L
LDSCRIPT:=scripts/ld/elf32pic32mx.ld
DEF+=MPLAB_PIC32MX_PORT __GCC_PIC32__ NDEBUG
CFLAGS:=-mprocessor=$(PIC) -O3 -x c $(addprefix -D,$(DEF)) -Wall -Wextra -fomit-frame-pointer
ASFLAGS:=-mprocessor=$(PIC) -Wa,--keep-locals,--gdwarf-2
LDFLAGS:=-mprocessor=$(PIC) -T $(LDSCRIPT) -O3 -Wl,--defsym=__MPLAB_BUILD=1,--defsym=_min_heap_size=0,--defsym=_min_heap_size=0,--report-mem

INCLUDES+=-Isrc/rtos/portable/MPLAB/PIC32MX
OBJ-PORT+= rtos/portable/MPLAB/PIC32MX/port.o
OBJ-PORT+= rtos/portable/MPLAB/PIC32MX/port_asm.o
endif

ifneq ($(MAKECMDGOALS),clean)
MK:=$(shell find . -name 'build.mk')
-include $(MK)
endif

SRC_DOC=$(shell find . -name '*.dot')
BIN_DOC=$(SRC_DOC:.dot=.png)

# règles
$(_obj)/%.d: $(_obj)/%.o

$(_obj)/%.o: $(src)/%.c
	@echo [CC] $<
	@mkdir -p `dirname $@`
	@$(CC) $(CFLAGS) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES) || rm -vfr $(@:.o=.d)
	@# conversion des \ en / (format unix) dans les chemins (mais pas en bout de ligne) (compilo gcc modifié par MPLAB - version windows)
	@sed -i 's/\\/\//g;s/ \// \\/g' $(@:.o=.d)

$(_obj)/%.o: $(src)/%.S
	@echo [AS] $<
	@$(AS) $(AFLAGS) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES)

# cibles
ifeq ($(SIMU),1)
all: $(addprefix $(_bin)/,$(BIN))
else
all: $(addprefix $(_bin)/, $(addsuffix .hex, $(BIN)))
endif
.PHONY: all

$(foreach var,$(BIN),$(eval $(_bin)/$(var):$(addprefix $(_obj)/,$(obj-$(var)) )))
$(foreach var,$(BIN),$(eval DEP += $(addprefix $(_obj)/,$(obj-$(var):.o=.d))))

ifneq ($(MAKECMDGOALS),clean)
-include $(DEP)
endif

%.hex: %
	@echo [HEX] $<
	@$(HEX) $<

$(_bin)/%:
	@echo [LD] $@
	@mkdir -p `dirname $@`
	@$(LD) $(LDFLAGS) $($(patsubst $(_bin)/%,lib-%, $@)) $^ -o $@ -Wl,-Map="$@.map"

%.png: %.dot
	@echo [DOT] $@
	@$(DOT) $< -Tpng -o $@

doc: $(BIN_DOC)
	@mkdir -p $(doc)/doxygen
	@doxygen Doxyfile > /dev/null

.PHONY: doc

toutout:
	@+make SIMU=1
	@+make SIMU=0

.PHONY: toutout

clean:
	@rm -frv $(obj)
	@rm -frv $(bin)
	@rm -frv $(doc)/doxygen

.PHONY: clean
