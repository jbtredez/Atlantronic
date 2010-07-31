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
#   - en fonction de la variable SIMU, les fichiers dépendants à l'architecture sont ajoutés (variable OBJ).
#

# dossiers
src := src
obj := obj
bin := bin

# ajout de la configuration perso si elle existe
-include .cfg

ifneq ($(MAKECMDGOALS),clean)
MK:=$(shell find . -name 'build.mk')
-include $(MK)
endif

INCLUDES:=-I. -Iinclude -Iinclude/rtos

SIMU ?= 1

DEBUG ?= 0

ifeq ($(SIMU),1)
CC:=gcc
AS:=gcc
LD:=gcc

MARCH ?= core2

DEF+=USE_STDIO=1 __GCC_POSIX__=1
LDSCRIPT:=scripts/elfPC.ld

ifeq ($(DEBUG),1)
CFLAGS:=-march=$(MARCH) -x c $(addprefix -D,$(DEF)) -Wall -Wextra
LDFLAGS:=-march=$(MARCH) -T $(LDSCRIPT) -pthread -lrt -ldl
else
CFLAGS:=-march=$(MARCH) -O3 -x c $(addprefix -D,$(DEF)) -Wall -Wextra -fomit-frame-pointer
LDFLAGS:=-march=$(MARCH) -T $(LDSCRIPT) -O3 -pthread -lrt -ldl
endif


INCLUDES+=-Isrc/rtos/portable/GCC/Posix
OBJ+=rtos/portable/GCC/Posix/port.o
else
CC:=wine pic32-gcc
AS:=wine pic32-gcc
LD:=wine pic32-gcc
HEX:=wine pic32-bin2hex

PIC:=32MX575F256H
LDSCRIPT:=scripts/elf32pic32mx.ld
DEF+=MPLAB_PIC32MX_PORT __GCC_PIC32__
CFLAGS:=-mprocessor=$(PIC) -O3 -x c $(addprefix -D,$(DEF)) -Wall -Wextra -fomit-frame-pointer
ASFLAGS:=-mprocessor=$(PIC) -Wa,--keep-locals,--gdwarf-2
LDFLAGS:=-mprocessor=$(PIC) -T $(LDSCRIPT) -O3 -Wl,--defsym=__MPLAB_BUILD=1,--defsym=_min_heap_size=0,--defsym=_min_heap_size=0

INCLUDES+=-Isrc/rtos/portable/MPLAB/PIC32MX
OBJ+=rtos/portable/MPLAB/PIC32MX/port.o
OBJ+= rtos/portable/MPLAB/PIC32MX/port_asm.o
endif

OBJ:=$(addprefix $(obj)/, $(OBJ))
DEP:=$(OBJ:.o=.d)

# règles
$(obj)/%.d: $(obj)/%.o

$(obj)/%.o: $(src)/%.c
	@echo [CC] $<
	@mkdir -p `dirname $@`
	@$(CC) $(CFLAGS) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES) || rm -vfr $(@:.o=.d)
	@# conversion des \ en / (format unix) dans les chemins (mais pas en bout de ligne) (compilo gcc modifié par MPLAB - version windows)
	@sed -i 's/\\/\//g;s/ \// \\/g' $(@:.o=.d)

$(obj)/%.o: $(src)/%.S
	@echo [AS] $<
	@$(AS) $(AFLAGS) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES)

# cibles
ifeq ($(SIMU),1)
all: $(addprefix $(bin)/,$(BIN))
else
all: $(addprefix $(bin)/, $(addsuffix .hex, $(BIN)))
endif
.PHONY: all

$(foreach var,$(BIN),$(eval $(bin)/$(var):$(addprefix $(obj)/,$(obj-$(var)) ) $(OBJ)))
$(foreach var,$(BIN),$(eval DEP += $(addprefix $(obj)/,$(obj-$(var):.o=.d))))

ifneq ($(MAKECMDGOALS),clean)
-include $(DEP)
endif


%.hex: %
	@echo [HEX] $<
	@$(HEX) $<

$(bin)/%:
	@echo [LD] $@
	@mkdir -p `dirname $@`
	@$(LD) $(LDFLAGS) $^ -o $@ -Wl,-Map="$@.map"

clean:
	@rm -frv $(obj)
	@rm -frv $(bin)

.PHONY: clean
