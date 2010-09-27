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

DOT:=dot

ARCH ?= gcc_posix
#ARCH:=pic32

DEBUG ?= 1

BIT ?= 32

SIMU:=0
include src/arch/$(ARCH)/Makefile

ifneq ($(MAKECMDGOALS),clean)
MK:=$(shell find . -name 'build.mk')
-include $(MK)
endif

SRC_DOC=$(shell find . -name '*.dot')
BIN_DOC=$(SRC_DOC:.dot=.png)

# règles
$(obj)/$(ARCH)/%.d: $(obj)/$(ARCH)/%.o

$(obj)/$(ARCH)/%.o: $(src)/%.c
	@echo [CC] $<
	@mkdir -p `dirname $@`
	@$(CC) $(CFLAGS) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES) || rm -vfr $(@:.o=.d)
	@# conversion des \ en / (format unix) dans les chemins (mais pas en bout de ligne) (compilo gcc modifié par MPLAB - version windows)
	@sed -i 's/\\/\//g;s/ \// \\/g' $(@:.o=.d)

$(obj)/$(ARCH)/%.o: $(src)/%.S
	@echo [AS] $<
	@$(AS) $(AFLAGS) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES)

# cibles
ifeq ($(SIMU),1)
all: $(addprefix $(bin)/$(ARCH)/,$(BIN))
else
all: $(addprefix $(bin)/$(ARCH)/, $(addsuffix .hex, $(BIN)))
endif
.PHONY: all

$(foreach var,$(BIN),$(eval $(bin)/$(ARCH)/$(var):$(addprefix $(obj)/$(ARCH)/,$(obj-$(var)) )))
$(foreach var,$(BIN),$(eval DEP += $(addprefix $(obj)/$(ARCH)/,$(obj-$(var):.o=.d))))

ifneq ($(MAKECMDGOALS),clean)
-include $(DEP)
endif

%.hex: %
	@echo [HEX] $<
	@$(HEX) $<

$(bin)/$(ARCH)/%:
	@echo [LD] $@
	@mkdir -p `dirname $@`
	@$(LD) $(LDFLAGS) $($(patsubst $(bin)/$(ARCH)/%,lib-%, $@)) $^ -o $@ -Wl,-Map="$@.map"

%.png: %.dot
	@echo [DOT] $@
	@$(DOT) $< -Tpng -o $@

doc: $(BIN_DOC)
	@mkdir -p $(doc)/doxygen
	@doxygen Doxyfile > /dev/null

.PHONY: doc

toutout:
	@+make ARCH=gcc_posix
	@+make ARCH=pic32

.PHONY: toutout

clean:
	@rm -frv $(obj)
	@rm -frv $(bin)
	@rm -frv $(doc)/doxygen

.PHONY: clean
