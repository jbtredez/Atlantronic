# @file Makefile
# @author Jean-Baptiste Trédez
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

ARCH ?= arm_cm3

DEBUG ?= 1

# on tente de détecter la configuration native
BIT ?= $(shell getconf LONG_BIT)

SIMU:=0
include src/arch/$(ARCH)/Makefile

AS:=$(CROSSCOMPILE)as
CC:=$(CROSSCOMPILE)gcc
CXX:=$(CROSSCOMPILE)g++
OBJCOPY:=$(CROSSCOMPILE)objcopy
STRIP:=$(CROSSCOMPILE)strip
DOT:=dot

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
	@$(CC) $(CFLAGS) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES) || ( rm -vfr $@ $(@:.o=.d) ; exit 1 )

$(obj)/$(ARCH)/%.o: $(src)/%.cxx
	@echo [CXX] $<
	@mkdir -p `dirname $@`
	@$(CXX) $(CXXFLAGS) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES) || ( rm -vfr $@ $(@:.o=.d) ; exit 1 )

$(obj)/$(ARCH)/%.o: $(src)/%.S
	@echo [AS] $<
	@$(AS) $(AFLAGS) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES)

# cibles
# cible par defaut :
all-$(ARCH):

all:
	@+make ARCH=gcc_posix all-gcc_posix
	@+make ARCH=arm_cm3 all-arm_cm3

.PHONY: all

all-$(ARCH): $(addprefix $(bin)/$(ARCH)/,$(BIN))

.PHONY: all-$(ARCH)

$(foreach var,$(BIN),$(eval $(bin)/$(ARCH)/$(var):$(addprefix $(obj)/$(ARCH)/,$(obj-$(var)) )))
$(foreach var,$(BIN),$(eval DEP += $(addprefix $(obj)/$(ARCH)/,$(obj-$(var):.o=.d))))

ifneq ($(MAKECMDGOALS),clean)
-include $(DEP)
endif

$(bin)/$(ARCH)/%:
	@echo [LD] $@
	@mkdir -p `dirname $@`
	@$(CC) $^ -o $@ $($(patsubst $(bin)/$(ARCH)/%,lib-%, $@)) -Wl,-Map="$@.map" $(LDFLAGS)
	@$(OBJCOPY) --only-keep-debug $@ $@.debug
	@$(OBJCOPY) --add-gnu-debuglink $@.debug $@
	@$(STRIP) $@

%.png: %.dot
	@echo [DOT] $@
	@$(DOT) $< -Tpng -o $@

doc: $(BIN_DOC)
	@mkdir -p $(doc)/doxygen
	@doxygen Doxyfile > /dev/null

.PHONY: doc

clean:
	@rm -frv $(obj)
	@rm -frv $(bin)
	@rm -frv $(doc)/doxygen
	@find . -name \*~ -exec rm \-fv {} \;

.PHONY: clean
