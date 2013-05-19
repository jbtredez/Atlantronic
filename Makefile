# @file Makefile
# @author Jean-Baptiste Trédez
#
# fichier de configuration : .cfg
# les fichiers objets *.o sont construits à partir des fichiers *.c ou *.S
#
# fonctionnement :
#   - recherche tout les fichiers build.mk dans les sous dossiers
#   - ces fichiers indiquent les programmes à compiler et comment les compiler :
#       - on suppose qu'on veut compiler le programme homologation de la carte foo
#       - bin-foo += homologation
#       - obj-foo-homologation += x.o y.o z.o
#       - lib-foo-homologation += -lncurses

# pour le module noyau
export INSTALL_MOD_PATH:=$(DESTDIR)

# dossiers
src := src
obj := obj
bin := bin
doc := doc

# ajout de la configuration perso si elle existe
-include .cfg

INCLUDES:=-I. -Iinclude -Isrc

DEBUG ?= 0
VERBOSE ?= 0

ifeq ($(MAKECMDGOALS),foo)
ARCH=foo
endif

ifeq ($(MAKECMDGOALS),bar)
ARCH=bar
endif

ifeq ($(MAKECMDGOALS),linux)
ARCH=linux
endif

ifeq ($(VERBOSE),0)
V:=@
endif

-include src/$(ARCH)/Makefile

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
	@echo [CC] $@
	@mkdir -p `dirname $@`
	$(V)$(CC) $(CFLAGS) $($(patsubst $(obj)/$(ARCH)/%,cflags-$(ARCH)-%, $@)) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES) || ( rm -vfr $@ $(@:.o=.d) ; exit 1 )

$(obj)/$(ARCH)/%.o: $(src)/%.cxx
	@echo [CXX] $@
	@mkdir -p `dirname $@`
	$(V)@$(CXX) $(CXXFLAGS) $($(patsubst $(obj)/$(ARCH)/%,cxxflags-$(ARCH)-%, $@)) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES) || ( rm -vfr $@ $(@:.o=.d) ; exit 1 )

$(obj)/$(ARCH)/%.o: $(src)/%.S
	@echo [AS] $@
	$(V)@$(AS) $(AFLAGS) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES)

%.pdf: %.tex
	pdflatex -output-directory doc $<
	pdflatex -output-directory doc $<

# cibles
# cible par defaut :
ifneq ($(ARCH),)
$(ARCH): $(addprefix $(bin)/$(ARCH)/,$(bin-$(ARCH)))

.PHONY: $(ARCH)
endif

all:
	@+make --no-print-directory ARCH=foo
	@+make --no-print-directory ARCH=bar
	@+make --no-print-directory ARCH=linux

.PHONY: all

modules:
	@+make --no-print-directory -C /lib/modules/`uname -r`/build M=`pwd`/src/linux/modules

.PHONY: modules

clean_modules:
	@+make --no-print-directory -C /lib/modules/`uname -r`/build M=`pwd`/src/linux/modules clean

.PHONY: clean_modules

install: modules
	@echo "  INSTALL $(shell pwd)/scripts/udev/65-atlantronic.rules"
	@+install -D -m 644 -o root -g root -p scripts/udev/65-atlantronic.rules $(DESTDIR)/etc/udev/rules.d/65-atlantronic.rules
	@+make --no-print-directory -C /lib/modules/`uname -r`/build M=`pwd`/src/linux/modules modules_install

qemu:
	@cd qemu && ./configure --target-list=arm-softmmu --disable-docs
	@+make -C qemu

.PHONY: qemu

$(foreach var,$(bin-$(ARCH)),$(eval $(bin)/$(ARCH)/$(var):$(addprefix $(obj)/$(ARCH)/,$(obj-$(ARCH)-$(var)) )))
$(foreach var,$(bin-$(ARCH)),$(eval DEP += $(addprefix $(obj)/$(ARCH)/,$(obj-$(ARCH)-$(var):.o=.d))))

ifneq ($(MAKECMDGOALS),clean)
-include $(DEP)
endif

$(bin)/$(ARCH)/%:
	@echo [LD] $@
	@mkdir -p `dirname $@`
	$(V)$(CC) $^ -o $@ $($(patsubst $(bin)/$(ARCH)/%,lib-$(ARCH)-%, $@)) -Wl,-Map="$@.map" $(LDFLAGS)
	$(V)$(OBJCOPY) --only-keep-debug $@ $@.debug
	$(V)$(OBJCOPY) --add-gnu-debuglink $@.debug $@
	$(V)$(STRIP) $@

%.png: %.dot
	@echo [DOT] $@
	$(V)$(DOT) $< -Tpng -o $@

dot: $(BIN_DOC)

.PHONY: dot

pdf: $(doc)/atlantronic.pdf

.PHONY: pdf

TEXSRC:= $(shell find $(doc)/ -name "*.tex")

doc/atlantronic.pdf: $(TEXSRC)

doc: dot pdf
	@mkdir -p $(doc)/doxygen
	@doxygen Doxyfile > /dev/null

.PHONY: doc

clean:
	@rm -frv $(obj)
	@rm -frv $(bin)
	@rm -frv $(doc)/doxygen
	@rm -fv $(doc)/*.pdf
	@rm -fv $(doc)/*.dvi
	@rm -fv $(doc)/*.aux
	@rm -fv $(doc)/*.ps
	@rm -fv $(doc)/*.log
	@rm -fv $(doc)/*.toc

	@find . -name \*~ -exec rm \-fv {} \;

.PHONY: clean
