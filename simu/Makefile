CXX:=g++
CPPFLAGS:=$(shell pkg-config --cflags gtk+-2.0 gthread-2.0 gtkglext-1.0)
LDFLAGS:=$(shell pkg-config --libs gtk+-2.0 gthread-2.0 gtkglext-1.0) -lrt

DEP:=$(shell find . -iname '*.d')

simu: simu.o 3ds.o modele3D.o MatriceHomogene.o Robot.o ArmCm3.o ArmRcc.o CpuEmu.o

$(obj)/$(ARCH)/%.d: $(obj)/$(ARCH)/%.o

%.d: %.o

%.o: %.cxx
	@echo [CXX] $<
	@mkdir -p `dirname $@`
	@$(CXX) $(CPPFLAGS) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES) || rm -vfr $(@:.o=.d)

%:
	@echo [LD] $@
	@$(CXX) -o $@ $^ $(LDFLAGS)

ifneq ($(MAKECMDGOALS),clean)
-include $(DEP)
endif
