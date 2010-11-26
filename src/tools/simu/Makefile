CXX:=g++
CPPFLAGS:=$(shell pkg-config --cflags gtk+-2.0 gthread-2.0 gtkglext-1.0)
LDFLAGS:=$(shell pkg-config --libs gtk+-2.0 gthread-2.0 gtkglext-1.0) -lrt

simu: simu.o 3ds.o modele3D.o

%.o: %.cxx
	$(CXX) -o $@ -c $< $(CPPFLAGS)

%:
	$(CXX) -o $@ $^ $(LDFLAGS)

