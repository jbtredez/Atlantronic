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

#cflags-
