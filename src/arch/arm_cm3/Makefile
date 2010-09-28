CC:=arm-elf-gcc
AS:=arm-elf-gcc
LD:=arm-elf-gcc

MCPU:=cortex-m3
MTUNE:=cortex-m3

LDSCRIPT:=scripts/ld/arm-elf.ld

ifeq ($(DEBUG),1)
CFLAGS:=-mcpu=$(MCPU) -mfpu=vfp -mthumb -msoft-float -gdwarf-2 -x c $(addprefix -D,$(DEF)) -Wall -Wextra
LDFLAGS:=-mcpu=$(MCPU) -mfpu=vfp -mthumb -msoft-float -gdwarf-2 -T $(LDSCRIPT) -lgcc -lc -lm -lnosys -lg
else
DEF+= NDEBUG
CFLAGS:=-mcpu=$(MCPU) -mtune=$(MTUNE) -mthumb -msoft-float -O3 -x c $(addprefix -D,$(DEF)) -Wall -Wextra -fomit-frame-pointer
LDFLAGS:=-mcpu=$(MCPU) -mtune=$(MTUNE) -mthumb -msoft-float -lc -lm -lgcc -T $(LDSCRIPT) -O3
endif

INCLUDES+=-Isrc/rtos/portable/GCC/ARM_CM3
OBJ-PORT+=rtos/portable/GCC/ARM_CM3/port.o
