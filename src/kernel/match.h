#ifndef END_H
#define END_H

#include <stdint.h>

extern volatile int match_end;

#define COLOR_UNKNOWN     0
#define COLOR_RED         1
#define COLOR_YELLOW      2

portBASE_TYPE match_go_from_isr(void);
portBASE_TYPE match_set_color_from_isr(void);

#ifndef LINUX
static inline void match_color_change_disable()
{
	extern volatile uint8_t match_color_change_enable;
	match_color_change_enable = 0;
}

static inline uint32_t match_get_color()
{
	extern volatile uint32_t match_color;
	return match_color;
}

static inline uint8_t match_get_go()
{
	extern volatile uint8_t match_go;
	return match_go;
}

static inline uint8_t match_is_go_enable()
{
	extern volatile uint8_t match_enable_go;
	return match_enable_go;
}

void match_wait_go();
#endif
// ---------------- interface usb ------------
enum
{
	MATCH_CMD_ENABLE_GO,
	MATCH_CMD_GO,
};

struct gpio_cmd_go_arg
{
	uint8_t cmd;
};

#endif
