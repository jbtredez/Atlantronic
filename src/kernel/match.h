#ifndef MATCH_H
#define MATCH_H

#include <stdint.h>

extern volatile int match_end;

#define COLOR_PURPLE     -1     //!< couleur avec x negatif (=> -1)
//#define COLOR_UNKNOWN     0     //!< couelur inconnue (non choisie)
#define COLOR_GREEN       1     //!< couleur avec x positif (=> 1)

portBASE_TYPE match_go_from_isr(void);
portBASE_TYPE match_set_color_from_isr(void);
portBASE_TYPE match_set_strat_isr(void);

static inline int match_get_color()
{
	extern volatile int match_color;
	return match_color;
}

static inline uint8_t match_get_go()
{
	extern volatile uint8_t match_go;
	return match_go;
}

void match_wait_go();

// ---------------- interface usb ------------
enum
{
	MATCH_CMD_RECALAGE,
	MATCH_CMD_ENABLE_GO,
	MATCH_CMD_GO,
};

struct gpio_cmd_match_arg
{
	uint8_t cmd;
};

#endif
