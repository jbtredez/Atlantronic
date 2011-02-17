#include "io/systick.h"
#include "module.h"

volatile int32_t systick_last_load_used;
volatile int64_t systick_time;

static int systick_module_init()
{
	systick_last_load_used = SYSTICK_MAXCOUNT - 1;

	// TODO

	return 0;
}

module_init(systick_module_init, INIT_SYSTICK);

int systick_reconfigure(uint64_t tick)
{
	(void) tick;
	// TODO
	return 0;
}
