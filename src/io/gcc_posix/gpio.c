#include "io/gpio.h"
#include "module.h"

static int gpio_module_init(void)
{

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);
