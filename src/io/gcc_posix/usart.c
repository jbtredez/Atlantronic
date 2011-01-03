#include "io/usart.h"
#include "module.h"

static int usart_module_init(void)
{
	return 0;
}

module_init(usart_module_init, INIT_USART);
