#include "io/usart.h"
#include "module.h"

static int usart_module_init(void)
{
	return 0;
}

module_init(usart_module_init, INIT_USART);

void usart_write(unsigned char* buf, uint16_t size)
{
	(void) buf;
	(void) size;
	// TODO
}
