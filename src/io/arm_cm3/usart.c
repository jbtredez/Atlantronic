#include "io/usart.h"
#include "module.h"

static int usart_module_init(void)
{
	// USART3 (remapage partiel) => Tx = PC10, Rx = PC11
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	// remap PC10 / PC11
	AFIO->MAPR &= ~AFIO_MAPR_USART3_REMAP;
	AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_PARTIALREMAP;

	// GPIOC utilisee, configuration de PC10 et PC11
	RCC->APB2ENR |=  RCC_APB2ENR_IOPCEN;
	GPIOC->CRH   &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11 | GPIO_CRH_MODE10 | GPIO_CRH_CNF10);     // on efface la conf de PC10 et PC11
	GPIOC->CRH   |=  GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1;               	// Tx = PC10 : alternate output push-pull, 50MHz
	GPIOC->CRH   |=  GPIO_CRH_CNF11_0;                                                          // Rx = PC11 : input floating

	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // usart3 clock enable

	// TODO : BRR
//	USART3->BRR = ; // USARTDIV : selon baud rate

	USART3->CR1 = 0x00;
//	USART3->CR1 |= USART_CR1_PEIE; // TODO : controle de parité ? (+ voir USART_CR1_PS si impaire)
	USART3->CR1 |= USART_CR1_TXEIE;// | USART_CR1_PEIE; // TODO : voir / interruption sur transmit data buffer empty et erreur parité
	USART3->CR2 = 0; // TODO : voir / stop bit (	USART_CR2_STOP_0 et 	USART_CR2_STOP_1)

	USART3->CR3 = 0;
//	USART3->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT; // TODO : voir / DMA

	USART3->CR1 |= (USART_CR1_RE | USART_CR1_TE);  // activation Rx et tx

	USART3->CR1 |= USART_CR1_UE;

	NVIC_EnableIRQ(USART3_IRQn);

	return 0;
}

module_init(usart_module_init, INIT_USART);

void isr_usart3(void)
{
	// TODO
}
