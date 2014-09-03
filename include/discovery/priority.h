#ifndef PRIORITY_H
#define PRIORITY_H

//! @file priority.h
//! @brief Tasks priority
//! @author Atlantronic
//!
//! idle task priority : 0

//! une priorite differente par tache pour eviter les pb
enum
{
	PRIORITY_IDLE,
	PRIORITY_TASK_LED,
	PRIORITY_TASK_XBEE,
	PRIORITY_TASK_STRATEGY,
	PRIORITY_TASK_PINCE,
	PRIORITY_TASK_TRAJECTORY,
	PRIORITY_TASK_FAULT,
	PRIORITY_TASK_USB,
	PRIORITY_TASK_HOKUYO,
	PRIORITY_TASK_DETECTION,
	PRIORITY_TASK_ARM,
	PRIORITY_TASK_DYNAMIXEL,
	PRIORITY_TASK_ACCELERO,
	PRIORITY_TASK_GYRO,
	PRIORITY_TASK_CAN,
	PRIORITY_TASK_CONTROL,
	PRIORITY_TASK_ADC,
	PRIORITY_TASK_END,
	configMAX_PRIORITIES
};

// attention, les priorités sont codées sur 4 bits (les bits de poids fort).
// La fonction setpriority s'occupe de mettre la priorité comme il faut.
// La priorite 15 est reservee pour l'os

// priorites des IT NON bloquees par l'os

// priorites des IT bloquees par l'os
#define PRIORITY_IRQ_SYSCALL          5
#define PRIORITY_IRQ_EXTI0            9
#define PRIORITY_IRQ_EXTI1            9
#define PRIORITY_IRQ_EXTI2            9
#define PRIORITY_IRQ_EXTI3            9
#define PRIORITY_IRQ_EXTI4            9
#define PRIORITY_IRQ_EXTI9_5          9
#define PRIORITY_IRQ_EXTI15_10        9
#define PRIORITY_IRQ_CAN1_SCE        10
#define PRIORITY_IRQ_CAN1_RX0        10
#define PRIORITY_IRQ_CAN1_TX         11
#define PRIORITY_IRQ_USART2          12
#define PRIORITY_IRQ_DMA1_STREAM5    12
#define PRIORITY_IRQ_DMA1_STREAM6    12
#define PRIORITY_IRQ_USART3          12
#define PRIORITY_IRQ_DMA1_STREAM1    12
#define PRIORITY_IRQ_DMA1_STREAM3    12
#define PRIORITY_IRQ_UART4           12
#define PRIORITY_IRQ_DMA1_STREAM2    12
#define PRIORITY_IRQ_DMA1_STREAM4    12
#define PRIORITY_IRQ_UART5           12
#define PRIORITY_IRQ_DMA1_STREAM0    12
#define PRIORITY_IRQ_DMA1_STREAM7    12
#define PRIORITY_IRQ_USART6          12
#define PRIORITY_IRQ_DMA2_STREAM1    12
#define PRIORITY_IRQ_DMA2_STREAM6    12
#define PRIORITY_IRQ_SPI             12
#define PRIORITY_IRQ_DMA2_STREAM0    12
#define PRIORITY_IRQ_DMA2_STREAM3    12
#define PRIORITY_IRQ_DMA2_STREAM4    12
#define PRIORITY_IRQ_USB             14
#define PRIORITY_IRQ_KERNEL          15
#define PRIORITY_IRQ_PENDSV          PRIORITY_IRQ_KERNEL
#define PRIORITY_IRQ_SYSTICK         PRIORITY_IRQ_KERNEL

// dans FreeRtos, on utilise directement l'octet de priorité
#define configKERNEL_INTERRUPT_PRIORITY			(PRIORITY_IRQ_KERNEL*16)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY	(PRIORITY_IRQ_SYSCALL*16)

#endif
