#ifndef PRIORITY_H
#define PRIORITY_H

//! @file priority.h
//! @brief Tasks priority
//! @author Atlantronic
//!
//! idle task priority : 0

#define PRIORITY_TASK_FAULT           1
#define PRIORITY_TASK_USB             2
#define PRIORITY_TASK_STRATEGY        3
#define PRIORITY_TASK_TRAJECTORY      4
#define PRIORITY_TASK_HOKUYO          5
#define PRIORITY_TASK_DETECTION       5
#define PRIORITY_TASK_AX12            6
#define PRIORITY_TASK_CAN             7
#define PRIORITY_TASK_CONTROL         8
#define PRIORITY_TASK_ADC             9
#define PRIORITY_TASK_END            10

// priority : 0 ... configMAX_PRIORITIES-1
#define configMAX_PRIORITIES		  11

// attention, les priorités sont codées sur 4 bits (les bits de poids fort).
// La fonction setpriority s'occupe de mettre la priorité comme il faut.
// La priorite 15 est reservee pour l'os

// priorites des IT NON bloquees par l'os

// priorites des IT bloquees par l'os
#define PRIORITY_IRQ_SYSCALL          8
#define PRIORITY_IRQ_EXTI1            9
#define PRIORITY_IRQ_EXTI3            9
#define PRIORITY_IRQ_EXTI9_5          9
#define PRIORITY_IRQ_EXTI15_10        9
#define PRIORITY_IRQ_CAN1_RX0        10
#define PRIORITY_IRQ_CAN1_TX         11
#define PRIORITY_IRQ_USART3          12
#define PRIORITY_IRQ_DMA1_CHANNEL1   12
#define PRIORITY_IRQ_DMA1_CHANNEL2   12
#define PRIORITY_IRQ_DMA1_CHANNEL3   12
#define PRIORITY_IRQ_UART4           12
#define PRIORITY_IRQ_DMA2_CHANNEL3   12
#define PRIORITY_IRQ_DMA2_CHANNEL5   12
#define PRIORITY_IRQ_USB             14
#define PRIORITY_IRQ_KERNEL          15
#define PRIORITY_IRQ_PENDSV          PRIORITY_IRQ_KERNEL
#define PRIORITY_IRQ_SYSTICK         PRIORITY_IRQ_KERNEL

// dans FreeRtos, on utilise directement l'octet de priorité
#define configKERNEL_INTERRUPT_PRIORITY			(PRIORITY_IRQ_KERNEL*16)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY	(PRIORITY_IRQ_SYSCALL*16)

#endif
