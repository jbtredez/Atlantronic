#ifndef PRIORITY_H
#define PRIORITY_H

//! @file priority.h
//! @brief Tasks priority
//! @author Atlantronic
//!
//! idle task priority : 0

#define PRIORITY_TASK_LOG             1
#define PRIORITY_TASK_STRATEGY        2
#define PRIORITY_TASK_DETECTION       3
#define PRIORITY_TASK_AX12            4
#define PRIORITY_TASK_CAN             5
#define PRIORITY_TASK_CONTROL_PINCE   6
#define PRIORITY_TASK_CONTROL         7
#define PRIORITY_TASK_ADC             8
#define PRIORITY_TASK_END             9

// priority : 0 ... configMAX_PRIORITIES-1
#define configMAX_PRIORITIES		  10

#endif
