#ifndef PRIORITY_H
#define PRIORITY_H

//! @file priority.h
//! @brief Tasks priority
//! @author Jean-Baptiste Trédez
//!
//! idle task priority : 0

#define PRIORITY_TASK_LOG             1
#define PRIORITY_TASK_STRATEGY        2
#define PRIORITY_TASK_AX12            3
#define PRIORITY_TASK_CONTROL         4
#define PRIORITY_TASK_END             5

// priority : 0 ... configMAX_PRIORITIES-1
#define configMAX_PRIORITIES			( ( unsigned portBASE_TYPE ) 6 )

#endif
