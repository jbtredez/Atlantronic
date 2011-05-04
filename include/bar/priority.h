#ifndef PRIORITY_H
#define PRIORITY_H

//! @file priority.h
//! @brief Tasks priority
//! @author Jean-Baptiste Trédez
//!
//! idle task priority : 0

#define PRIORITY_TASK_LOG             1
#define PRIORITY_TASK_HOKUYO          2
#define PRIORITY_TASK_END             3

// priority : 0 ... configMAX_PRIORITIES-1
#define configMAX_PRIORITIES		  4

#endif
