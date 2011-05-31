#ifndef PRIORITY_H
#define PRIORITY_H

//! @file priority.h
//! @brief Tasks priority
//! @author Atlantronic
//!
//! idle task priority : 0

#define PRIORITY_TASK_LOG             1
#define PRIORITY_TASK_HOKUYO          2
#define PRIORITY_TASK_CAN_US          3
#define PRIORITY_TASK_CAN             4
#define PRIORITY_TASK_TEST_US         4
#define PRIORITY_TASK_END             5

// priority : 0 ... configMAX_PRIORITIES-1
#define configMAX_PRIORITIES		  6

#endif
