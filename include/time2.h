#ifndef TIME2_H
#define TIME2_H

//! @file time2.h
//! @brief Time module
//! @author Jean-Baptiste Trédez

//! get sys time since vTaskStartScheduler()
//!
//! @return sys time since vTaskStartScheduler()
extern unsigned long time_sys();

//! get time from the begining of the match
//!
//! @return time from the begining of the match
extern unsigned long time_match();

//! get (from isr) sys time since vTaskStartScheduler()
//!
//! @return sys time since vTaskStartScheduler()
extern unsigned long time_sys_from_isr();

//! get (from isr) time from the begining of the match
//!
//! @return time from the begining of the match
extern unsigned long time_match_from_isr();

//! start match clock
//!
extern void time_start_match();

//! start match clock from isr
//!
extern void time_start_match_from_isr();

#endif
