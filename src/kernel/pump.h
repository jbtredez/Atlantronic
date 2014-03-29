#ifndef PUMP_H
#define PUMP_H

//! @file pump.h
//! @brief Pump
//! @author Atlantronic

#include <stdint.h>
#include "kernel/asm/asm_base_func.h"

#ifndef WEAK_PUMP
#define WEAK_PUMP __attribute__((weak, alias("nop_function") ))
#endif

enum
{
	PUMP_1,
	PUMP_2,
	PUMP_3,
	PUMP_4,
	PUMP_MAX
};

class Pump
{
	public:
		Pump(uint8_t Pwm_id)
		{
			pwm_id = Pwm_id;
			val = 0;
		}

		void set(float percent);

		void update();

	protected:
		uint8_t pwm_id;
		float val;
};

void pump_update() WEAK_PUMP;

//------------------ interface usb -------------------
struct pump_cmd_arg
{
	uint8_t id;         //!< id de la pompe
	uint8_t val;        //!< puissance de 0 a 100
} __attribute__((packed));

#endif
