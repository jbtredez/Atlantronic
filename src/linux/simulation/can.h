#ifndef CAN_H
#define CAN_H

//! @file can.h
//! @brief Module CAN
//! @author Atlantronic

#include "log.h"
#include <stdint.h>
#include "cpu_io_interface.h"

class Can : public CAN_TypeDef, public CpuIoInterface
{
public:
	Can();
	~Can();

	void memory_write(uint64_t offset, uint32_t val);
	uint32_t memory_read(uint64_t offset);

protected:

};

#endif
