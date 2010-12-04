//! @file encoders.c
//! @brief Encoders
//! @author Jean-Baptiste Trédez

#include "io/encoders.h"
#include "module.h"
#include "init.h"
#include "simu/model.h"

uint16_t encoders_get(unsigned int num)
{
	return model_encoders_get(num);
}

