//! @file model.h
//! @brief Model of the robot
//! @author Jean-Baptiste Trédez

#include <stdint.h>

void model_pwm_set(unsigned int num, uint32_t val, int dir);

uint16_t model_encoders_get(unsigned int num);

void model_update();
