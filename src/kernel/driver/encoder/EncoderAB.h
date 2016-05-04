#ifndef ENCODER_AB_H
#define ENCODER_AB_H

//! @file encoder.h
//! @brief Encoder
//! @author Atlantronic

#include <stdint.h>
#include "EncoderInterface.h"

enum
{
	ENCODER_1 = 0,
	ENCODER_2,
	ENCODER_3,
	ENCODER_MAX
};

uint16_t encoder_ab_get(const unsigned int id);

class EncoderAB : public EncoderInterface
{
	public:
		void init(int id, float outputFactor);
		void update(float dt);
		float getPosition();
		float getSpeed();
		float getOutputFactor(){return m_outputFactor;};
		void  setOutputFactor(float outputFactor){m_outputFactor = outputFactor;};
	protected:
		int m_id;
		float m_outputFactor;

		uint16_t m_lastRawPos;
		uint16_t m_rawPos;
		float m_speed;
};

#endif
