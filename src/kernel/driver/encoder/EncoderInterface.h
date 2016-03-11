#ifndef ENCODER_INTERFACE_H
#define ENCODER_INTERFACE_H

class EncoderInterface
{
	public:
		virtual void update(float dt) = 0;
		virtual float getPosition() = 0;
		virtual float getSpeed() = 0;
};

#endif
