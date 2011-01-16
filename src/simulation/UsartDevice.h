#ifndef USART_DEVICE_H
#define USART_DEVICE_H

//! @file UsartDevice.h
//! @brief UsartDevice
//! @author Jean-Baptiste Trédez

class UsartDevice
{
	public:
		UsartDevice(){};
		virtual ~UsartDevice(){};
		virtual void usart_read(unsigned char octet) = 0;
};

#endif
