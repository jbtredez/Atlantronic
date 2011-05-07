#ifndef USART_DEVICE_H
#define USART_DEVICE_H

//! @file UsartDevice.h
//! @brief UsartDevice
//! @author Atlantronic

class UsartDevice
{
	public:
		UsartDevice(){};
		virtual ~UsartDevice(){};
		virtual void usart_read(uint8_t octet) = 0;
		virtual uint8_t usart_write() = 0;
		virtual bool usart_write_request() = 0;
};

#endif
