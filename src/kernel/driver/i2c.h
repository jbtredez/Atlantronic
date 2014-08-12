#ifndef I2C_H
#define I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
	I2C_ERROR_NONE      = 0x00,    //!< No error
	I2C_ERROR_BERR      = 0x01,    //!< BERR error
	I2C_ERROR_ARLO      = 0x02,    //!< ARLO error
	I2C_ERROR_AF        = 0x04,    //!< AF error
	I2C_ERROR_OVR       = 0x08,    //!< OVR error
	I2C_ERROR_DMA       = 0x10,    //!< DMA transfer error
	I2C_ERROR_TIMEOUT   = 0x20,    //!< Timeout error
	I2C_ERROR_BUSY      = 0x40,    //!< Busy error
}I2c_error;


I2c_error i2c_transaction(uint16_t i2c_addr, void* _tx_data, uint16_t tx_size, void* _rx_data, uint16_t rx_size, uint32_t timeout);

static inline I2c_error i2c_write_data(uint16_t i2c_addr, void* data, uint16_t size, uint32_t timeout)
{
	return i2c_transaction(i2c_addr, data, size, 0, 0, timeout);
}

#ifdef __cplusplus
}
#endif

#endif
