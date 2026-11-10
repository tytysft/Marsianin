#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include <avr/io.h>

void i2c_master_init(void);
void i2c_master_sendByte(uint8_t address, uint8_t data);
void i2c_master_sendBytes(uint8_t address, uint8_t* data, uint8_t length);
void i2c_master_readByte(uint8_t address, uint8_t* data);

#endif
