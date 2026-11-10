#include <avr/io.h>
#include "i2c_master.h"

void i2c_master_init(void) {
    TWBR = 72;
    TWSR = 0;
    TWCR = (1 << TWEN);
}

void i2c_master_sendByte(uint8_t address, uint8_t data) {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    
    TWDR = (address << 1);
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}

void i2c_master_sendBytes(uint8_t address, uint8_t* data, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        i2c_master_sendByte(address, data[i]);
    }
}

void i2c_master_readByte(uint8_t address, uint8_t* data) {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    
    TWDR = (address << 1) | 1;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    
    *data = TWDR;
    
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}
