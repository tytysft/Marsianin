#include "gyroscope.h"
#include "i2c_master.h"

static uint8_t mpu_read_reg(uint8_t reg) {
    uint8_t val = 0;
    i2c_master_sendByte(MPU6050_ADDR, reg);
    i2c_master_readByte(MPU6050_ADDR, &val);
    return val;
}

static int16_t mpu_read_word(uint8_t reg_high) {
    uint8_t high = mpu_read_reg(reg_high);
    uint8_t low  = mpu_read_reg(reg_high + 1);
    return (int16_t)((high << 8) | low);
}

void gyro_init(void) {
    i2c_master_writeReg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
}

void gyro_read_raw(gyro_data_t *data) {
    data->accel_x = mpu_read_word(0x3B);
    data->accel_y = mpu_read_word(0x3D);
    data->accel_z = mpu_read_word(0x3F);
    data->temp    = mpu_read_word(0x41);
    data->gyro_x  = mpu_read_word(0x43);
    data->gyro_y  = mpu_read_word(0x45);
    data->gyro_z  = mpu_read_word(0x47);
}

