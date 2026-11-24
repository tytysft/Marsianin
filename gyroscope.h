#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <stdint.h>

#define MPU6050_ADDR       0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_X_H  0x3B

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} gyro_data_t;

void gyro_init(void);
void gyro_read_raw(gyro_data_t *data);

#endif
