#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"
#include <stdbool.h>
#include <stdint.h>

// MPU-6050 I2C address
#define MPU6050_ADDR 0x68 // 0x69 if AD0 = 1

// Register addresses
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44

// Conversion constants
#define CONVERT_ACCEL_2G 16384
#define CONVERT_ACCEL_4G 8192
#define CONVERT_ACCEL_8G 4096
#define CONVERT_ACCEL_16G 2048

#define CONVERT_GYRO_250DPS 131.0
#define CONVERT_GYRO_500DPS 65.5
#define CONVERT_GYRO_1000DPS 32.8
#define CONVERT_GYRO_2000DPS 16.4

#define BIAS_SAMPLE_COUNT 1000

// MPU6050 Address Enum
typedef enum {
  MPU6050_ADDRESS_0 = 0x68,
  MPU6050_ADDRESS_1 = 0x69
} MPU6050_Address;

// Accelerometer scale enum
typedef enum {
  MPU6050_ACCEL_SCALE_2G = 0,
  MPU6050_ACCEL_SCALE_4G = 1,
  MPU6050_ACCEL_SCALE_8G = 2,
  MPU6050_ACCEL_SCALE_16G = 3
} MPU6050_AccelScale;

// Gyroscope scale enum
typedef enum {
  MPU6050_GYRO_SCALE_250DPS = 0,
  MPU6050_GYRO_SCALE_500DPS = 1,
  MPU6050_GYRO_SCALE_1000DPS = 2,
  MPU6050_GYRO_SCALE_2000DPS = 3
} MPU6050_GyroScale;

// Operational state enum
typedef enum { MPU6050_STATE_SLEEP = 0, MPU6050_STATE_CYCLE } MPU6050_StateEnum;

// Low Power Wake Control frequency enum
typedef enum {
  MPU6050_WAKE_FREQ_1_25HZ = 0,
  MPU6050_WAKE_FREQ_5HZ = 1,
  MPU6050_WAKE_FREQ_20HZ = 2,
  MPU6050_WAKE_FREQ_40HZ = 3
} MPU6050_LowPowerWakeFreq;

// MPU6050 state structure
typedef struct {
  MPU6050_Address address;
  MPU6050_AccelScale accel_scale;
  MPU6050_GyroScale gyro_scale;
  MPU6050_StateEnum state;
  MPU6050_LowPowerWakeFreq lp_wake_ctrl;
  bool standby[6]; // [XA, YA, ZA, XG, YG, ZG]
  float accel_bias_x, accel_bias_y, accel_bias_z;
  float gyro_bias_x, gyro_bias_y, gyro_bias_z;
} MPU6050_State;

// Function prototypes
esp_err_t mpu6050_init(MPU6050_State *state);
esp_err_t mpu6050_write(MPU6050_State *state, uint8_t reg, uint8_t data);
esp_err_t mpu6050_read(MPU6050_State *state, uint8_t reg, uint8_t *data,
                       size_t len);
esp_err_t mpu6050_set_accel_scale(MPU6050_State *state,
                                  MPU6050_AccelScale scale);
MPU6050_AccelScale mpu6050_get_accel_scale(MPU6050_State *state);
esp_err_t mpu6050_set_gyro_scale(MPU6050_State *state, MPU6050_GyroScale scale);
MPU6050_GyroScale mpu6050_get_gyro_scale(MPU6050_State *state);
esp_err_t mpu6050_set_sleep_mode(MPU6050_State *state);
esp_err_t mpu6050_set_cycle_mode(MPU6050_State *state);
esp_err_t mpu6050_set_low_power_wake(MPU6050_State *state,
                                     MPU6050_LowPowerWakeFreq freq);
esp_err_t mpu6050_set_standby(MPU6050_State *state, uint8_t axis, bool enable);
esp_err_t mpu6050_read_accel(MPU6050_State *state, float *x, float *y,
                             float *z);
esp_err_t mpu6050_read_gyro(MPU6050_State *state, float *x, float *y, float *z);
esp_err_t mpu6050_calibrate_bias(MPU6050_State *state);

void mpu6050_debug_registers(MPU6050_State *state);
esp_err_t mpu6050_reset(MPU6050_State *state);
esp_err_t mpu6050_go_to_normal_mode(MPU6050_State *state);
#endif // MPU6050_H