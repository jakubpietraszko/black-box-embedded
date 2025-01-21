#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"

// I2C configuration
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 100000

static const char *TAG = "MPU6050";

void byte_to_binary(uint8_t byte, char *buffer) {
  for (int i = 7; i >= 0; i--) {
    buffer[7 - i] = (byte & (1 << i)) ? '1' : '0';
  }
  buffer[8] = '\0';
}

// Initialize the MPU6050
esp_err_t mpu6050_init(MPU6050_State *state) {
  i2c_config_t conf = {.mode = I2C_MODE_MASTER,
                       .sda_io_num = I2C_MASTER_SDA_IO,
                       .scl_io_num = I2C_MASTER_SCL_IO,
                       .sda_pullup_en = GPIO_PULLUP_ENABLE,
                       .scl_pullup_en = GPIO_PULLUP_ENABLE,
                       .master.clk_speed = I2C_MASTER_FREQ_HZ};
  ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

  uint8_t data = 0;
  esp_err_t ret = mpu6050_read(state, PWR_MGMT_1, &data, 1);
  if (ret != ESP_OK) {
    return ret;
  }

  // Wake up the MPU6050 by setting the SLEEP bit to 0
  data &= ~(0x40);
  return mpu6050_write(state, PWR_MGMT_1, data);
}

// Write to MPU6050 register
esp_err_t mpu6050_write(MPU6050_State *state, uint8_t reg, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (state->address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Read from MPU6050 register
esp_err_t mpu6050_read(MPU6050_State *state, uint8_t reg, uint8_t *data,
                       size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (state->address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (state->address << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return ret;
}

void mpu6050_debug_registers(MPU6050_State *state) {
  uint8_t accel_config, gyro_config, pwr_mgmt_1, pwr_mgmt_2;
  char binary_str[9]; // Buffer for binary representation

  if (mpu6050_read(state, 0x1C, &accel_config, 1) == ESP_OK) {
    byte_to_binary(accel_config, binary_str);
    ESP_LOGI(TAG, "ACCEL_CONFIG (0x1C): HEX: 0x%02X, BIN: %s | _ _ _ A A _ _ _ | %d",
             accel_config, binary_str, state->accel_scale);
  } else {
    ESP_LOGE(TAG, "Failed to read ACCEL_CONFIG (0x1C)");
  }

  if (mpu6050_read(state, 0x1B, &gyro_config, 1) == ESP_OK) {
    byte_to_binary(gyro_config, binary_str);
    ESP_LOGI(TAG, "GYRO_CONFIG (0x1B): HEX: 0x%02X, BIN: %s | _ _ _ G G _ _ _ | %d",
             gyro_config, binary_str, state->gyro_scale);
  } else {
    ESP_LOGE(TAG, "Failed to read GYRO_CONFIG (0x1B)");
  }

  if (mpu6050_read(state, 0x6B, &pwr_mgmt_1, 1) == ESP_OK) {
    byte_to_binary(pwr_mgmt_1, binary_str);
    ESP_LOGI(TAG, "PWR_MGMT_1 (0x6B): HEX: 0x%02X, BIN: %s | R S C _ _ _ _ _",
             pwr_mgmt_1, binary_str);
  } else {
    ESP_LOGE(TAG, "Failed to read PWR_MGMT_1 (0x6B)");
  }

  if (mpu6050_read(state, 0x6C, &pwr_mgmt_2, 1) == ESP_OK) {
    byte_to_binary(pwr_mgmt_2, binary_str);
    ESP_LOGI(TAG, "PWR_MGMT_2 (0x6C): HEX: 0x%02X, BIN: %s | W W S S S S S S ",
             pwr_mgmt_2, binary_str);
  } else {
    ESP_LOGE(TAG, "Failed to read PWR_MGMT_2 (0x6C)");
  }

  ESP_LOGI(TAG, "-----------------------------------");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// Set accelerometer scale
esp_err_t mpu6050_set_accel_scale(MPU6050_State *state,
                                  MPU6050_AccelScale scale) {
  uint8_t reg_value;
  esp_err_t ret;

  // Read the current value of the ACCEL_CONFIG register
  ret = mpu6050_read(state, ACCEL_CONFIG, &reg_value, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read ACCEL_CONFIG register");
    return ret;
  }

  // Clear bits 3 and 4 (accelerometer scale bits)
  reg_value &= ~0x18; // 0x18 = 00011000 (clear the 3rd and 4th bits)

  // Set the new scale value by shifting it to the correct position (3rd and 4th
  // bits)
  reg_value |= (scale << 3); // Shift the scale value to bits 3 and 4

  // Write the modified value back to the ACCEL_CONFIG register
  ret = mpu6050_write(state, ACCEL_CONFIG, reg_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write to ACCEL_CONFIG register");
    return ret;
  }

  state->accel_scale = scale;

  ESP_LOGI(TAG, "ACCEL_CONFIG updated successfully, new value: 0x%02X",
           reg_value);
  return ESP_OK;
}

// Get accelerometer scale
MPU6050_AccelScale mpu6050_get_accel_scale(MPU6050_State *state) {
  uint8_t reg_value;
  mpu6050_read(state, ACCEL_CONFIG, &reg_value, 1);
  return (MPU6050_AccelScale)((reg_value >> 3) & 0x03);
}

// Set gyroscope scale
esp_err_t mpu6050_set_gyro_scale(MPU6050_State *state,
                                 MPU6050_GyroScale scale) {
  uint8_t reg_value;
  esp_err_t ret;

  // Read the current value of the GYRO_CONFIG register
  ret = mpu6050_read(state, GYRO_CONFIG, &reg_value, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read GYRO_CONFIG register");
    return ret;
  }

  // Clear bits 3 and 4 (gyroscope scale bits)
  reg_value &= ~0x18; // 0x18 = 00011000 (clear the 3rd and 4th bits)

  // Set the new scale value by shifting it to the correct position (3rd and 4th
  // bits)
  reg_value |= (scale << 3); // Shift the scale value to bits 3 and 4

  // Write the modified value back to the GYRO_CONFIG register
  ret = mpu6050_write(state, GYRO_CONFIG, reg_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write to GYRO_CONFIG register");
    return ret;
  }

state->gyro_scale = scale;

  ESP_LOGI(TAG, "GYRO_CONFIG updated successfully, new value: 0x%02X",
           reg_value);
  return ESP_OK;
}

// Get gyroscope scale
MPU6050_GyroScale mpu6050_get_gyro_scale(MPU6050_State *state) {
  uint8_t reg_value;
  mpu6050_read(state, GYRO_CONFIG, &reg_value, 1);
  return (MPU6050_GyroScale)((reg_value >> 3) & 0x03);
}

// Set the device to sleep mode
esp_err_t mpu6050_set_sleep_mode(MPU6050_State *state) {
  uint8_t reg_value;
  esp_err_t ret;

  // Read the current value of the PWR_MGMT_1 register
  ret = mpu6050_read(state, PWR_MGMT_1, &reg_value, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read PWR_MGMT_1 register");
    return ret;
  }

  // Set the sleep bit (bit 6) and clear the cycle bit (bit 5) while preserving
  // other bits
  reg_value |= (1 << 6);  // Set SLEEP bit
  reg_value &= ~(1 << 5); // Clear CYCLE bit

  // Write the modified value back to the PWR_MGMT_1 register
  ret = mpu6050_write(state, PWR_MGMT_1, reg_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write to PWR_MGMT_1 register");
    return ret;
  }

  state->state = MPU6050_STATE_SLEEP;

  ESP_LOGI(TAG, "Device set to sleep mode, PWR_MGMT_1 updated: 0x%02X",
           reg_value);
  return ESP_OK;
}

// Set the device to cycle mode
esp_err_t mpu6050_set_cycle_mode(MPU6050_State *state) {
  uint8_t reg_value;
  esp_err_t ret;

  // Read the current value of the PWR_MGMT_1 register
  ret = mpu6050_read(state, PWR_MGMT_1, &reg_value, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read PWR_MGMT_1 register");
    return ret;
  }

  // Set the cycle bit (bit 5) and clear the sleep bit (bit 6), preserving other
  // bits
  reg_value |= (1 << 5);  // Set CYCLE bit
  reg_value &= ~(1 << 6); // Clear SLEEP bit

  // Write the modified value back to the PWR_MGMT_1 register
  ret = mpu6050_write(state, PWR_MGMT_1, reg_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write to PWR_MGMT_1 register");
    return ret;
  }

  state->state = MPU6050_STATE_CYCLE;

  ESP_LOGI(TAG, "Device set to cycle mode, PWR_MGMT_1 updated: 0x%02X",
           reg_value);
  return ESP_OK;
}

// Set low power wake control frequency
esp_err_t mpu6050_set_low_power_wake(MPU6050_State *state,
                                     MPU6050_LowPowerWakeFreq freq) {
  uint8_t reg_value;
  esp_err_t ret;

  // Read the current value of the PWR_MGMT_2 register
  ret = mpu6050_read(state, PWR_MGMT_2, &reg_value, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read PWR_MGMT_2 register");
    return ret;
  }

  // Clear bits 6 and 7 (the wake frequency bits)
  reg_value &= ~(0b11000000);

  // Set the frequency bits based on the given frequency
  reg_value |= (freq << 6);

  // Write the modified value back to the PWR_MGMT_2 register
  ret = mpu6050_write(state, PWR_MGMT_2, reg_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write to PWR_MGMT_2 register");
    return ret;
  }

  state->lp_wake_ctrl = freq;



  ESP_LOGI(TAG, "PWR_MGMT_2 updated successfully, new value: 0x%02X",
           reg_value);
  return ESP_OK;
}

// Set standby for specific axis (0 = disabled, 1 = enabled)
esp_err_t mpu6050_set_standby(MPU6050_State *state, uint8_t axis, bool enable) {
  uint8_t reg_value;
  mpu6050_read(state, PWR_MGMT_2, &reg_value, 1);
  if (enable) {
    reg_value |= (1 << axis); // Set the corresponding axis bit to 1
  } else {
    reg_value &= ~(1 << axis); // Clear the corresponding axis bit to 0
  }
  state->standby[axis] = enable;
  return mpu6050_write(state, PWR_MGMT_2, reg_value);
}

esp_err_t mpu6050_read_accel(MPU6050_State *state, float *x, float *y,
                             float *z) {
  uint8_t data[6];
  esp_err_t ret = mpu6050_read(state, ACCEL_XOUT_H, data, 6);
  if (ret != ESP_OK) {
    return ret;
  }

  // Combine the high and low bytes to get the raw values
  int16_t raw_x = ((int16_t)(data[0] << 8) | data[1]);
  int16_t raw_y = ((int16_t)(data[2] << 8) | data[3]);
  int16_t raw_z = ((int16_t)(data[4] << 8) | data[5]);

  // Convert raw accelerometer data to 'g' using the appropriate conversion
  // constant
  switch (state->accel_scale) {
  case MPU6050_ACCEL_SCALE_2G:
    *x = (raw_x / (float)CONVERT_ACCEL_2G) - state->accel_bias_x;
    *y = (raw_y / (float)CONVERT_ACCEL_2G) - state->accel_bias_y;
    *z = (raw_z / (float)CONVERT_ACCEL_2G) - state->accel_bias_z;
    break;
  case MPU6050_ACCEL_SCALE_4G:
    *x = (raw_x / (float)CONVERT_ACCEL_4G) - state->accel_bias_x;
    *y = (raw_y / (float)CONVERT_ACCEL_4G) - state->accel_bias_y;
    *z = (raw_z / (float)CONVERT_ACCEL_4G) - state->accel_bias_z;
    break;
  case MPU6050_ACCEL_SCALE_8G:
    *x = (raw_x / (float)CONVERT_ACCEL_8G) - state->accel_bias_x;
    *y = (raw_y / (float)CONVERT_ACCEL_8G) - state->accel_bias_y;
    *z = (raw_z / (float)CONVERT_ACCEL_8G) - state->accel_bias_z;
    break;
  case MPU6050_ACCEL_SCALE_16G:
    *x = (raw_x / (float)CONVERT_ACCEL_16G) - state->accel_bias_x;
    *y = (raw_y / (float)CONVERT_ACCEL_16G) - state->accel_bias_y;
    *z = (raw_z / (float)CONVERT_ACCEL_16G) - state->accel_bias_z;
    break;
  }
  return ESP_OK;
}

esp_err_t mpu6050_read_gyro(MPU6050_State *state, float *x, float *y,
                            float *z) {
  uint8_t data[6];
  esp_err_t ret = mpu6050_read(state, GYRO_XOUT_H, data, 6);
  if (ret != ESP_OK) {
    return ret;
  }

  // Combine the high and low bytes to get the raw values
  int16_t raw_x = ((int16_t)(data[0] << 8) | data[1]);
  int16_t raw_y = ((int16_t)(data[2] << 8) | data[3]);
  int16_t raw_z = ((int16_t)(data[4] << 8) | data[5]);

  // Convert raw gyroscope data to 'deg/s' using the appropriate conversion
  // constant
  switch (state->gyro_scale) {
  case MPU6050_GYRO_SCALE_250DPS:
    *x = (raw_x / (float)CONVERT_GYRO_250DPS) - state->gyro_bias_x;
    *y = (raw_y / (float)CONVERT_GYRO_250DPS) - state->gyro_bias_y;
    *z = (raw_z / (float)CONVERT_GYRO_250DPS) - state->gyro_bias_z;
    break;
  case MPU6050_GYRO_SCALE_500DPS:
    *x = (raw_x / (float)CONVERT_GYRO_500DPS) - state->gyro_bias_x;
    *y = (raw_y / (float)CONVERT_GYRO_500DPS) - state->gyro_bias_y;
    *z = (raw_z / (float)CONVERT_GYRO_500DPS) - state->gyro_bias_z;
    break;
  case MPU6050_GYRO_SCALE_1000DPS:
    *x = (raw_x / (float)CONVERT_GYRO_1000DPS) - state->gyro_bias_x;
    *y = (raw_y / (float)CONVERT_GYRO_1000DPS) - state->gyro_bias_y;
    *z = (raw_z / (float)CONVERT_GYRO_1000DPS) - state->gyro_bias_z;
    break;
  case MPU6050_GYRO_SCALE_2000DPS:
    *x = (raw_x / (float)CONVERT_GYRO_2000DPS) - state->gyro_bias_x;
    *y = (raw_y / (float)CONVERT_GYRO_2000DPS) - state->gyro_bias_y;
    *z = (raw_z / (float)CONVERT_GYRO_2000DPS) - state->gyro_bias_z;
    break;
  }
  return ESP_OK;
}

// Function to automatically set bias for accelerometer and gyroscope
esp_err_t mpu6050_calibrate_bias(MPU6050_State *state) {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;

  float accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
  float gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;

  // Take BIAS_SAMPLE_COUNT samples
  for (int i = 0; i < BIAS_SAMPLE_COUNT; i++) {
    // Read accelerometer values
    esp_err_t ret = mpu6050_read_accel(state, &accel_x, &accel_y, &accel_z);
    if (ret != ESP_OK) {
      return ret;
    }

    // Read gyroscope values
    ret = mpu6050_read_gyro(state, &gyro_x, &gyro_y, &gyro_z);
    if (ret != ESP_OK) {
      return ret;
    }

    // Sum the values for averaging
    accel_x_sum += accel_x;
    accel_y_sum += accel_y;
    accel_z_sum += accel_z;
    gyro_x_sum += gyro_x;
    gyro_y_sum += gyro_y;
    gyro_z_sum += gyro_z;
  }

  // Calculate the average (bias)
  state->accel_bias_x = accel_x_sum / BIAS_SAMPLE_COUNT;
  state->accel_bias_y = accel_y_sum / BIAS_SAMPLE_COUNT;
  state->accel_bias_z = accel_z_sum / BIAS_SAMPLE_COUNT - 1;

  state->gyro_bias_x = gyro_x_sum / BIAS_SAMPLE_COUNT;
  state->gyro_bias_y = gyro_y_sum / BIAS_SAMPLE_COUNT;
  state->gyro_bias_z = gyro_z_sum / BIAS_SAMPLE_COUNT;





  ESP_LOGI(TAG, "Accelerometer Bias: X = %.2f, Y = %.2f, Z = %.2f",
           state->accel_bias_x, state->accel_bias_y, state->accel_bias_z);
  ESP_LOGI(TAG, "Gyroscope Bias: X = %.2f, Y = %.2f, Z = %.2f",
           state->gyro_bias_x, state->gyro_bias_y, state->gyro_bias_z);

  return ESP_OK;
}

// Function to reset the MPU6050 device
esp_err_t mpu6050_reset(MPU6050_State *state) {
  uint8_t reg_value;

  // Read the current value of the PWR_MGMT_1 register
  esp_err_t ret = mpu6050_read(state, PWR_MGMT_1, &reg_value, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read PWR_MGMT_1 register");
    return ret;
  }

  // Set the RESET bit (bit 7)
  reg_value |= (1 << 7);

  // Write the modified value back to the PWR_MGMT_1 register to trigger reset
  ret = mpu6050_write(state, PWR_MGMT_1, reg_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write to PWR_MGMT_1 register");
    return ret;
  }

  // Wait for the reset to complete (it takes around 100ms)
  vTaskDelay(100 / portTICK_PERIOD_MS);

  ESP_LOGI(TAG, "MPU6050 device reset successfully");
  return ESP_OK;
}

esp_err_t mpu6050_go_to_normal_mode(MPU6050_State *state) {
  uint8_t reg_value;

  // Read the current value of the PWR_MGMT_1 register
  esp_err_t ret = mpu6050_read(state, PWR_MGMT_1, &reg_value, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read PWR_MGMT_1 register");
    return ret;
  }

  // Clear the SLEEP (bit 6) and CYCL (bit 5) bits in PWR_MGMT_1 to disable
  // sleep and cycle mode
  reg_value &= ~(1 << 6); // Clear SLEEP bit (0)
  reg_value &= ~(1 << 5); // Clear CYCL bit (0)

  // Write the modified value back to the PWR_MGMT_1 register
  ret = mpu6050_write(state, PWR_MGMT_1, reg_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write to PWR_MGMT_1 register");
    return ret;
  }

  // Read the current value of the PWR_MGMT_2 register
  ret = mpu6050_read(state, PWR_MGMT_2, &reg_value, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read PWR_MGMT_2 register");
    return ret;
  }

  ESP_LOGI(TAG, "MPU6050 successfully set to normal mode");
  return ESP_OK;
}
