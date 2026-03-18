/*
  This file is part of the Arduino_LSM6DS3 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cstring>
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "LSM6DS3.h"

#define I2C_NUM I2C_NUM_0
#define I2C_TICKS_TO_WAIT 100 // Maximum ticks to wait before issuing a timeout.

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

// Arduino compatible macros
#define delay(ms) esp_rom_delay_us(ms*1000)

/** Default constructor, uses default I2C address.
 * @see LSM6DS3_DEFAULT_ADDRESS
 */
LSM6DS3::LSM6DS3() {
  devAddr = LSM6DS3_DEFAULT_ADDRESS;
}

/** Specific address constructor.
 * @param address I2C address
 * @see LSM6DS3_DEFAULT_ADDRESS
 * @see LSM6DS3_ADDRESS_00
 * @see LSM6DS3_ADDRESS_01
 */
LSM6DS3::LSM6DS3(uint16_t address) {
  devAddr = address;
}

int LSM6DS3::begin()
{
  // using I2C for communication
  i2c_master_bus_config_t i2c_mst_config = {};
  i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
  i2c_mst_config.glitch_ignore_cnt = 7;
  i2c_mst_config.i2c_port = I2C_NUM;
  i2c_mst_config.scl_io_num = (gpio_num_t)CONFIG_GPIO_SCL;
  i2c_mst_config.sda_io_num = (gpio_num_t)CONFIG_GPIO_SDA;
  i2c_mst_config.flags.enable_internal_pullup = true;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = devAddr;
  dev_cfg.scl_speed_hz = 400000;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

  // Get Who_AM_I register
  //int who_am_i = readRegister(LSM6DS3_WHO_AM_I_REG);
  who_am_i = readRegister(LSM6DS3_WHO_AM_I_REG);
  printf("who_am_i=0x%x\n", who_am_i);
  switch(who_am_i) {
    case 0x69:
      printf("IMU is LSM6DS3\n");
      break;
    case 0x6A:
      printf("IMU is LSM6DSM/LSM6DSL\n");
      break;
    case 0x6B:
      printf("IMU is LSM6DSR\n");
      break;
    case 0x6C:
      printf("IMU is LSM6DSO\n");
      break;
    default:
      printf("IMU is unknown\n");
      end();
      return 0;
  } // end switch

#if 0
  if (!(readRegister(LSM6DS3_WHO_AM_I_REG) == 0x6C || readRegister(LSM6DS3_WHO_AM_I_REG) == 0x69)) {
    end();
    return 0;
  }
#endif

  // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass mode and enable ODR/4
  // low pass filter (check figure9 of LSM6DS3's datasheet)
  writeRegister(LSM6DS3_CTRL1_XL, 0x4A);

#if 0
  //set the gyroscope control register to work at 104 Hz, 2000 dps and in bypass mode
  writeRegister(LSM6DS3_CTRL2_G, 0x4C);
#endif
  //set the gyroscope control register to work at 104 Hz, 250 dps and in bypass mode
  writeRegister(LSM6DS3_CTRL2_G, 0x40);

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  writeRegister(LSM6DS3_CTRL7_G, 0x00);

  // Set the ODR config register to ODR/4
  writeRegister(LSM6DS3_CTRL8_XL, 0x09);

  return 1;
}

void LSM6DS3::end()
{
  writeRegister(LSM6DS3_CTRL2_G, 0x00);
  writeRegister(LSM6DS3_CTRL1_XL, 0x00);
}

int LSM6DS3::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DS3_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
    return 0;
  }

#if 0
  x = data[0] * 4.0 / 32768.0;
  y = data[1] * 4.0 / 32768.0;
  z = data[2] * 4.0 / 32768.0;
#endif
  x = data[0] / 8192.0;
  y = data[1] / 8192.0;
  z = data[2] / 8192.0;

  return 1;
}

int LSM6DS3::accelerationAvailable()
{
  if (readRegister(LSM6DS3_STATUS_REG) & 0x01) {
    return 1;
  }

  return 0;
}

float LSM6DS3::accelerationSampleRate()
{
  return 104.0F;
}

int LSM6DS3::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DS3_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    return 0;
  }

  // Gyroscope full-scale of LSM6DS3 is 250 dps.
  float gyroScale = 32768.0 / 250.0;
  // Gyroscope full-scale of LSM6DSM is 245 dps.
  if (who_am_i == 0x6A) gyroScale = 32768.0 / 245.0;
  //printf("gyroScale=%f\n", gyroScale);
#if 0
  x = data[0] * 250.0 / 32768.0;
  y = data[1] * 250.0 / 32768.0;
  z = data[2] * 250.0 / 32768.0;
#endif

#if 0
  x = (data[0] - _gyroBias[0]) / 131.0;
  y = (data[1] - _gyroBias[1]) / 131.0;
  z = (data[2] - _gyroBias[2]) / 131.0;
#endif

  x = (data[0] - _gyroBias[0]) / gyroScale;
  y = (data[1] - _gyroBias[1]) / gyroScale;
  z = (data[2] - _gyroBias[2]) / gyroScale;
  return 1;
}

int LSM6DS3::gyroscopeAvailable()
{
  if (readRegister(LSM6DS3_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

float LSM6DS3::gyroscopeSampleRate()
{
  return 104.0F;
}

void LSM6DS3::getGyroscopeBias(float *gyroBias) {
  int16_t data[3];
  int32_t sum[3] = {0};
  int count = 0;

  while(1) {
    if (readRegisters(LSM6DS3_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
      sum[0] += data[0];
      sum[1] += data[1];
      sum[2] += data[2];
      //printf("sum=%ld %ld %ld\n", sum[0],sum[1],sum[2]);
      count++;
      if (count == 100) break;
      delay(10);
    }
  }
  for (int i = 0; i < 3; ++i) {
    gyroBias[i] = sum[i] / 100.0f;
  }
}

void LSM6DS3::setGyroscopeBias(float *gyroBias) {
  for (int i = 0; i < 3; ++i) {
    _gyroBias[i] = gyroBias[i];
  }
}

int LSM6DS3::readRegister(uint8_t address)
{
  uint8_t value;
  
  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }
  
  return value;
}

int LSM6DS3::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
  uint8_t out_buf[1];
  out_buf[0] = address;
  ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, out_buf, 1, data, length, I2C_TICKS_TO_WAIT));
  return 1;
}

int LSM6DS3::writeRegister(uint8_t address, uint8_t value)
{
  uint8_t out_buf[2];
  out_buf[0] = address;
  out_buf[1] = value;
  ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, out_buf, 2, I2C_TICKS_TO_WAIT));
  return 1;
}
