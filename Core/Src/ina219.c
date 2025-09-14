#include "ina219.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c2; // Use I2C2 handle from STM32 HAL

// INA219 macros
#define INA219_I2C_SLAVE_ADD 0x40
#define INA219_REG_CONFIGURATION 0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE 0x02
#define INA219_REG_POWER 0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05

#define INA219_CONFIG_MODE_POWERDOWN              0x0000  // Power-down mode
#define INA219_CONFIG_MODE_SHUNT_AND_BUS_CONT     0x0007  // Normal operation


static uint16_t config_val = 0x399F; // Configuration value tracker.
static float current_lsb = 0.0; // Please calibrate for current/power measurements.

uint16_t i2c_read(INA219_t *ina219, const uint8_t reg);
bool i2c_write(INA219_t *ina219, const uint8_t reg, const uint16_t val);

bool ina219_init(INA219_t *ina219)
{
  if(ina219_reset(ina219))
  {
    return true;
  }
  return false;
}

void ina219_save_power(INA219_t *ina219, bool on)
{
  uint16_t config = i2c_read(ina219, INA219_REG_CONFIGURATION);

  if (on) {
    config = (config & 0xFFF8) | INA219_CONFIG_MODE_POWERDOWN; 
  } else {
    config = (config & 0xFFF8) | INA219_CONFIG_MODE_SHUNT_AND_BUS_CONT;
  }

  i2c_write(ina219, INA219_REG_CONFIGURATION,config);

}


bool ina219_reset(INA219_t *ina219)
{
  if(i2c_write(ina219, INA219_REG_CONFIGURATION, 0x8000)){
    printf("reset write success\n");
  }
  else
  {
    printf("reset write failed\n");

  }
  if(i2c_read(ina219, INA219_REG_CONFIGURATION) == 0x399F)
  {
    config_val = 0x399F;
    current_lsb = 0.0;
    printf("reset success\n");
    return true;
  }
  printf("reset failed\n");
  return false;
}

bool ina219_calibrate(INA219_t *ina219, const float max_current, const float r_shunt)
{
  float temp_current_lsb = max_current * 3.0517578125e-5;
  const uint16_t calibration_val = (uint16_t)(0.04096 / (temp_current_lsb * r_shunt));
  i2c_write(ina219, INA219_REG_CALIBRATION, calibration_val);

  if(i2c_read(ina219, INA219_REG_CALIBRATION) == (calibration_val & 0xFFFE))
  {
    current_lsb = temp_current_lsb;
    return true;
  }
  return false;
}

bool ina219_configure(INA219_t *ina219, const uint16_t val)
{
  i2c_write(ina219, INA219_REG_CONFIGURATION, val);
  if(i2c_read(ina219, INA219_REG_CONFIGURATION) == val)
  {
    config_val = val;
    return true;
  }
  return false;
}

float ina219_get_bus_voltage(INA219_t *ina219)
{
  uint16_t reg_val = i2c_read(ina219, INA219_REG_BUS_VOLTAGE);
  return (reg_val >> 3) * 4e-3;
}

float ina219_get_current(INA219_t *ina219)
{
  float reg_val = (int16_t)i2c_read(ina219, INA219_REG_CURRENT);
  return reg_val * current_lsb;
}

float ina219_get_shunt_voltage(INA219_t *ina219)
{
  float reg_val = (int16_t)i2c_read(ina219, INA219_REG_SHUNT_VOLTAGE);
  return reg_val * 1e-5;
}

float ina219_get_power(INA219_t *ina219)
{
  float reg_val = (int16_t)i2c_read(ina219, INA219_REG_POWER);
  return reg_val * 20 * current_lsb;
}

bool ina219_get_status(INA219_t *ina219)
{
  return (i2c_read(ina219, INA219_REG_CONFIGURATION) == config_val);
}

uint16_t i2c_read(INA219_t *ina219, uint8_t reg)
{
  uint8_t data[2] = {0};
  HAL_I2C_Master_Transmit(&hi2c2, ina219->address << 1, &reg, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c2, ina219->address << 1, data, 2, HAL_MAX_DELAY);
  return (data[0] << 8) | data[1];
}

bool i2c_write(INA219_t *ina219, const uint8_t reg, const uint16_t val)
{
  uint8_t data[3] = {reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
  if(HAL_I2C_Master_Transmit(&hi2c2, ina219->address << 1, data, 3, HAL_MAX_DELAY)==HAL_OK) return 1;
  else return 0;
}
