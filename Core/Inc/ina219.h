/**
 * @brief INA219 driver using I2C2 for STM32F103.
 * @cite ina219.pdf, rm008.pdf
 * @date 2024-06-28
 * @note Must calibrate for current/power measurements.
 */

#ifndef _INA219_H_
#define _INA219_H_

#include <inttypes.h>
#include <stdbool.h>

enum ina219_status
{
  INA219_OK,    // I2C operation successful
  INA219_ERROR, // I2C error
  INA219_OVF    // Math overflow in current/power calculation
};


typedef struct 
{
  uint8_t address;        // 7-bit I2C address
  // float current_lsb;
  // uint16_t config_val;



}INA219_t;



// Configuration and utility functions.
bool ina219_init(INA219_t *ina219);
bool ina219_reset(INA219_t *ina219);
bool ina219_get_status(INA219_t *ina219);
float ina219_get_power(INA219_t *ina219);
bool ina219_configure(INA219_t *ina219, const uint16_t val);
bool ina219_calibrate(INA219_t *ina219, const float max_current, const float r_shunt);

// Functions for measurements.
float ina219_get_current(INA219_t *ina219);
float ina219_get_bus_voltage(INA219_t *ina219);
float ina219_get_shunt_voltage(INA219_t *ina219);
void ina219_save_power(INA219_t *ina219, bool on);
bool i2c_write(INA219_t *ina219, const uint8_t reg, const uint16_t val);
uint16_t i2c_read(INA219_t *ina219, uint8_t reg);

#endif // ina219.h