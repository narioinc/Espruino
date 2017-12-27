#ifndef __itracker_I2C_DRV_H__
#define __itracker_I2C_DRV_H__

#include "stdint.h"
#include "nrf_drv_twi.h"

uint32_t itracker_i2c_init(const nrf_drv_twi_config_t *twi_config);
uint32_t itracker_i2c_write(uint8_t twi_addr, uint8_t reg, uint8_t *data, uint16_t len);
uint8_t itracker_i2c_read(uint8_t twi_addr, uint8_t reg, uint8_t * data, uint16_t len);
uint32_t itracker_i2c_simple_write(uint8_t twi_addr, uint8_t *data, uint16_t len);
uint8_t itracker_i2c_simple_read(uint8_t twi_addr, uint8_t * data, uint16_t len);
void itracker_i2c_disable();
void itracker_i2c_deinit();

#endif
