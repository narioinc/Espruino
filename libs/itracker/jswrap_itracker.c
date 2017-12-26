/*
 * This file is part of Espruino, a JavaScript interpreter for Microcontrollers
 *
 * Copyright (C) 2013 Gordon Williams <gw@pur3.co.uk>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * ----------------------------------------------------------------------------
 * This file is designed to be parsed during the build process
 *
 * Contains JavaScript interface for Puck.js
 * ----------------------------------------------------------------------------
 */
 #include <stdbool.h>
 #include <stdint.h>
 #include <string.h>

#include "jswrap_itracker.h"
#include "jsinteractive.h"
#include "jsdevices.h"
#include "jshardware.h"
#include "jspin.h"
#include "jstimer.h"
#include "jswrap_bluetooth.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf5x_utils.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
//#include "jswrap_flash.h" // for jsfRemoveCodeFromFlash

/*
		bme280 PIN Assignment
		BME_CS		--	P0.02
		BME_SDI		--	P0.03
		BME_SCK		--	P0.04
		BME_SDO		--	P0.05

*/

#ifndef NRF52
#define NFR52 1
#endif

#define             BME280_SPI_CS_PIN                         2
#define             BME280_SPI_SDI_PIN                        3
#define             BME280_SPI_SCK_PIN                        4
#define             BME280_SPI_SDO_PIN                        5
#define             I2C_TIMEOUT                               100000


// Has the magnetometer been turned on?
//bool mag_enabled = false;
//int16_t mag_reading[3];  //< magnetometer xyz reading


#define SPI_CS_PIN   25  /* nRF52832ֻ��ʹ��GPIO��ΪƬѡ��������������������SPI CS�ܽ�.*/
#define SPI_BUFSIZE 8

#define SPI_INSTANCE  1 /**< SPI instance index. */

/* Private function prototypes -----------------------------------------------*/

uint8_t   SPI_Tx_Buf[SPI_BUFSIZE];
uint8_t   SPI_Rx_Buf[SPI_BUFSIZE];
volatile  uint8_t   SPIReadLength, SPIWriteLength;
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
}

static uint32_t bme280_spi_init(void)
{
    uint32_t err_code;
        nrf_drv_spi_config_t spi_bme_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_bme_config.ss_pin   = BME280_SPI_CS_PIN;
    spi_bme_config.miso_pin = BME280_SPI_SDO_PIN;
    spi_bme_config.mosi_pin = BME280_SPI_SDI_PIN;
    spi_bme_config.sck_pin  = BME280_SPI_SCK_PIN;


    err_code = nrf_drv_spi_init(&spi, &spi_bme_config, spi_event_handler);
    if(err_code != NRF_SUCCESS)
	  {
		    return err_code;
	  }

	  return NRF_SUCCESS;
}

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
		nrf_delay_ms(period);
}

int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |----------------+---------------------+-------------|
     * | MOSI           | MISO                | Chip Select |
     * |----------------+---------------------|-------------|
     * | (don't care)   | (don't care)        | HIGH        |
     * | (reg_addr)     | (don't care)        | LOW         |
     * | (don't care)   | (reg_data[0])       | LOW         |
     * | (....)         | (....)              | LOW         |
     * | (don't care)   | (reg_data[len - 1]) | LOW         |
     * | (don't care)   | (don't care)        | HIGH        |
     * |----------------+---------------------|-------------|
     */
    spi_xfer_done = false;
		nrf_gpio_pin_write ( BME280_SPI_CS_PIN, 0 );
		/* Write transaction */
    SPI_Tx_Buf[0] = reg_addr;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, SPI_Tx_Buf, 1, SPI_Rx_Buf, len+1));
	  while(spi_xfer_done == false);
    /* Send received value back to the caller */
    memcpy(reg_data, &SPI_Rx_Buf[1], len);
		nrf_gpio_pin_write ( BME280_SPI_CS_PIN, 1 );
    return rslt;
}

int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |---------------------+--------------+-------------|
     * | MOSI                | MISO         | Chip Select |
     * |---------------------+--------------|-------------|
     * | (don't care)        | (don't care) | HIGH        |
     * | (reg_addr)          | (don't care) | LOW         |
     * | (reg_data[0])       | (don't care) | LOW         |
     * | (....)              | (....)       | LOW         |
     * | (reg_data[len - 1]) | (don't care) | LOW         |
     * | (don't care)        | (don't care) | HIGH        |
     * |---------------------+--------------|-------------|
     */
		    spi_xfer_done = false;

		/* Write transaction */
    SPI_Tx_Buf[0] = reg_addr;
		memcpy(&SPI_Tx_Buf[1], reg_data, len);
		nrf_gpio_pin_write ( BME280_SPI_CS_PIN, 0 );
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, SPI_Tx_Buf, len+1, NULL, 0));
	  while(spi_xfer_done == false);
		nrf_gpio_pin_write ( BME280_SPI_CS_PIN, 1 );
    return rslt;
}


/*!
* @brief This API converts compensated sensor data to a JSON object
*/
JsVar *bme_to_pht(struct bme280_data *comp_data) {
  JsVar *obj = jsvNewObject();
  if (!obj) return 0;
  jsvObjectSetChildAndUnLock(obj,"p",jsvNewFromInteger(comp_data->pressure));
  jsvObjectSetChildAndUnLock(obj,"h",jsvNewFromInteger(comp_data->humidity));
  jsvObjectSetChildAndUnLock(obj,"t",jsvNewFromInteger(comp_data->temperature));
  return obj;
}

/*JSON{
    "type": "class",
    "class" : "iTracker"
}
Class containing [iTracker's] utility functions.
*/

/*JSON{
    "type" : "staticmethod",
    "class" : "iTracker",
    "name" : "bme280data",
    "ifdef" : "NRF52",
    "generate" : "jswrap_itracker_bme280data",
    "return" : ["JsVar", "get sensor reading from BME 280 sensor" ]
}*/

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data.
 */
JsVar jswrap_itracker_bme280data(){
  struct bme280_dev dev;
  int8_t rslt = BME280_OK;
  JsVar *obj = jsvNewObject();

  bme280_spi_init();
  /* Sensor_0 interface over SPI with native chip select line */
  dev.dev_id = 0;
  dev.intf = BME280_SPI_INTF;
  dev.read = user_spi_read;
  dev.write = user_spi_write;
  dev.delay_ms = user_delay_ms;

  rslt = bme280_init(&dev);

  	uint8_t settings_sel;
  	struct bme280_data comp_data;

  	/* Recommended mode of operation: Indoor navigation */
  	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  	dev.settings.filter = BME280_FILTER_COEFF_16;
  	dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

  	settings_sel = BME280_OSR_PRESS_SEL;
  	settings_sel |= BME280_OSR_TEMP_SEL;
  	settings_sel |= BME280_OSR_HUM_SEL;
  	settings_sel |= BME280_STANDBY_SEL;
  	settings_sel |= BME280_FILTER_SEL;
  	rslt = bme280_set_sensor_settings(settings_sel, &dev);
  	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);

  	//printf("Temperature, Pressure, Humidity\r\n");
  	//while (1) {
  		/* Delay while the sensor completes a measurement */
  		//dev->delay_ms(70);
  		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
      obj = bme_to_pht(&comp_data);
      //print_sensor_data(&comp_data);
  	//}

  	//return rslt;
}
