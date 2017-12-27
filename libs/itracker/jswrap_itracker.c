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
#include "bme280/bme280_driver.h"
#include "itracker_i2c_drv.h"
#include "opt3001/opt3001_driver.h"
#include "lis3dh/lis3dh_driver.h"
#include "lis2mdl/lis2mdl_driver.h"

#ifndef NRF52
#define NFR52 1
#endif


/*
		bme280 PIN Assignment
		BME_CS		--	P0.02
		BME_SDI		--	P0.03
		BME_SCK		--	P0.04
		BME_SDO		--	P0.05

*/
#define             BME280_SPI_CS_PIN                         2
#define             BME280_SPI_SDI_PIN                        3
#define             BME280_SPI_SCK_PIN                        4
#define             BME280_SPI_SDO_PIN                        5
#define             I2C_TIMEOUT                               100000

/*
		OPT3001 PIN Assignment
		OPT_SDA		--	P0.21
		OPT_INT		--	P0.22
		OPT_SCL		--	P0.23

*/
#define             OPT3001_TWI_SDA_PIN                        21
#define             OPT3001_INT_PIN                        		 22
#define             OPT3001_TWI_SCL_PIN                        23
#define             OPT3001_ADDR                               0x44


/*
		lis3dh PIN Assignment
		LIS3DH_SCL		--	P0.18
		LIS3DH_SDA		--	P0.19
		LIS3DH_INT1		--	P0.25
		LIS3DH_RES		--	P0.26
		LIS3DH_INT2		--	P0.27

*/
#define             LIS3DH_TWI_SCL_PIN                     16
#define             LIS3DH_TWI_SDA_PIN                     15
#define             LIS3DH_INT1_PIN                        25
#define 						LIS3DH_RES_PIN											   26
#define             LIS3DH_INT2_PIN                        27
#define             LIS3DH_ADDR                            0x19


/*
		lis2mdl PIN Assignment
		LIS2MDL_SCL		--	P0.11
		LIS2MDL_SDA		--	P0.13
		LIS2MDL_INT		--	P0.16

*/
#define             LIS2MDL_TWI_SCL_PIN                        11
#define             LIS2MDL_TWI_SDA_PIN                        13
#define             LIS2MDL_INT_PIN                            16
#define             LIS2MDL_ADDR                               0x1E

// Has the magnetometer been turned on?
// bool mag_enabled = false;
// int16_t mag_reading[3];  //< magnetometer xyz reading


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
JsVar *jswrap_itracker_bme280data(){
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

  	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
    obj = bme_to_pht(&comp_data);

    return obj;
}

static uint32_t opt3001_twi_init(void)
{
    uint32_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = OPT3001_TWI_SCL_PIN,
       .sda                = OPT3001_TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST
    };

    err_code = itracker_i2c_init(&twi_config);
    if(err_code != NRF_SUCCESS)
	  {
		    return err_code;
	  }

	  return NRF_SUCCESS;
}

uint32_t opt3001_init(void)
{
    uint32_t err_code;
	  uint8_t id =0;

		//init i2c
	  err_code = opt3001_twi_init();
    if(err_code != NRF_SUCCESS) return err_code;

		sensorOpt3001Enable(1);
		sensorOpt3001Test();
    return NRF_SUCCESS;
}

void opt3001_deinit()
{
		itracker_i2c_deinit();
}


/*JSON{
    "type" : "staticmethod",
    "class" : "iTracker",
    "name" : "opt3001data",
    "ifdef" : "NRF52",
    "generate" : "jswrap_itracker_opt3001data",
    "return" : ["JsVar", "get sensor reading from OPT 3001 sensor" ]
}*/

JsVar *jswrap_itracker_opt3001data()
{
		int count = 1;
		uint16_t light_raw_data=0;
    JsVar *obj = jsvNewObject();

    opt3001_init();

		while(count--)
		{
			sensorOpt3001Read(&light_raw_data);
			float light_data = sensorOpt3001Convert(light_raw_data);
      jsvObjectSetChildAndUnLock(obj,"l",jsvNewFromInteger(light_data));
      nrf_delay_ms(1000);
		}
    opt3001_deinit();
    return obj;
}


static uint32_t lis3dh_twi_init(void)
{
    uint32_t err_code;

    const nrf_drv_twi_config_t twi_lis_config = {
       .scl                = LIS3DH_TWI_SCL_PIN,
       .sda                = LIS3DH_TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST
    };

    err_code = itracker_i2c_init(&twi_lis_config);
    if(err_code != NRF_SUCCESS)
	  {
		    return err_code;
	  }

	  return NRF_SUCCESS;
}

uint32_t lis3dh_init(void)
{
    uint32_t err_code;
	  uint8_t id =0;

	  //��ʼ��TWI
	  err_code = lis3dh_twi_init();
    if(err_code != NRF_SUCCESS) return err_code;

    return NRF_SUCCESS;
}

void lis3dh_deinit()
{
	itracker_i2c_deinit();
}

/*!
* @brief This API converts compensated sensor data to a JSON object
*/
JsVar *lis3dh_to_xyz(AxesRaw_t data) {
  JsVar *obj = jsvNewObject();
  if (!obj) return 0;
  jsvObjectSetChildAndUnLock(obj,"x",jsvNewFromInteger(data.AXIS_X));
  jsvObjectSetChildAndUnLock(obj,"y",jsvNewFromInteger(data.AXIS_Y));
  jsvObjectSetChildAndUnLock(obj,"z",jsvNewFromInteger(data.AXIS_Z));

  return obj;
}


/*JSON{
    "type" : "staticmethod",
    "class" : "iTracker",
    "name" : "lis3dhdata",
    "ifdef" : "NRF52",
    "generate" : "jswrap_itracker_lis3dhdata",
    "return" : ["JsVar", "get sensor reading from LIS3DH sensor" ]
}*/
JsVar *jswrap_itracker_lis3dhdata()
{
		uint8_t response;
		AxesRaw_t data;
		uint8_t position=0, old_position=0;
    JsVar *obj = jsvNewObject();

    lis3dh_init();

		//Inizialize MEMS Sensor
		//set ODR (turn ON device)
		response = LIS3DH_SetODR(LIS3DH_ODR_100Hz);
		if(response==1){
			//DPRINTF(LOG_INFO,"%s", "\n\rSET_ODR_OK    \n\r\0");
		}
		//set PowerMode
		response = LIS3DH_SetMode(LIS3DH_NORMAL);
		if(response==1){
			//DPRINTF(LOG_INFO,"%s", "SET_MODE_OK     \n\r\0");
		}
		//set Fullscale
		response = LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);
		if(response==1){
			//DPRINTF(LOG_INFO,"%s", "SET_FULLSCALE_OK\n\r\0");
		}
		//set axis Enable
		response = LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE);
		if(response==1){
			//DPRINTF(LOG_INFO,"%s", "SET_AXIS_OK     \n\r\0");
		}

		uint8_t cnt=10;
		while(cnt--)
		{
				//get Acceleration Raw data
				response = LIS3DH_GetAccAxesRaw(&data);
				if(response==1)
				{
						//DPRINTF(LOG_INFO,"X=%6d Y=%6d Z=%6d \r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
						old_position = position;
				}
		}
		obj = lis3dh_to_xyz(data);
    lis3dh_deinit();
    return obj;

}

lis2mdl_ctx_t dev_ctx;

static uint32_t lis2mdl_twi_init(void)
{
    uint32_t err_code;

    const nrf_drv_twi_config_t twi_lis_config = {
       .scl                = LIS2MDL_TWI_SCL_PIN,
       .sda                = LIS2MDL_TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST
    };

    err_code = itracker_i2c_init(&twi_lis_config);
    if(err_code != NRF_SUCCESS)
	  {
		    return err_code;
	  }

	  return NRF_SUCCESS;
}

static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
    uint32_t err_code;
		err_code = itracker_i2c_write(LIS2MDL_ADDR, Reg, Bufp, len);
    return err_code;
}


static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
		uint32_t err_code;
		err_code = itracker_i2c_read(LIS2MDL_ADDR, Reg, Bufp, len);
    return err_code;
}


uint32_t lis2mdl_init(void)
{
    uint32_t err_code;

	  //��ʼ��TWI
	  err_code = lis2mdl_twi_init();
    if(err_code != NRF_SUCCESS) return err_code;

		/*
		 *  Initialize mems driver interface
		 */

		dev_ctx.write_reg = platform_write;
		dev_ctx.read_reg = platform_read;

    return NRF_SUCCESS;
}

void lis2mdl_deinit()
{
		itracker_i2c_deinit();
}

/*!
* @brief This API converts compensated sensor data to a JSON object
*/
JsVar *lis2mdl_to_xyz(int16_t magnetic_mG[]) {
  JsVar *obj = jsvNewObject();
  if (!obj) return 0;
  jsvObjectSetChildAndUnLock(obj,"x",jsvNewFromInteger(magnetic_mG[0]));
  jsvObjectSetChildAndUnLock(obj,"y",jsvNewFromInteger(magnetic_mG[1]));
  jsvObjectSetChildAndUnLock(obj,"z",jsvNewFromInteger(magnetic_mG[2]));

  return obj;
}

/*JSON{
    "type" : "staticmethod",
    "class" : "iTracker",
    "name" : "lis2mdldata",
    "ifdef" : "NRF52",
    "generate" : "jswrap_itracker_lis2mdldata",
    "return" : ["JsVar", "get sensor reading from LIS2MDL sensor" ]
}*/

JsVar *jswrap_itracker_lis2mdldata()
{
	axis3bit16_t data_raw_magnetic;
	axis1bit16_t data_raw_temperature;
	float magnetic_mG[3];
	float temperature_degC;
	uint8_t whoamI, rst;
  JsVar *obj = jsvNewObject();

  lis2mdl_init();

	 /*
   *  Check device ID
   */
  whoamI = 0;
  lis2mdl_device_id_get(&dev_ctx, &whoamI);
  if ( whoamI != LIS2MDL_ID )
    while(1); /*manage here device not found */
	/*
   *  Restore default configuration
   */
  lis2mdl_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lis2mdl_reset_get(&dev_ctx, &rst);
  } while (rst);
  /*
   *  Enable Block Data Update
   */
  lis2mdl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Set Output Data Rate
   */
  lis2mdl_data_rate_set(&dev_ctx, LIS2MDL_ODR_10Hz);
  /*
   * Set / Reset sensor mode
   */
  lis2mdl_set_rst_mode_set(&dev_ctx, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);
  /*
   * Enable temperature compensation
   */
  lis2mdl_offset_temp_comp_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Set device in continuos mode
   */
  lis2mdl_operating_mode_set(&dev_ctx, LIS2MDL_CONTINUOUS_MODE);

  /*
   * Read samples in polling mode (no int)
   */
	uint8_t cnt=10;
	uint8_t valid=0;
  while(cnt-- || valid==0)
  {
    /*
     * Read output only if new value is available
     */
    lis2mdl_reg_t reg;
    lis2mdl_status_get(&dev_ctx, &reg.status_reg);

    if (reg.status_reg.zyxda)
    {
      /* Read magnetic field data */
      memset(data_raw_magnetic.u8bit, 0x00, 3*sizeof(int16_t));
      lis2mdl_magnetic_raw_get(&dev_ctx, data_raw_magnetic.u8bit);
      magnetic_mG[0] = LIS2MDL_FROM_LSB_TO_mG( data_raw_magnetic.i16bit[0]);
      magnetic_mG[1] = LIS2MDL_FROM_LSB_TO_mG( data_raw_magnetic.i16bit[1]);
      magnetic_mG[2] = LIS2MDL_FROM_LSB_TO_mG( data_raw_magnetic.i16bit[2]);

      //sprintf((char*)buf, "%4.2f\t%4.2f\t%4.2f\r\n", magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
      //DPRINTF(LOG_INFO, "%s", buf);
			valid = 1;

    }
  }
  obj = lis2mdl_to_xyz(magnetic_mG);
  lis2mdl_deinit();
  return obj;
}
