/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>
#include <string.h>
#include "nrf_delay.h"
#include "sensors.h"
#include "bme280.h"
#include "bme280_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define BME280_I2C_DEV_ADDR             BME280_I2C_ADDR_PRIM
 
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static struct bme280_dev bme280_dev;
static int8_t bme280_rslt = BME280_OK;
 
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
 
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

int8_t bme280_sensor_init(void *twi_intf) {
    uint8_t settings_sel;
	
    bme280_rslt = bme280_interface_selection(&bme280_dev, BME280_I2C_INTF, BME280_I2C_DEV_ADDR, twi_intf);	
    if (bme280_rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to select interface (code %+d).", bme280_rslt);
        return bme280_rslt;
    }
	
    bme280_rslt = bme280_init(&bme280_dev);
    if (bme280_rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to initialize sensor (code %+d).", bme280_rslt);
        return bme280_rslt;
    }
	
    /* Recommended mode of operation: Indoor navigation */
    bme280_dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    bme280_dev.settings.osr_p = BME280_OVERSAMPLING_1X;
    bme280_dev.settings.osr_t = BME280_OVERSAMPLING_1X;
    bme280_dev.settings.filter = BME280_FILTER_COEFF_OFF;

    /* Enable all measurements, without filter */
    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL;

    bme280_rslt = bme280_set_sensor_settings(settings_sel, &bme280_dev);
    if (bme280_rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to set sensor settings (code %+d).", bme280_rslt);
        return bme280_rslt;
    }
    return bme280_rslt;
}

int8_t bme280_get_measurement(struct bme280_data *comp_data) {
    int8_t rslt;
	
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280_dev);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to set sensor mode (code %+d).", rslt);
        return rslt;
    }
    
    /* Wait for the measurement to complete */
    bme280_dev.delay_us(40000, bme280_dev.intf_ptr);
    rslt = bme280_get_sensor_data(BME280_ALL, comp_data, &bme280_dev);
    if (rslt != BME280_OK)
    {
        fprintf(stderr, "Failed to get sensor data (code %+d).", rslt);
        return rslt;
    }
}