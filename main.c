/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_twim.h"
#include "nrfx_saadc.h"
#include "app_scheduler.h"

#include "sensors.h"
#include "bme280_defs.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_COMPANY_IDENTIFIER          0xFFFF                             /**< Manufacturer identifier 0xFFFF -> default, no specific company */
#define APP_SHORT_LOCAL_NAME            "nagiandev"                        /**< Shortened local name */
#define APP_DEVICE_TYPE                 0x0001                             /**< Manufacturer specific device type: temperature/humidity/pressure/illuminance/battery */
#define APP_BEACON_INFO_LENGTH          0x0B                               /**< Total length of information advertised by the Beacon (11 bytes). */

#define APP_BEACON_INFO_TEMP_OFFSET     2
#define APP_BEACON_INFO_RH_OFFSET       4
#define APP_BEACON_INFO_PRESS_OFFSET    6
#define APP_BEACON_INFO_ILLU_OFFSET     8
#define APP_BEACON_INFO_BATT_OFFSET     10

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define ADV_BUFFERS_USED                2

#define SCHED_MAX_EVENT_DATA_SIZE       10
#define SCHED_QUEUE_SIZE                10

#define TWI_SCL_PIN                     26
#define TWI_SDA_PIN                     25
#define VSENS_EN_N_PIN                  4

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

#define MEAS_INTERVAL                   15000                              /**< Time interval for taking sensor measurements (in ms) */

APP_TIMER_DEF(m_meas_timer_id);                                        	   /**< Sensor measurement timer. */

//static ble_advdata_t        m_raw_adv_data;                                      /**< Unencoded advertising data */
static ble_gap_adv_params_t m_adv_params;                                        /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;       /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata_buff1[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< 1st buffer for storing an encoded advertising set. */
static uint8_t              m_enc_advdata_buff2[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< 2nd buffer for storing an encoded advertising set. */

/* TWI instance */
static const nrfx_twim_t m_twi = NRFX_TWIM_INSTANCE(0);

/* Structure containing the BME280 measurements */
struct bme280_data bme280_meas;

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data[ADV_BUFFERS_USED] =
{
    {
        .adv_data =
        {
            .p_data = m_enc_advdata_buff1,
            .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
        },
        .scan_rsp_data =
        {
            .p_data = NULL,
            .len    = 0

        }
    },
    {
        .adv_data =
        {
            .p_data = m_enc_advdata_buff2,
            .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
        },
        .scan_rsp_data =
        {
            .p_data = NULL,
            .len    = 0

        }
    }
};

static uint8_t ui_buff_index = 0;


//static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
/*{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};*/

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    0x00, 0x00,          // Dummy temperature (real value will be filled-in later)
    0x00, 0x00,          // Dummy relative humidity (real value will be filled-in later)
    0x00, 0x00,          // Dummy atmospheric pressure (real value will be filled-in later)
    0x00, 0x00,          // Dummy illuminance (real value will be filled-in later)
    0x00                 // Dummy battery level (real value will be filled-in later)
};

/**@brief Function to move the sensor data to the m_beacon_info structure
 */
static void update_m_beacon_info(void)
{
    int16_t temperature = (int16_t)(bme280_meas.temperature * 10);
    uint16_t humidity = (uint16_t)(bme280_meas.humidity * 10);
    uint16_t pressure = (uint16_t)(bme280_meas.pressure / 10);
    uint16_t illuminance = 0; // (uint16_t)();
    printf("Temperature: %d, Humidity: %d, Pressure: %d\n", temperature, humidity, pressure);
    // Temperature measurement
    m_beacon_info[APP_BEACON_INFO_TEMP_OFFSET] = (temperature & 0xFF00) >> 8;
    m_beacon_info[APP_BEACON_INFO_TEMP_OFFSET + 1] = (temperature & 0x00FF);
    // Relative humidity measurement
    m_beacon_info[APP_BEACON_INFO_RH_OFFSET] = (humidity & 0xFF00) >> 8;
    m_beacon_info[APP_BEACON_INFO_RH_OFFSET + 1] = (humidity & 0x00FF);
    // Pressure measurement
    m_beacon_info[APP_BEACON_INFO_PRESS_OFFSET] = (pressure & 0xFF00) >> 8;
    m_beacon_info[APP_BEACON_INFO_PRESS_OFFSET + 1] = (pressure & 0x00FF);
    // Illuminance measurement
    m_beacon_info[APP_BEACON_INFO_ILLU_OFFSET] = (illuminance & 0xFF00) >> 8;
    m_beacon_info[APP_BEACON_INFO_ILLU_OFFSET + 1] = (illuminance & 0x00FF);
    // Battery level measurement
    m_beacon_info[APP_BEACON_INFO_BATT_OFFSET] = 0;
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function that builts raw (unencoded) advertising data
 *
 */
/*static void advdata_build(void)
{
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&m_raw_adv_data, 0, sizeof(m_raw_adv_data));

    m_raw_adv_data.name_type             = BLE_ADVDATA_NO_NAME;
    m_raw_adv_data.flags                 = flags;
    m_raw_adv_data.p_manuf_specific_data = &manuf_specific_data;
}*/

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_16(major_value);
    m_beacon_info[index++] = LSB_16(major_value);

    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
#endif

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 0;       // Never time out.

    err_code = ble_advdata_encode(&advdata, m_adv_data[ui_buff_index].adv_data.p_data, &m_adv_data[ui_buff_index].adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data[ui_buff_index], &m_adv_params);
    APP_ERROR_CHECK(err_code);

    if( err_code == NRF_SUCCESS )
    {
        ui_buff_index++;
        ui_buff_index = ui_buff_index % ADV_BUFFERS_USED;
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for updating advertising data.
 */
static void advertising_update_data(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    err_code = ble_advdata_encode(&advdata, m_adv_data[ui_buff_index].adv_data.p_data, &m_adv_data[ui_buff_index].adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data[ui_buff_index], NULL);
    APP_ERROR_CHECK(err_code);

    if( err_code == NRF_SUCCESS )
    {
        ui_buff_index++;
        ui_buff_index = ui_buff_index % ADV_BUFFERS_USED;
    }
}

/**@brief Function for starting application timers. */
static void application_timers_start(void)
{
    ret_code_t err_code;
    uint32_t meas_timer_ticks;

    meas_timer_ticks = APP_TIMER_TICKS(MEAS_INTERVAL);

    err_code = app_timer_start(m_meas_timer_id, meas_timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing GPIOs. */
static void gpio_init(void)
{
    // Configure VSENS_EN_N pin as output
    nrf_gpio_cfg_output(VSENS_EN_N_PIN);
    // Set VSENS_EN_N to 0 (enabled)
    nrf_gpio_pin_clear(VSENS_EN_N_PIN);
}

/**@brief Function for initializing the sensors. */
static void sensors_init(void)
{
    bme280_sensor_init(&m_twi);
    //max44000_config(&m_twi);
}

/**@brief Function to get a measurement from the sensors. */
static void sensors_sample(void)
{
    bme280_get_measurement(&bme280_meas);
}

/**@brief TWI events handler. */
void twi_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
}

/**@brief Function for initializing the I2C bus. */
static void twi_init(void)
{
    ret_code_t err_code;
	
    const nrfx_twim_config_t twi_config = {
       .scl                = TWI_SCL_PIN,
       .sda                = TWI_SDA_PIN,
       .frequency          = NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY,
       .interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
       .hold_bus_uninit    = NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT
    };
	
    err_code = nrfx_twim_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
	
    nrfx_twim_enable(&m_twi);
}

/**@brief Function for taking measurements from all sensors and put them ready for being used in the advertising
 *
 */
static void meas_update_scheduler_event_handler(void * p_event_data, uint16_t event_size)
{
    UNUSED_PARAMETER(p_event_data);
    UNUSED_PARAMETER(event_size);
    
    // Take sensor measurements
    sensors_sample();

    // Update advertised data
    update_m_beacon_info();
    advertising_update_data();

}

/**@brief Function for handling the sensor measurement timer timeouts.
 *
 * @details This function will be called each time the sensor
 *          measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    app_sched_event_put(NULL, 0, meas_update_scheduler_event_handler);
}

/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
	
	// Create DOF service timer.
    err_code = app_timer_create(&m_meas_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    gpio_init();
    twi_init();   // For the I2C bus
    timers_init();
    leds_init();
    power_management_init();
    sensors_init();
    sensors_sample();
    ble_stack_init();
    advertising_init();

    // Start execution.
    NRF_LOG_INFO("Beacon example started.");
    application_timers_start();
    advertising_start();

    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    // Enter main loop.
    for (;; )
    {
        app_sched_execute();
        idle_state_handle();
    }
}


/**
 * @}
 */
