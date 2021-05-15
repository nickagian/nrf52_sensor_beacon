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


#define CONFIG_APP_MEAS_INTERVAL        60                                 /**< Time interval for taking sensor measurements [s] */

#define CONFIG_BATT_MEAS_ENABLED        1                                  /**< Enable battery measurements */
#define CONFIG_BATT_MEAS_MIN_LEVEL      1700                               /**< Voltage of 0% Battery Level [mV] */
#define CONFIG_BATT_MEAS_MAX_LEVEL      3000                               /**< Voltage of 100% Battery Level [mV] */
#define CONFIG_BATT_MEAS_POLL_INTERVAL  300                                /**< Battery Voltage Polling Interval [s] */
#define CONFIG_BATT_NOTIF_THRESHOLD     1                                  /**< Battery Level Notification Threshold [percentage point] <0-100>, set to 0 for every measurement */
#define CONFIG_BATT_MEAS_ADC_CHANNEL    0                                  /**< ADC Channel for Battery Voltage Measurements */
#define CONFIG_BATT_MEAS_ADC_DIVIDER    6                                  /**< ADC Divider for Battery Voltage Measurements */
#define CONFIG_BATT_MEAS_ADC_REFERENCE  600                                /**< ADC Reference Voltage for Battery Voltage Measurements [mV] */
#define CONFIG_BATT_MEAS_ADC_MAX_CONV   ((1 << 14) - 1)                    /**< Maximum ADC Output */

#if CONFIG_BATT_MEAS_ENABLED
// Verify SDK configuration.
STATIC_ASSERT(NRFX_SAADC_ENABLED);
APP_TIMER_DEF(m_batt_timer);                              /**< Battery measurement timer. */
static uint8_t            m_batt_meas_prev_level = 255;   /**< Previous notified battery level. */
static nrf_saadc_value_t  saadc_value = 0;                /**< Current value of the adc conversion. */
#endif // CONFIG_BATT_MEAS_ENABLED

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(2000, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement in ms (2000 ms). This value can vary between 100ms to 10.24s). */

#define APP_COMPANY_IDENTIFIER          0xFFFF                             /**< Manufacturer identifier 0xFFFF -> default, no specific company */
#define APP_DEVICE_NAME                 "niagdev"                          /**< Device name */
#define APP_DEVICE_SHORT_NAME_LEN       7                                  /**< Short device name length. IMPORTANT: has to be <= APP_DEVICE_NAME */
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

APP_TIMER_DEF(m_meas_timer);                                        	         /**< Sensor measurement timer. */

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

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    0xFF, 0xFF,          // Dummy temperature (real value will be filled-in later)
    0xFF, 0xFF,          // Dummy relative humidity (real value will be filled-in later)
    0xFF, 0xFF,          // Dummy atmospheric pressure (real value will be filled-in later)
    0xFF, 0xFF,          // Dummy illuminance (real value will be filled-in later)
    0xFF                 // Dummy battery level (real value will be filled-in later)
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
#if CONFIG_BATT_MEAS_ENABLED
    // Battery level measurement
    m_beacon_info[APP_BEACON_INFO_BATT_OFFSET] = m_batt_meas_prev_level;
#endif // CONFIG_BATT_MEAS_ENABLED
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

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t                  err_code;
    ble_advdata_t             advdata;
    uint8_t                   flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    ble_advdata_manuf_data_t  manuf_specific_data;
    ble_gap_conn_sec_mode_t   sec_mode;

    ASSERT(APP_DEVICE_SHORT_NAME_LEN <= strlen(APP_DEVICE_NAME));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)APP_DEVICE_NAME,
                                          strlen(APP_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_SHORT_NAME;
    advdata.short_name_len        = APP_DEVICE_SHORT_NAME_LEN;
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
    uint32_t                  err_code;
    ble_advdata_t             advdata;
    uint8_t                   flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    ble_advdata_manuf_data_t  manuf_specific_data;
    ble_gap_conn_sec_mode_t   sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)APP_DEVICE_NAME,
                                          strlen(APP_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_SHORT_NAME;
    advdata.short_name_len        = APP_DEVICE_SHORT_NAME_LEN;
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

    err_code = app_timer_start(m_meas_timer, APP_TIMER_TICKS(1000u * CONFIG_APP_MEAS_INTERVAL), NULL);
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

    STATIC_ASSERT(NRFX_TWIM_ENABLED);
	
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
	
    // Create sensor measurement timer.
    err_code = app_timer_create(&m_meas_timer,
                                APP_TIMER_MODE_REPEATED,
                                meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

#if CONFIG_BATT_MEAS_ENABLED
/**@brief Process ADC data for the battery level measurement.
 */
static void m_batt_meas_process(void *p_context, uint16_t size)
{
    //nrf_saadc_value_t *p_saadc_value = *(nrf_saadc_value_t *)(p_context);
    nrf_saadc_value_t measurement = *(nrf_saadc_value_t *)(p_context);
    //nrf_saadc_value_t measurement = *p_saadc_value;
    uint32_t voltage;
    uint8_t level;
    ret_code_t err_code;

    // Calculate battery voltage.
    voltage = ((uint32_t)(measurement) * CONFIG_BATT_MEAS_ADC_DIVIDER * CONFIG_BATT_MEAS_ADC_REFERENCE) / CONFIG_BATT_MEAS_ADC_MAX_CONV;

    // A simple linear approximation is sufficient when the power consumption is fairly low (< 100 mW).
    if (voltage >= CONFIG_BATT_MEAS_MAX_LEVEL)
    {
        level = 100;
    }
    else if (voltage <= CONFIG_BATT_MEAS_MIN_LEVEL)
    {
        level = 0;
    }
    else
    {
        level = 100 * (voltage - CONFIG_BATT_MEAS_MIN_LEVEL) /
                (CONFIG_BATT_MEAS_MAX_LEVEL - CONFIG_BATT_MEAS_MIN_LEVEL);
    }

    printf("Battery level: %u%% (%u mV, %u ADC)\n", level, voltage, measurement);

    /*
     * Only notify the application about the battery level if:
     * - change in the battery level since the last notification has exceeded the threshold,
     * - or if the battery level is 0.
     */
    if ((level <= ((int)(m_batt_meas_prev_level) - CONFIG_BATT_NOTIF_THRESHOLD)) ||
        (level >= ((int)(m_batt_meas_prev_level) + CONFIG_BATT_NOTIF_THRESHOLD)) ||
        (level == 0))
    {
        m_batt_meas_prev_level = level;
    }
}

/**@brief Handler that is called upon timeout of the timer for battery measurements. It will start an ADC conversion.
 */
static void m_batt_meas_timeout_handler(void* p_context)
{
    static nrf_saadc_value_t buffer;

    // Start an ADC conversion
    APP_ERROR_CHECK(nrfx_saadc_buffer_convert(&buffer, 1));
    APP_ERROR_CHECK(nrfx_saadc_sample());
}

/**@brief Handler that is called upon an ADC event (calibration done or conversion done)
 */
void m_batt_meas_saadc_event_handler(const nrfx_saadc_evt_t *p_event)
{
    switch (p_event->type)
    {
        case NRFX_SAADC_EVT_CALIBRATEDONE:
            // Perform first measurement just after calibration.
            APP_ERROR_CHECK(app_sched_event_put(&(p_event->data.done.p_buffer),
                                                 sizeof(p_event->data.done.p_buffer),
                                                 m_batt_meas_process
                                                 ));

            // The following measurements will be done at regular intervals.
            APP_ERROR_CHECK(app_timer_start(m_batt_timer,
                                            APP_TIMER_TICKS(1000u * CONFIG_BATT_MEAS_POLL_INTERVAL),
                                            NULL));
            break;

        case NRFX_SAADC_EVT_DONE:
            ASSERT(p_event->data.done.size == 1);
            
            saadc_value = *(p_event->data.done.p_buffer);
            //APP_ERROR_CHECK(app_sched_event_put(&(p_event->data.done.p_buffer),
                                                 //sizeof(p_event->data.done.p_buffer),
                                                 //m_batt_meas_process
                                                 //));

            APP_ERROR_CHECK(app_sched_event_put(&saadc_value,
                                                sizeof(saadc_value),
                                                m_batt_meas_process
                                                ));
            
            break;

        default:
            /* Ignore */
            break;
    }
}

static void m_batt_meas_init(void)
{
    nrf_saadc_channel_config_t adc_channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    nrfx_saadc_config_t adc_config = NRFX_SAADC_DEFAULT_CONFIG;
    ret_code_t status;

    // Verify the SAADC driver configuration.
    ASSERT(NRFX_SAADC_CONFIG_RESOLUTION == NRF_SAADC_RESOLUTION_14BIT);
    ASSERT(adc_channel_config.reference == NRF_SAADC_REFERENCE_INTERNAL);
    ASSERT(adc_channel_config.gain      == NRF_SAADC_GAIN1_6);

    // Enable Burst Mode if oversampling is enabled.
    adc_channel_config.burst = (NRFX_SAADC_CONFIG_OVERSAMPLE != 0) ? NRF_SAADC_BURST_ENABLED :
                                                                NRF_SAADC_BURST_DISABLED;

    status = nrfx_saadc_init(&adc_config, m_batt_meas_saadc_event_handler);
    APP_ERROR_CHECK(status);

    status = nrfx_saadc_channel_init(CONFIG_BATT_MEAS_ADC_CHANNEL, &adc_channel_config);
    APP_ERROR_CHECK(status);

    status = app_timer_create(&m_batt_timer, APP_TIMER_MODE_REPEATED, m_batt_meas_timeout_handler);
    APP_ERROR_CHECK(status);

    // The following measurements will be done at regular intervals.
    APP_ERROR_CHECK(app_timer_start(m_batt_timer,
                                    APP_TIMER_TICKS(1000u * CONFIG_BATT_MEAS_POLL_INTERVAL),
                                    NULL));
}
#endif /* CONFIG_BATT_MEAS_ENABLED */

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
    power_management_init();
    sensors_init();
    sensors_sample();
#if CONFIG_BATT_MEAS_ENABLED
    m_batt_meas_init();
#endif
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
