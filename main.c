/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_common.h"
#include "nrf_pwr_mgmt.h"
//#include "nrf_drv_timer.h"
//#include "bsp.h"

//files needed for log
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//files needed for ble
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_lbs.h"
#include "ble_icm20948.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"

/* InvenSense drivers and utils */
#include "Invn/Devices/HostSerif.h"
#include "Invn/Devices/DeviceIcm20948.h"
#include "Invn/EmbUtils/Message.h"

#include "Icm20948.h"
#include "Icm20948MPUFifoControl.h"
#include "Icm20948DataBaseDriver.h"
#include "Icm20948Transport.h"
#include "Icm20948Defs.h"

#include "idd_io_hal.h"

#define PIN_INT 45 //(P1.13)

#define ODR_NONE       0 /* Asynchronous sensors don't need to have a configured ODR */

/*
 * Set O/1 to start the following sensors in this example
 * NB: In case you are using IddWrapper (USE_IDDWRAPPER = 1), the following compile switch will have no effect.
 */
#define USE_RAW_ACC 0   /* USE FOR WOM */
#define USE_RAW_GYR 0
#define USE_GRV     0
#define USE_CAL_ACC 0
#define USE_CAL_ACC 0
#define USE_CAL_GYR 0
#define USE_CAL_MAG 0
#define USE_UCAL_GYR 0
#define USE_UCAL_MAG 0
#define USE_RV      0    /* requires COMPASS*/
#define USE_GEORV   1    /* requires COMPASS*/
#define USE_ORI     0    /* requires COMPASS*/
#define USE_STEPC   0
#define USE_STEPD   0
#define USE_SMD     0
#define USE_BAC     0
#define USE_TILT    0
#define USE_PICKUP  0
#define USE_GRAVITY 0
#define USE_LINACC  0
#define USE_B2S     0

/************Parameters for BLE********************/
#define DEVICE_NAME                     "IMU_ICM20948"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                255                                     /**< The advertising interval (in units of 0.625 ms; this value corresponds to 159 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   2                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

BLE_ICM20948_DEF(m_icm20948);                                                   /**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
APP_TIMER_DEF(m_timer);
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

//const nrf_drv_timer_t TIMER_0 = NRF_DRV_TIMER_INSTANCE(0);

uint64_t tick_us = 0;

/*
 * Sensor to start in this example
 */
static const struct {
	uint8_t  type;
	uint32_t period_us;
} sensor_list[] = {
#if USE_RAW_ACC
	{ INV_SENSOR_TYPE_RAW_ACCELEROMETER, 50000 /* 20 Hz */ },
#endif
#if USE_RAW_GYR
	{ INV_SENSOR_TYPE_RAW_GYROSCOPE,     50000 /* 20 Hz */ },
#endif
#if USE_CAL_ACC
	{ INV_SENSOR_TYPE_ACCELEROMETER, 50000 /* 20 Hz */ },
#endif
#if USE_CAL_GYR
	{ INV_SENSOR_TYPE_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_CAL_MAG
	{ INV_SENSOR_TYPE_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_GYR
	{ INV_SENSOR_TYPE_UNCAL_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_MAG
	{ INV_SENSOR_TYPE_UNCAL_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_GRV
	{ INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_RV
	{ INV_SENSOR_TYPE_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_GEORV
	{ INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_ORI
	{ INV_SENSOR_TYPE_ORIENTATION, 50000 /* 20 Hz */ },
#endif
#if USE_STEPC
	{ INV_SENSOR_TYPE_STEP_COUNTER, ODR_NONE },
#endif
#if USE_STEPD
	{ INV_SENSOR_TYPE_STEP_DETECTOR, ODR_NONE},
#endif
#if USE_SMD
	{ INV_SENSOR_TYPE_SMD, ODR_NONE},
#endif
#if USE_BAC
	{ INV_SENSOR_TYPE_BAC, ODR_NONE},
#endif
#if USE_TILT
	{ INV_SENSOR_TYPE_TILT_DETECTOR, ODR_NONE},
#endif
#if USE_PICKUP
	{ INV_SENSOR_TYPE_PICK_UP_GESTURE, ODR_NONE},
#endif
#if USE_GRA
	{ INV_SENSOR_TYPE_GRAVITY, 50000 /* 20 Hz */},
#endif
#if USE_LINACC
	{ INV_SENSOR_TYPE_LINEAR_ACCELERATION, 50000 /* 20 Hz */},
#endif
#if USE_B2S
	{ INV_SENSOR_TYPE_B2S, ODR_NONE},
#endif
};

#ifdef INV_MSG_ENABLE
static void msg_printer(int level, const char * str, va_list ap);
#endif

/*
 * Flag set from device irq handler 
 */
static volatile int irq_from_device;

static icm20948_data_t icm20948Data;

/*
 * Last time at which 20948 IRQ was fired
 */
static volatile uint32_t last_irq_time = 0;

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

static void check_rc(int rc);

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{ICM20948_UUID_BASE, m_icm20948.uuid_type}};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void icm20948_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{

}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t            err_code;
    ble_icm20948_init_t   init_icm20948  = {0};
    nrf_ble_qwr_init_t    qwr_init  = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize MPU.
    init_icm20948.icm20948_write_handler = icm20948_write_handler;

    err_code = ble_icm20948_init(&m_icm20948, &init_icm20948);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t  err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            nrf_drv_gpiote_in_event_enable(PIN_INT, true);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            nrf_drv_gpiote_in_event_enable(PIN_INT, false);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
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

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/* 
 * High resolution sleep implementation for Icm20948.
 * Used at initilization stage. ~100us is sufficient.
 */
void inv_icm20948_sleep_us(int us)
{
    /*
     * You may provide a sleep function that blocks the current programm
     * execution for the specified amount of us
     */
    nrf_delay_us(us);
}
/*
 * Time implementation for Icm20948.
 */
uint64_t inv_icm20948_get_time_us(void)
{
    /*
     * You may provide a time function that return a monotonic timestamp in us
     */
    
    return tick_us;
}

const char * activityName(int act)
{
    switch(act) {
    case INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_BEGIN:          return "BEGIN IN_VEHICLE";
    case INV_SENSOR_BAC_EVENT_ACT_WALKING_BEGIN:             return "BEGIN WALKING";
    case INV_SENSOR_BAC_EVENT_ACT_RUNNING_BEGIN:             return "BEGIN RUNNING";
    case INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_BEGIN:          return "BEGIN ON_BICYCLE";
    case INV_SENSOR_BAC_EVENT_ACT_TILT_BEGIN:                return "BEGIN TILT";
    case INV_SENSOR_BAC_EVENT_ACT_STILL_BEGIN:               return "BEGIN STILL";
    case INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_END:            return "END IN_VEHICLE";
    case INV_SENSOR_BAC_EVENT_ACT_WALKING_END:               return "END WALKING";
    case INV_SENSOR_BAC_EVENT_ACT_RUNNING_END:               return "END RUNNING";
    case INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_END:            return "END ON_BICYCLE";
    case INV_SENSOR_BAC_EVENT_ACT_TILT_END:                  return "END TILT";
    case INV_SENSOR_BAC_EVENT_ACT_STILL_END:                 return "END STILL";
    default:                                                 return "unknown activity!";
    }
}

/*
 * Callback called upon sensor event reception
 * This function is called in the same function than inv_device_poll()
 */
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg)
{
  //INV_MSG(INV_MSG_LEVEL_INFO, "new sensor event !");
  if(event->status == INV_SENSOR_STATUS_DATA_UPDATED) {

        switch(INV_SENSOR_ID_TO_TYPE(event->sensor)) {
        case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
        case INV_SENSOR_TYPE_RAW_GYROSCOPE:
                INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (lsb): %llu %d %d %d", inv_sensor_str(event->sensor),
                                event->timestamp,
                                (int)event->data.raw3d.vect[0],
                                (int)event->data.raw3d.vect[1],
                                (int)event->data.raw3d.vect[2]);
                break;
        case INV_SENSOR_TYPE_ACCELEROMETER:
        case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
        case INV_SENSOR_TYPE_GRAVITY:
                INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mg): %d %d %d", inv_sensor_str(event->sensor),
                                (int)(event->data.acc.vect[0]*1000),
                                (int)(event->data.acc.vect[1]*1000),
                                (int)(event->data.acc.vect[2]*1000));
                break;
        case INV_SENSOR_TYPE_GYROSCOPE:
                INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mdps): %d %d %d", inv_sensor_str(event->sensor),
                                (int)(event->data.gyr.vect[0]*1000),
                                (int)(event->data.gyr.vect[1]*1000),
                                (int)(event->data.gyr.vect[2]*1000));
                break;
        case INV_SENSOR_TYPE_MAGNETOMETER:
                INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (nT): %d %d %d", inv_sensor_str(event->sensor),
                                (int)(event->data.mag.vect[0]*1000),
                                (int)(event->data.mag.vect[1]*1000),
                                (int)(event->data.mag.vect[2]*1000));
                break;
        case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
                INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mdps): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
                                (int)(event->data.gyr.vect[0]*1000),
                                (int)(event->data.gyr.vect[1]*1000),
                                (int)(event->data.gyr.vect[2]*1000),
                                (int)(event->data.gyr.bias[0]*1000),
                                (int)(event->data.gyr.bias[1]*1000),
                                (int)(event->data.gyr.bias[2]*1000));
                break;
        case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
                INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (nT): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
                                (int)(event->data.mag.vect[0]*1000),
                                (int)(event->data.mag.vect[1]*1000),
                                (int)(event->data.mag.vect[2]*1000),
                                (int)(event->data.mag.bias[0]*1000),
                                (int)(event->data.mag.bias[1]*1000),
                                (int)(event->data.mag.bias[2]*1000));
                break;
        case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
        case INV_SENSOR_TYPE_ROTATION_VECTOR:
        case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
                INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (e-3): %d %d %d %d ", inv_sensor_str(event->sensor),
                                (int)(event->data.quaternion.quat[0]*1000),
                                (int)(event->data.quaternion.quat[1]*1000),
                                (int)(event->data.quaternion.quat[2]*1000),
                                (int)(event->data.quaternion.quat[3]*1000));
                break;
        case INV_SENSOR_TYPE_ORIENTATION:
              icm20948Data.psi    = (event->data.orientation.x);
              icm20948Data.theta  = (event->data.orientation.y);
              icm20948Data.phi    = (event->data.orientation.z);
              ble_icm20948_data_change(m_conn_handle, &m_icm20948, &icm20948Data);
              /*INV_MSG(INV_MSG_LEVEL_INFO, ",%d,%d,%d",
                                (int)(event->data.orientation.x),
                                (int)(event->data.orientation.y),
                                (int)(event->data.orientation.z));
                /*INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (e-3): %d %d %d %d ", inv_sensor_str(event->sensor),
                                (int)(event->data.orientation.x),
                                (int)(event->data.orientation.y),
                                (int)(event->data.orientation.z));*/
                break;
        case INV_SENSOR_TYPE_BAC:
                INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : %d %s", inv_sensor_str(event->sensor),
                                event->data.bac.event, activityName(event->data.bac.event));
                break;
        case INV_SENSOR_TYPE_STEP_COUNTER:
                INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : %lu", inv_sensor_str(event->sensor),
                                (unsigned long)event->data.step.count);
                break;
        case INV_SENSOR_TYPE_PICK_UP_GESTURE:
        case INV_SENSOR_TYPE_STEP_DETECTOR:
        case INV_SENSOR_TYPE_SMD:
        case INV_SENSOR_TYPE_B2S:
        case INV_SENSOR_TYPE_TILT_DETECTOR:
        default:
                INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : ...", inv_sensor_str(event->sensor));
                break;
        }
    }
}

static bool send_1_0 = true;

/*
 * Callback called upon rising edge on external interrupt line
 */
void ext_interrupt_cb(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
#if 1
    last_irq_time = inv_icm20948_get_time_us();
    irq_from_device = 1;
#elif 1
    icm20948_data_t data;

    if(send_1_0){
        data.phi    = 0xFFFF;
        data.psi    = 0xFFFF;
        data.theta  = 0xFFFF;
        send_1_0 = 0;
        ble_icm20948_data_change(m_conn_handle, &m_icm20948, &data);
    }else{
        data.phi    = 0x0000;
        data.psi    = 0x0000;
        data.theta  = 0x0000;
        send_1_0 = 1;
        ble_icm20948_data_change(m_conn_handle, &m_icm20948, &data);
    }
#endif
}

void log_init(void){
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void interrupt_init()
{
    uint32_t err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    //in_config.pull = NRF_GPIO_PIN_PULLUP;
    in_config.pull = NRF_GPIO_PIN_NOPULL;

    err_code = nrf_drv_gpiote_in_init(PIN_INT, &in_config, ext_interrupt_cb);
    APP_ERROR_CHECK(err_code);
}

/*void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
    //INV_MSG(INV_MSG_LEVEL_INFO, "Timer = %d us", app_timer_cnt_get()/ 32.768);
    tick_us++;
}*/

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
/*static void timers_init(void)
{
    uint32_t time_us = 20000; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;

    //Configure TIMER_0 for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_0, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_us_to_ticks(&TIMER_0, time_us);

    nrf_drv_timer_extended_compare(
         &TIMER_0, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMER_0);
}*/

static bool on_off = true;

void timer_handler(void * p_context)
{
    //NRF_LOG_INFO("%d", app_timer_cnt_get()/ 32.768);
#if 0
    if(on_off){
        inv_icm20948_set_chip_power_state(icm20948_instance, CHIP_AWAKE, 1);
        on_off = false;
    }else{
        inv_icm20948_set_chip_power_state(icm20948_instance, CHIP_AWAKE, 0);
        on_off = true;
    }
#endif
}

/**@brief Function starting the internal LFCLK oscillator.
 *
 * @details This is needed by RTC1 which is used by the Application Timer
 *          (When SoftDevice is enabled the LFCLK is always running and this is not needed).
 */
static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init1(void)
{
    lfclk_request();

    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_timer, APP_TIMER_MODE_REPEATED, timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_timer, APP_TIMER_TICKS(2000), NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init2(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/*
 * A listener onject will handle sensor events
 */
static inv_sensor_listener_t sensor_listener = {
    sensor_event_cb, /* callback that will receive sensor events */
    (void *)0xDEAD   /* some pointer passed to the callback */
};

/*
 * States for icm20948 device object
 */
static inv_device_icm20948_t device_icm20948;
static uint8_t dmp3_image[] = {
        //0
 #include "Invn/Images/icm20948_img.dmp3a.h"
};

void output_voltage_setup(void)
{
    // Configure UICR_REGOUT0 register only if it is set to default value.
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) ==
        (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_1V8 << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        // System reset is needed to update UICR registers.
        NVIC_SystemReset();
    }
}

/*
 * serif_hal object that abstract low level serial interface between host and device
 */
int main(void)
{
    ret_code_t err_code;
    int rc = 0;
    inv_device_t * device; /* just a handy variable to keep the handle to device object */
    uint8_t whoami;

    if (NRF_POWER->MAINREGSTATUS &
       (POWER_MAINREGSTATUS_MAINREGSTATUS_High << POWER_MAINREGSTATUS_MAINREGSTATUS_Pos))
    {
        output_voltage_setup();
    }

    // Initialize the application timer module.
    //log_init();
    timers_init1();
    //timers_init2();
    power_management_init();
#if 1
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    advertising_start();
#endif

#if 1
    interrupt_init();
#endif

#ifdef INV_MSG_ENABLE
    /* Setup message logging */
    INV_MSG_SETUP(INV_MSG_LEVEL_MAX, msg_printer);
#endif
#if 1
    INV_MSG(INV_MSG_LEVEL_INFO, "###################################");
    INV_MSG(INV_MSG_LEVEL_INFO, "#        ICM20948 example         #");
    INV_MSG(INV_MSG_LEVEL_INFO, "###################################");

    /*
     * Open serial interface (SPI or I2C) before playing with the device
     */
    /* call low level drive initialization here... */
    rc += inv_host_serif_open(idd_io_hal_get_serif_instance_i2c());
    /*
     * Create ICM20948 Device 
     * Pass to the driver:
     * - reference to serial interface object,
     * - reference to listener that will catch sensor events,
     */
    //inv_device_icm20948_init2(&device_icm20948, &serif_instance, &sensor_listener, dmp3_image, sizeof(dmp3_image));
    inv_device_icm20948_init(&device_icm20948, idd_io_hal_get_serif_instance_i2c(), &sensor_listener, dmp3_image, sizeof(dmp3_image));
    
    /*
     * Simply get generic device handle from Icm20948 Device
     */
    device = inv_device_icm20948_get_base(&device_icm20948);
    /*
     * Just get the whoami
     */
    rc += inv_device_whoami(device, &whoami);
    /* ... do something with whoami */

    /*
     * Configure and initialize the Icm20948 device
     */
    rc += inv_device_setup(device);
    /*
     * Now that device is ready, you must call inv_device_poll() function
     * periodically or upon interrupt.
     * The poll function will check for sensor events, and notify, if any,
     * by means of the callback from the listener that was provided on device init.
     */
    rc += inv_device_poll(device);

    /*
     * Now that Icm20948 device was inialized, we can proceed with DMP image loading
     * This step is mandatory as DMP image are not store in non volatile memory
     */
    INV_MSG(INV_MSG_LEVEL_INFO, "Load DMP3 image");
    rc = inv_device_load(device, NULL, dmp3_image, sizeof(dmp3_image), true /* verify */, NULL);
    check_rc(rc);

    uint64_t available_sensor_mask; /* To keep track of available sensors*/
    unsigned i;
    /*
     * Check sensor availibitlity
     * if rc value is 0, it means sensor is available,
     * if rc value is INV_ERROR or INV_ERROR_BAD_ARG, sensor is NA
     */
    available_sensor_mask = 0;
    for(i = 0; i < sizeof(sensor_list)/sizeof(sensor_list[0]); ++i) {
        const int rc = inv_device_ping_sensor(device, sensor_list[i].type);
        INV_MSG(INV_MSG_LEVEL_INFO, "Ping %s %s", inv_sensor_2str(sensor_list[i].type), (rc == 0) ? "OK" : "KO");
        if(rc == 0) {
            available_sensor_mask |= (1ULL << sensor_list[i].type);
        }
    }

    /*
     * Start all available sensors from the sensor list
     */
    for(i = 0; i < sizeof(sensor_list)/sizeof(sensor_list[0]); ++i) {
        if(available_sensor_mask & (1ULL << sensor_list[i].type)) {
            INV_MSG(INV_MSG_LEVEL_INFO, "Starting %s @ %u us", inv_sensor_2str(sensor_list[i].type), sensor_list[i].period_us);
            rc = inv_device_set_sensor_period_us(device, sensor_list[i].type, sensor_list[i].period_us);
            check_rc(rc);
            rc += inv_device_start_sensor(device, sensor_list[i].type);
            check_rc(rc);
        }
    }

    rc += inv_device_poll(device);

    while(1){
#if 1
        if (irq_from_device) {
            rc = inv_device_poll(device);
            check_rc(rc);

            if(rc >= 0) {
                __disable_irq();
                irq_from_device = 0;
                __enable_irq();
            }
        }
#endif
        nrf_pwr_mgmt_run();
    }

    /*
     * Stop accelerometer sensor
     */
    rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_RAW_ACCELEROMETER);
    rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_RAW_GYROSCOPE);
    /*
     * Shutdown everything.
     */
    rc += inv_device_cleanup(device);
    /*
     * Close serial interface link
     */
    /* call low level drive de-initialization here... */
    rc += inv_host_serif_close(idd_io_hal_get_serif_instance_i2c());

    return rc;
#elif 1
    /*
     * Open serial interface (SPI or I2C) before playing with the device
     */
    /* call low level drive initialization here... */
    rc += inv_host_serif_open(idd_io_hal_get_serif_instance_i2c());
    /*
     * Create ICM20948 Device 
     * Pass to the driver:
     * - reference to serial interface object,
     * - reference to listener that will catch sensor events,
     */
    //inv_device_icm20948_init2(&device_icm20948, &serif_instance, &sensor_listener, dmp3_image, sizeof(dmp3_image));
    inv_device_icm20948_init(&device_icm20948, idd_io_hal_get_serif_instance_i2c(), &sensor_listener, dmp3_image, sizeof(dmp3_image));
    
    /*
     * Simply get generic device handle from Icm20948 Device
     */
    device = inv_device_icm20948_get_base(&device_icm20948);
    /*
     * Just get the whoami
     */
    rc += inv_device_whoami(device, &whoami);
    /* ... do something with whoami */

    /*
     * Configure and initialize the Icm20948 device
     */
    rc += inv_device_setup(device);
    /*
     * Now that device is ready, you must call inv_device_poll() function
     * periodically or upon interrupt.
     * The poll function will check for sensor events, and notify, if any,
     * by means of the callback from the listener that was provided on device init.
     */
    rc += inv_device_poll(device);

    uint64_t available_sensor_mask; /* To keep track of available sensors*/
    unsigned i;
    /*
     * Check sensor availibitlity
     * if rc value is 0, it means sensor is available,
     * if rc value is INV_ERROR or INV_ERROR_BAD_ARG, sensor is NA
     */
    available_sensor_mask = 0;
    for(i = 0; i < sizeof(sensor_list)/sizeof(sensor_list[0]); ++i) {
        const int rc = inv_device_ping_sensor(device, sensor_list[i].type);
        INV_MSG(INV_MSG_LEVEL_INFO, "Ping %s %s", inv_sensor_2str(sensor_list[i].type), (rc == 0) ? "OK" : "KO");
        if(rc == 0) {
            available_sensor_mask |= (1ULL << sensor_list[i].type);
        }
    }

    /*
     * Start all available sensors from the sensor list
     */
    for(i = 0; i < sizeof(sensor_list)/sizeof(sensor_list[0]); ++i) {
        if(available_sensor_mask & (1ULL << sensor_list[i].type)) {
            INV_MSG(INV_MSG_LEVEL_INFO, "Starting %s @ %u us", inv_sensor_2str(sensor_list[i].type), sensor_list[i].period_us);
            rc = inv_device_set_sensor_period_us(device, sensor_list[i].type, sensor_list[i].period_us);
            check_rc(rc);
            rc += inv_device_start_sensor(device, sensor_list[i].type);
            check_rc(rc);
        }
    }

    //inv_icm20948_set_chip_power_state(icm20948_instance, CHIP_AWAKE, 1);

    //inv_icm20948_set_chip_power_state(icm20948_instance, CHIP_LP_ENABLE, 1);

    inv_icm20948_set_int1_assertion(icm20948_instance, 0);

    //inv_icm20948_sleep_mems(icm20948_instance);

    for(;;){
        nrf_pwr_mgmt_run();
    }
#elif 1
    for(;;){
        nrf_pwr_mgmt_run();
    }    
#endif

}

static void check_rc(int rc)
{
    if(rc == -1) {
        INV_MSG(INV_MSG_LEVEL_INFO, "BAD RC=%d", rc);
        while(1);
    }
}

#if INV_MSG_ENABLE
/*
* Printer function for message facility
*/
static void msg_printer(int level, const char * str, va_list ap){
    
    static char out_str[256]; /* static to limit stack usage */
    unsigned idx = 0;
    const char * ptr = out_str;
    const char * s[INV_MSG_LEVEL_MAX] = {
            "",    // INV_MSG_LEVEL_OFF
            "[E] ", // INV_MSG_LEVEL_ERROR
            "[W] ", // INV_MSG_LEVEL_WARNING
            "[I] ", // INV_MSG_LEVEL_INFO
            "[V] ", // INV_MSG_LEVEL_VERBOSE
            "[D] ", // INV_MSG_LEVEL_DEBUG
    };

    idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
    if(idx >= (sizeof(out_str)))
            return;
    idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
    if(idx >= (sizeof(out_str)))
            return;
    idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\n");
    if(idx >= (sizeof(out_str)))
            return;

    NRF_LOG_RAW_INFO("%s", ptr);
    NRF_LOG_FLUSH();
}
#endif

/** @} */