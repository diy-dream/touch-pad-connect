 /*
  * The library is not extensively tested and only
  * meant as a simple explanation and for inspiration.
  * NO WARRANTY of ANY KIND is provided.
  */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_drv_icm20948_twi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_log.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* Pins to connect MPU. Pinout is different for nRF51 DK and nRF52 DK
 * and therefore I have added a conditional statement defining different pins
 * for each board. This is only for my own convenience. 
 */
#if 1
#define ICM20948_TWI_SCL_PIN 47 //(P1.15) 
#define ICM20948_TWI_SDA_PIN 46 //(P1.14) 
#else
#define ICM20948_TWI_SCL_PIN 11
#define ICM20948_TWI_SDA_PIN 12
#endif


#define ICM20948_TWI_BUFFER_SIZE       16 // 14 byte buffers will suffice to read acceleromter, gyroscope and temperature data in one transmission.
#define ICM20948_TWI_TIMEOUT           1000000 //default value 10000


static const nrf_drv_twi_t m_twi_instance = NRF_DRV_TWI_INSTANCE(0);
volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;

uint8_t twi_tx_buffer[3062];


static void nrf_drv_icm20948_twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch(p_event->xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_TX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXTX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    twi_rx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXRX:
                    twi_rx_done = true;
                    break;
                default:
                    break;
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}



/**
 * @brief TWI initialization.
 * Just the usual way. Nothing special here
 */
uint32_t nrf_drv_icm20948_init(void)
{
    uint32_t err_code;
    
    const nrf_drv_twi_config_t twi_icm20948_config = {
       .scl                = ICM20948_TWI_SCL_PIN,
       .sda                = ICM20948_TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST,
       .clear_bus_init     = false
    };
    
    err_code = nrf_drv_twi_init(&m_twi_instance, &twi_icm20948_config, nrf_drv_icm20948_twi_event_handler, NULL);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    nrf_drv_twi_enable(&m_twi_instance);

    return NRF_SUCCESS;
}

/**
 * @brief TWI dezinitialization.
 * Just the usual way. Nothing special here
 */
uint32_t nrf_drv_icm20948_dinit(void)
{
    return NRF_SUCCESS;
}

// The TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
// Hence we need to merge the MPU register address with the buffer and then transmit all as one transmission
static void merge_register_and_data(uint8_t * new_buffer, uint8_t reg, uint8_t * p_data, uint32_t length)
{
    new_buffer[0] = reg;
    memcpy((new_buffer + 1), p_data, length);
}

uint32_t nrf_drv_icm20948_write_registers2(uint8_t slave_addr, uint8_t reg_addr, uint32_t length, unsigned char *data)
{
    // This burst write function is not optimal and needs improvement.
    // The new SDK 11 TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
    uint32_t err_code;
    uint32_t timeout = ICM20948_TWI_TIMEOUT;

    // Merging MPU register address and p_data into one buffer.
    merge_register_and_data(twi_tx_buffer, reg_addr, data, length);

    // Setting up transfer
    nrf_drv_twi_xfer_desc_t xfer_desc;
    xfer_desc.address = slave_addr;
    xfer_desc.type = NRF_DRV_TWI_XFER_TX;
    xfer_desc.primary_length = length + 1;
    xfer_desc.p_primary_buf = twi_tx_buffer;

    // Transferring
    err_code = nrf_drv_twi_xfer(&m_twi_instance, &xfer_desc, 0);

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    return err_code;
}

uint32_t nrf_drv_icm20948_write_registers(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
    // This burst write function is not optimal and needs improvement.
    // The new SDK 11 TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
    uint32_t err_code;
    uint32_t timeout = ICM20948_TWI_TIMEOUT;

#ifdef TWI_DEBUG
    NRF_LOG_RAW_INFO("icm20948_write_registers(SlaveAddr = 0x%x, RegisterAddr = 0x%x, RegisterLen = %d, RegisterValue = 0x%x)\n", Address, RegisterAddr, RegisterLen, *RegisterValue);
    NRF_LOG_FLUSH();
#endif   

    // Merging MPU register address and p_data into one buffer.
    merge_register_and_data(twi_tx_buffer, RegisterAddr, RegisterValue, RegisterLen);

    // Setting up transfer
    nrf_drv_twi_xfer_desc_t xfer_desc;
    xfer_desc.address = Address;
    xfer_desc.type = NRF_DRV_TWI_XFER_TX;
    xfer_desc.primary_length = RegisterLen + 1;
    xfer_desc.p_primary_buf = twi_tx_buffer;

    // Transferring
    err_code = nrf_drv_twi_xfer(&m_twi_instance, &xfer_desc, 0);

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    return err_code;
}


uint32_t nrf_drv_icm20948_read_registers(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
    uint32_t err_code;
    uint32_t timeout = ICM20948_TWI_TIMEOUT;
#ifdef TWI_DEBUG
    NRF_LOG_RAW_INFO("icm20948_read_registers(SlaveAddr = 0x%x, RegisterAddr = 0x%x, RegisterLen = %d)\n", Address, RegisterAddr, RegisterLen);
    NRF_LOG_FLUSH();
#endif   
    err_code = nrf_drv_twi_tx(&m_twi_instance, Address, &RegisterAddr, 1, true);
    if(err_code != NRF_SUCCESS){
#ifdef TWI_DEBUG
        NRF_LOG_RAW_INFO("Error on nrf_drv_twi_tx = %d\n", err_code);
        NRF_LOG_FLUSH();
#endif         
        return err_code;
    }


    while((!twi_tx_done) && --timeout);
    if(!timeout){
#ifdef TWI_DEBUG
        NRF_LOG_RAW_INFO("Timeout of twi tx\n");
        NRF_LOG_FLUSH();
#endif               
        return NRF_ERROR_TIMEOUT;
    }
    twi_tx_done = false;

    err_code = nrf_drv_twi_rx(&m_twi_instance, Address, RegisterValue, RegisterLen);
    if(err_code != NRF_SUCCESS){
#ifdef TWI_DEBUG
        NRF_LOG_RAW_INFO("Error on nrf_drv_twi_rx = %d\n", err_code);
        NRF_LOG_FLUSH();
#endif         
        return err_code;
    }

    timeout = ICM20948_TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    if(!timeout){
#ifdef TWI_DEBUG
        NRF_LOG_RAW_INFO("Timeout of twi rx\n");
        NRF_LOG_FLUSH();
#endif               
        return NRF_ERROR_TIMEOUT;
    }
    twi_rx_done = false;

    return err_code;
}

/**
  @}
*/
