/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
 
#include "idd_io_hal.h"

// board drivers
#include "nrf_drv_icm20948_twi.h"

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

#define ICM_I2C_ADDR_REVA			0x68 	/* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB			0x69 	/* I2C slave address for INV device on Rev B board */
#define I2C_ADD ICM_I2C_ADDR_REVA

/* Host Serif object definition for SPI ***************************************/

static int idd_io_hal_init_i2c(void)
{
    return nrf_drv_icm20948_init();
}

static int idd_io_hal_read_reg_i2c(uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
    return nrf_drv_icm20948_read_registers(I2C_ADD, reg, rlen, rbuffer);
}

static int idd_io_hal_write_reg_i2c(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
    return nrf_drv_icm20948_write_registers(I2C_ADD, reg, wlen, wbuffer);
}

static const inv_host_serif_t serif_instance_i2c = {
    idd_io_hal_init_i2c,
    0,
    idd_io_hal_read_reg_i2c,
    idd_io_hal_write_reg_i2c,
    0,
    1024*32, /* max transaction size */
    1024*32, /* max transaction size */
    INV_HOST_SERIF_TYPE_I2C,
};

const inv_host_serif_t * idd_io_hal_get_serif_instance_i2c(void)
{
    return &serif_instance_i2c;
}
