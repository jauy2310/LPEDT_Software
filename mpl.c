/*
 * mpl.c
 *
 * Implementation file for the MPL3115A2 sensor module
 *
 *  Created on: Apr 2, 2024
 *      Author: Jake Uyechi
 */



/*********************************************************
 * Includes
 ********************************************************/
#include <as.h>
#include <stdio.h>
#include <string.h>
#include "sl_i2cspm.h"
#include "sl_i2cspm_instances.h"
#include "sl_i2cspm_sensor_config.h"
#include "sl_sleeptimer.h"

/*********************************************************
 * Macros
 ********************************************************/
// define the AS7331 configuration settings
#define MPL_CONFIG_I2C_DEVICE                     	(sl_i2cspm_sensor)
#define MPL_CONFIG_SLAVE_ADDRESS                  	0x60
#define MPL_CONFIG_SLAVE_ADDRESS_READ             	(AS_CONFIG_SLAVE_ADDRESS << 1 || 1)
#define MPL_CONFIG_SLAVE_ADDRESS_WRITE            	(AS_CONFIG_SLAVE_ADDRESS << 1 || 0)
#define MPL_CONFIG_DEVICE_ID                      	0xC4

// define macros for special registers
#define MPL_REG_STATUS                            	0x00

// output register addresses
#define MPL_REG_OUT_P_MSB							0x01
#define MPL_REG_OUT_P_CSB							0x02
#define MPL_REG_OUT_P_LSB							0x03
#define MPL_REG_OUT_T_MSB							0x04
#define MPL_REG_OUT_T_LSB							0x05

// data ready status register
#define MPL_REG_DR_STATUS							0x06

// output delta register addresses
#define MPL_REG_OUT_P_DELTA_MSB						0x07
#define MPL_REG_OUT_P_DELTA_CSB						0x08
#define MPL_REG_OUT_P_DELTA_LSB						0x09
#define MPL_REG_OUT_T_DELTA_MSB						0x0A
#define MPL_REG_OUT_T_DELTA_LSB						0x0B

// device ID register
#define MPL_REG_WHO_AM_I                            0x0C

// FIFO registers
#define MPL_REG_F_STATUS							0x0D
#define MPL_REG_F_DATA								0x0E
#define MPL_REG_F_SETUP								0x0F
#define MPL_REG_TIME_DLY							0x10

// system mode register
#define MPL_REG_SYSMOD								0x11

// interrupt source register
#define MPL_REG_INT_SOURCE							0x12

// PT data configuration register
#define MPL_REG_PT_DATA_CFG							0x13

// barometric input registers
#define MPL_REG_BAR_IN_MSB							0x14
#define MPL_REG_BAR_IN_LSB							0x15

// pressure/temperature target value registers
#define MPL_REG_P_TGT_MSB							0x16
#define MPL_REG_P_TGT_LSB							0x17
#define MPL_REG_T_TGT								0x18

// pressure/temperature window values
#define MPL_REG_P_WND_MSB							0x19
#define MPL_REG_P_WND_LSB							0x1A
#define MPL_REG_T_WND								0x1B

// minimum and maximum values for pressure and temperature
#define MPL_REG_P_MIN_MSB							0x1C
#define MPL_REG_P_MIN_CSB							0x1D
#define MPL_REG_P_MIN_LSB							0x1E
#define MPL_REG_T_MIN_MSB							0x1F
#define MPL_REG_T_MIN_LSB							0x20
#define MPL_REG_P_MAX_MSB							0x21
#define MPL_REG_P_MAX_CSB							0x22
#define MPL_REG_P_MAX_LSB							0x23
#define MPL_REG_T_MAX_MSB							0x24
#define MPL_REG_T_MAX_LSB							0x25

// control registers
#define MPL_REG_CTRL_REG1							0x26
#define MPL_REG_CTRL_REG2							0x27
#define MPL_REG_CTRL_REG3							0x28
#define MPL_REG_CTRL_REG4							0x29
#define MPL_REG_CTRL_REG5							0x2A

// user offset registers
#define MPL_REG_OFF_P								0x2B
#define MPL_REG_OFF_T								0x2C
#define MPL_REG_OFF_H								0x2D

// define FIFO modes
typedef enum {
	FIFO_DISABLED,
	FIFO_CIRCULARBUFF,
	FIFO_FULLSTOP
} FIFO_MODE_t;
/*********************************************************
 * Local Variables
 ********************************************************/
// sensor variables
float mpl_temperature, mpl_pressure, mpl_altitude;

// i2c buffer
#define I2C_BUFFER_LENGTH 16
uint8_t mpl_write_buf[I2C_BUFFER_LENGTH];
uint8_t mpl_read_buf[I2C_BUFFER_LENGTH];

/*********************************************************
 * Local Functions
 ********************************************************/
/**
 * MPL7331_transaction()
 *
 * Constructs an I2C sequence for a single transaction between the master (EFR32)
 * and slave (MPL7331).
 *
 * @param flag              The type of I2C transaction being made
 * @param writeCmd          The array of write addresses to send from master to slave
 * @param writeLen          The length of the write array
 * @param readCmd           The array of read addresses to store data from slave to master
 * @param readLen           The length of the read array
 *
 * @return                  The return status of the I2C transaction
 */
static I2C_TransferReturn_TypeDef MPL3115A2_transaction(uint16_t flag,
														uint8_t *writeCmd,
														size_t writeLen,
														uint8_t *readCmd,
														size_t readLen)
{
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;

	seq.addr = MPL_CONFIG_SLAVE_ADDRESS << 1;
	seq.flags = flag;

	switch (flag) {
		// Send the write command from writeCmd
		case I2C_FLAG_WRITE:
			seq.buf[0].data = writeCmd;
			seq.buf[0].len  = writeLen;

			break;

			// Receive data into readCmd of readLen
		case I2C_FLAG_READ:
			seq.buf[0].data = readCmd;
			seq.buf[0].len  = readLen;

			break;

			// Send the write command from writeCmd
			// and receive data into readCmd of readLen
		case I2C_FLAG_WRITE_READ:
			seq.buf[0].data = writeCmd;
			seq.buf[0].len  = writeLen;

			seq.buf[1].data = readCmd;
			seq.buf[1].len  = readLen;

			break;

			// default case
			default:
				return i2cTransferUsageFault;
	}

	// Perform the transfer and return status from the transfer
	ret = I2CSPM_Transfer(MPL_CONFIG_I2C_DEVICE, &seq);

	return ret;
}

/**
 * flush_buffers
 *
 * Clears I2C buffers
 */
void mpl_flush_buffers(void)
{
    memset(mpl_write_buf, 0, I2C_BUFFER_LENGTH);
    memset(mpl_read_buf, 0, I2C_BUFFER_LENGTH);
}

/*********************************************************
 * Global Functions
 ********************************************************/
void mpl_dev_id(void)
{
	I2C_TransferReturn_TypeDef ret;
	mpl_flush_buffers();
	mpl_write_buf[0] = MPL_REG_WHO_AM_I;

	// Wait for sensor to become ready
	sl_sleeptimer_delay_millisecond(80);

	// Check for device presence  and compare device ID
	ret = MPL3115A2_transaction(I2C_FLAG_WRITE_READ, mpl_write_buf, 1, mpl_read_buf, 8);

	// Print Device ID
	printf("%-10s Device ID: 0x%02X\r\n", "MPL3115A2", mpl_read_buf[0]);

	// Make sure transfer was successful
	EFM_ASSERT(ret == i2cTransferDone);

	// Check the Received Device ID
	EFM_ASSERT(mpl_read_buf[0] == MPL_CONFIG_DEVICE_ID);

	// clear the buffers
	mpl_flush_buffers();
}

void mpl_init(void)
{
	return;
}

void mpl_process_action(void)
{
	return;
}
