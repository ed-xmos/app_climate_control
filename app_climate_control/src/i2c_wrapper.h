/*
 * u8g_wrapper.h
 *
 *  Created on: Sep 22, 2015
 *      Author: Ed
 */
#include <xccompat.h>

//Allows C to call XC i2c functions

#define DEVICE_ADDR         0x3c
#define I2C_CMD_MODE        0x00
#define I2C_DATA_MODE       0x40
#define SEND_STOP_BIT       1
#define I2C_IDLE_TIME_US    0

void i2c_setup_glob_interface(CLIENT_INTERFACE(i2c_master_if, i2c));
void i2c_init(void);
unsigned i2c_write(unsigned char mode, unsigned arg_val, const unsigned char *arg_ptr);
unsigned i2c_read(unsigned char mode, unsigned arg_val, unsigned char *arg_ptr);
