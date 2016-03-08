/*
 * u8g_wrapper.h
 *
 *  Created on: Sep 22, 2015
 *      Author: Ed
 */
#include "i2c.h"
#include "i2c_wrapper.h"

#include <debug_print.h>
#include <timer.h>
#include <string.h>

#define GET_SHARED_GLOBAL(x, g) asm volatile("ldw %0, dp[" #g "]":"=r"(x)::"memory")
#define SET_SHARED_GLOBAL(g, v) asm volatile("stw %0, dp[" #g "]"::"r"(v):"memory")

unsafe client i2c_master_if g_i2c_wr;
unsafe client i2c_master_if g_i2c_rd;
unsafe {
    //This doesn't work (compiler crash) so use ASM below..
    //client i2c_master_if * unsafe g_i2c_ptr = (client i2c_master_if * unsafe) &g_i2c;
}

//Initialised the global interface ID
void i2c_setup_glob_interface(client i2c_master_if i_i2c){
  SET_SHARED_GLOBAL(g_i2c_wr, i_i2c);
  SET_SHARED_GLOBAL(g_i2c_rd, i_i2c);
}


unsigned i2c_write(unsigned char mode, unsigned arg_val, const unsigned char *arg_ptr){
  i2c_res_t result;
  size_t num_bytes_sent, tot = 0;
  unsigned char write_array[1025];
  if(mode == I2C_DATA_MODE){
    write_array[0] = mode;
    memcpy(&write_array[1], arg_ptr, arg_val);
    arg_val += 1;
    unsafe{
      result = g_i2c_wr.write(DEVICE_ADDR, write_array, arg_val, num_bytes_sent, 1);
    }
    tot = num_bytes_sent;
  }
  else{
    for (int i=0; i<arg_val;i++){
        write_array[0] = mode;
        write_array[1] = *arg_ptr;
        unsafe{
            result = g_i2c_wr.write(DEVICE_ADDR, write_array, 2, num_bytes_sent, 1);
            delay_microseconds(I2C_IDLE_TIME_US);
        }
        tot += num_bytes_sent;
        arg_ptr++;
    }
  }
  //debug_printf("\nWrote %d bytes, mode %x,result = %d\n", tot, mode, result);
  return !result;
}


