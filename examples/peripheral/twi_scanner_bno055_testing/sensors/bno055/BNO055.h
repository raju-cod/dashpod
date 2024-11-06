#ifndef BNO055_H
#define BNO055_H

#include "nrf_delay.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "sdk_errors.h"
#include "app_error.h"
#include "twi_init.h"




#define BNO055_ADDRESS 0x28  //slave address 
#define BNO055_OPR_MODE_ADDR 0x3D //operating mode selction
#define BNO055_PWR_MODE_ADDR 0x3E // power mode
#define BNO055_SYS_TRIGGER_ADDR 0x3F // system trigger 
#define BNO055_AXIS_MAP_CONFIG_ADDR 0x41 
#define BNO055_AXIS_MAP_SIGN_ADDR 0x42
#define BNO055_CALIB_STAT_ADDR 0x35

#define OPERATION_MODE_CONFIG 0x00
#define OPERATION_MODE_NDOF 0x0C //NDOF mode


void setup_bno055(const nrf_drv_twi_t* m_twi,uint8_t addr);

void i2c_scanner_bno055(const nrf_drv_twi_t* m_twi,uint8_t addr);

void quaternion_to_angles(float qw, float qx, float qy, float qz);

void displayCalStatus(const nrf_drv_twi_t* m_twi,uint8_t addr);

void read_euler(const nrf_drv_twi_t* m_twi, uint8_t addr);

void read_quaternion(const nrf_drv_twi_t* m_twi, uint8_t addr);

void read_accel_values(const nrf_drv_twi_t* m_twi, uint8_t addr);

void read_gyro_values(const nrf_drv_twi_t* m_twi, uint8_t addr);

void read_mag_values(const nrf_drv_twi_t* m_twi, uint8_t addr);

#endif