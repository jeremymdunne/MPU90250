#ifndef _MPU9250_HPP_
#define _MPU9250_HPP_

#include <Arduino.h> 
#include <SPI.h> 

#include "MPU9250_defs.h"


#define MPU9250_SERIAL_DEBUG 

#define MPU9250_SPI_FREQ 1000000 // 1MHz for everything, may be able to use 20

typedef enum{
    MPU9250_GYRO_FS_250DPS = 0, 
    MPU9250_GYRO_FS_500DPS = 1, 
    MPU9250_GYRO_FS_1000DPS = 2, 
    MPU9250_GYRO_FS_2000DPS = 3
} MPU9250_gyro_fs_sel; 

typedef enum{ 
    MPU9250_ACCEL_FS_2G = 0,
    MPU9250_ACCEL_FS_4G = 1, 
    MPU9250_ACCEL_FS_8G = 2,
    MPU9250_ACCEL_FS_16G = 3
} MPU9250_accel_fs_sel; 

typedef enum{
    MPU9250_CLKSEL_INTERNAL = 0, 
    MPU9250_CLKSEL_AUTO = 1, 
    MPU9250_CLKSEL_STOP = 7
} MPU9250_clck_sel; 

typedef enum{
    MPU9250_OK = 0, 
    MPU9250_COMMUNICATION_FAIL,
    MPU9250_ID_MISMATCH,
    MPU9250_MAG_ID_MISMATCH,
    MPU9250_MAG_OVERFLOW_SOFT_ERROR

} MPU9250_status_t; 

class MPU9250{
public: 
    MPU9250_status_t begin(int cs, bool en_mag = true); 
    
    MPU9250_status_t set_gyro_fs(MPU9250_gyro_fs_sel gyro_fs); 

    MPU9250_status_t set_accel_fs(MPU9250_accel_fs_sel accel_fs);

    MPU9250_status_t get_accel(float *accel); 

    MPU9250_status_t get_gyro(float *gyro); 

    MPU9250_status_t get_mag(float *mag); 

    MPU9250_status_t get_temp(float *temp); 

    MPU9250_status_t get_all(float *accel, float *gyro, float *mag, float *temp); 

    MPU9250_status_t get_accel_gyro(float *accel, float *gyro);

    MPU9250_status_t get_accel_temp_gyro(float *accel, float *gyro, float *temp);

    MPU9250_status_t set_gyro_offsets(float *gyro);

    MPU9250_status_t set_accel_offsets(float *accel); 

    MPU9250_status_t enable_FIFO(); 

    uint get_FIFO_count(); 

    MPU9250_status_t sleep(); 

    MPU9250_status_t wake(); 

    MPU9250_status_t reset(); 

private: 
    int _cs; 
    MPU9250_gyro_fs_sel _gyro_fs; 
    MPU9250_accel_fs_sel _accel_fs; 
    SPISettings _spi_settings; 
    bool _mag_en = false; 

    float _gyro_sensitivity = 131; 
    float _accel_sensitivity = 16384;  

    float _mag_adjustments[3] ={0,0,0}; 

    MPU9250_status_t check_id(); 
    
    MPU9250_status_t set_clock(MPU9250_clck_sel clk_sel = MPU9250_CLKSEL_AUTO);  

    MPU9250_status_t enable_mag(); 

    MPU9250_status_t write_reg(byte reg, byte *buffer, uint length); 

    MPU9250_status_t write_reg(byte reg, byte value); 

    MPU9250_status_t read_reg(byte reg, byte *buffer, uint length); 

    MPU9250_status_t write_mag_reg(byte reg, byte value); 

    MPU9250_status_t read_mag_reg(byte reg, byte *buffer, uint length);

    MPU9250_status_t mag_setup_read(byte reg, uint length); 


    void select(){
        SPI.beginTransaction(_spi_settings); 
        digitalWrite(_cs, LOW); 
    }

    void release(){
        digitalWrite(_cs,HIGH); 
        SPI.endTransaction(); 
    }

}; 

#endif 