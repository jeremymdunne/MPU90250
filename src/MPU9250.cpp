#include "MPU9250.hpp"


MPU9250_status_t MPU9250::begin(int cs_pin, bool en_mag){
    _cs = cs_pin; 
    _mag_en = en_mag; 
    // setup the spi settings 
    _spi_settings = SPISettings(MPU9250_SPI_FREQ, MSBFIRST, SPI_MODE_0);
    // setup the digital pin 
    pinMode(_cs, OUTPUT); 
    digitalWrite(_cs, HIGH); 

    // reset the chip 
    reset();
    // check the id 
    MPU9250_status_t id_status = check_id(); 
    if(id_status != MPU9250_OK) return id_status; 
    // set the clock mode 
    set_clock(MPU9250_CLKSEL_AUTO); 
    
    if(_mag_en){
        Serial.println("Enabling mag!!!");
        enable_mag(); 
    }

    return MPU9250_OK; 

}

MPU9250_status_t MPU9250::set_gyro_fs(MPU9250_gyro_fs_sel gyro_fs){
    // set the gyro full scale range 
    // preserve other settings 
    byte gyro_config_read; 
    read_reg(MPU9250_REGISTER_GYRO_CONFIG, &gyro_config_read, 1); 
    // append 
    byte mask = 0x03<<3; 
    byte gyro_config_write = (gyro_config_read & ~mask) | (gyro_fs<<3); 
    Serial.println(gyro_config_write, BIN); 
    write_reg(MPU9250_REGISTER_GYRO_CONFIG, &gyro_config_write, 1); 

    // set the scale 
    switch(gyro_fs){
        case(MPU9250_GYRO_FS_250DPS): 
            _gyro_sensitivity = 131; 
        break; 
        case(MPU9250_GYRO_FS_500DPS): 
            _gyro_sensitivity = 65.5; 
        break; 
        case(MPU9250_GYRO_FS_1000DPS): 
            _gyro_sensitivity = 32.8; 
        break; 
        case(MPU9250_GYRO_FS_2000DPS): 
            _gyro_sensitivity = 16.4; 
        break; 
    }

    return MPU9250_OK; 
}


MPU9250_status_t MPU9250::set_accel_fs(MPU9250_accel_fs_sel accel_fs){
    // set the gyro full scale range 
    // preserve other settings 
    byte accel_config_read; 
    read_reg(MPU9250_REGISTER_ACCEL_CONFIG, &accel_config_read, 1); 
    // append 
    byte mask = 0x03<<3; 
    byte accel_config_write = (accel_config_read & ~mask) | (accel_fs<<3); 
    Serial.println(accel_config_write, BIN); 
    write_reg(MPU9250_REGISTER_ACCEL_CONFIG, accel_config_write); 
    // set the scale 
    switch(accel_fs){
        case(MPU9250_ACCEL_FS_2G): 
            _accel_sensitivity = 16384; 
        break; 
        case(MPU9250_ACCEL_FS_4G): 
            _accel_sensitivity = 8192; 
        break; 
        case(MPU9250_ACCEL_FS_8G): 
            _accel_sensitivity = 4096; 
        break; 
        case(MPU9250_ACCEL_FS_16G): 
            _accel_sensitivity = 2048; 
        break; 
    }
    return MPU9250_OK; 
}

MPU9250_status_t MPU9250::get_accel(float *accel){
    // read the accel data 
    // convert into float 
    byte accel_data[6]; 
    read_reg(MPU9250_REGISTER_ACCEL_XOUT_H, accel_data, 6); 
    // convert the data 
    for(uint i = 0; i < 3; i ++){
        accel[i] = (int16_t)(accel_data[i*2]<<8 | accel_data[i*2+1]) / _accel_sensitivity; 
    }
    return MPU9250_OK;
}

MPU9250_status_t MPU9250::get_gyro(float *gyro){
    // read the gyro data 
    // convert into float 
    byte gyro_data[6]; 
    read_reg(MPU9250_REGISTER_GYRO_XOUT_H, gyro_data, 6); 
    // convert the data 
    for(uint i = 0; i < 3; i ++){
        gyro[i] = (int16_t)(gyro_data[i*2]<<8 | gyro_data[i*2+1]) / _gyro_sensitivity; 
    }
    return MPU9250_OK;
}

MPU9250_status_t MPU9250::get_mag(float *mag){
    byte data[8]; 
    read_reg(MPU9250_REGISTER_EXT_SENS_DATA_00, data, 8); 
    // check the overvlow 
    if(data[7] & 1<<3 != 0){
        // overflow, report error to not use 
        return MPU9250_MAG_OVERFLOW_SOFT_ERROR;
    }
    for(uint i = 0; i < 3; i ++){
        mag[i] = (int16_t)(data[i*2+1] | (data[i*2+2]<<8)) * 0.15; 
    }
    // check the i2c master status
    byte mst_read; 
    read_reg(MPU9250_REGISTER_I2C_MST_STATUS, &mst_read, 1); 
    if(mst_read & 0x01 == 1){
        // need to perform a reset b/c a NACK was recieved
        byte user_cntrl_read; 
        read_reg(MPU9250_REGISTER_USER_CTRL, &user_cntrl_read, 1); 
        // append 
        byte mask = 0x01<<5; 
        byte user_cntrl_write = (user_cntrl_read & ~mask) | (1<<5); 
        user_cntrl_write |= 1 << 1; 
        write_reg(MPU9250_REGISTER_USER_CTRL, user_cntrl_write); 
    }
    return MPU9250_OK; 
}

MPU9250_status_t MPU9250::get_temp(float *temp){
    // read the temp data 
    // convert into float 
    byte temp_data[2]; 
    read_reg(MPU9250_REGISTER_TEMP_OUT_H, temp_data, 2); 
    *temp = (int16_t)(temp_data[0]<<8|temp_data[1])/333.87 + 21.0;  
    return MPU9250_OK;
} 

MPU9250_status_t MPU9250::get_all(float *accel, float *gyro, float *mag, float *temp){
    // read everything 
    // read a continuous 21 bytes 
    byte data[22]; 
    read_reg(MPU9250_REGISTER_ACCEL_XOUT_H, data, 22);
    // accel, temp, gyro in that order
    for(uint i = 0; i < 3; i ++){
        accel[i] = (int16_t)(data[i*2]<<8 | data[i*2+1]) / _accel_sensitivity; 
        gyro[i] = (int16_t)(data[8+i*2]<<8 | data[9+i*2]) / _gyro_sensitivity; 
        mag[i] = (int16_t)(data[16+i*2]<<8 | data[15+i*2]) * 0.15; 
    }
    *temp = (int16_t)(data[6]<<8|data[7])/333.87 + 21.0;  
    // check the mag overflow 
    Serial.print('\t');
    Serial.print(data[14], BIN);
    Serial.print('\t');
    Serial.print(data[21]); 

    if(data[21] & 1<<3 != 0) return MPU9250_MAG_OVERFLOW_SOFT_ERROR; 
    // check the i2c master status
    byte mst_read; 
    read_reg(MPU9250_REGISTER_I2C_MST_STATUS, &mst_read, 1); 
    if(mst_read & 0x01 == 1){
        // need to perform a reset b/c a NACK was recieved
        byte user_cntrl_read; 
        read_reg(MPU9250_REGISTER_USER_CTRL, &user_cntrl_read, 1); 
        // append 
        byte mask = 0x01<<5; 
        byte user_cntrl_write = (user_cntrl_read & ~mask) | (1<<5); 
        user_cntrl_write |= 1 << 1; 
        write_reg(MPU9250_REGISTER_USER_CTRL, user_cntrl_write); 
        Serial.println("RECIEVED NACK, RESET!");
    }
    

    return MPU9250_OK;
}

MPU9250_status_t MPU9250::get_accel_gyro(float *accel, float *gyro){
    // get both the accel and gyro data 
    byte data[14]; 
    read_reg(MPU9250_REGISTER_ACCEL_XOUT_H, data, 14); 
    // accel, temp, gyro in that order
    for(uint i = 0; i < 3; i ++){
        accel[i] = (int16_t)(data[i*2]<<8 | data[i*2+1]) / _accel_sensitivity; 
        gyro[i] = (int16_t)(data[8+i*2]<<8 | data[8+i*2+1]) / _gyro_sensitivity; 
    }
    return MPU9250_OK;
}

MPU9250_status_t MPU9250::get_accel_temp_gyro(float *accel, float *gyro, float *temp){
     // get both the accel, temp, gyro data 
    byte data[14]; 
    read_reg(MPU9250_REGISTER_ACCEL_XOUT_H, data, 14); 
    // accel, temp, gyro in that order
    for(uint i = 0; i < 3; i ++){
        accel[i] = (int16_t)(data[i*2]<<8 | data[i*2+1]) / _accel_sensitivity; 
        gyro[i] = (int16_t)(data[8+i*2]<<8 | data[8+i*2+1]) / _gyro_sensitivity; 
    }
    *temp = (int16_t)(data[6]<<8|data[7])/333.87 + 21.0;  
    return MPU9250_OK;
}

MPU9250_status_t MPU9250::set_gyro_offsets(float *gyro){
    return MPU9250_OK;
}

MPU9250_status_t MPU9250::set_accel_offsets(float *accel){
    return MPU9250_OK;
} 

MPU9250_status_t MPU9250::enable_FIFO(){
    return MPU9250_OK;
}

uint MPU9250::get_FIFO_count(){
    return MPU9250_OK;
}


MPU9250_status_t MPU9250::sleep(){
    // put the device to sleep 
    // preserve other values
    byte pwr_read; 
    read_reg(MPU9250_REGISTER_PWR_MGMT_1, &pwr_read, 1); 
    // append 
    byte mask = 0x01<<6; 
    byte pwr_write = (pwr_read & ~mask) | (1<<6); 
    Serial.println(pwr_write, BIN); 
    write_reg(MPU9250_REGISTER_PWR_MGMT_1, pwr_write); 
    return MPU9250_OK;
} 

MPU9250_status_t MPU9250::wake(){
    // wake the device to sleep 
    // preserve other values
    byte pwr_read; 
    read_reg(MPU9250_REGISTER_PWR_MGMT_1, &pwr_read, 1); 
    // append 
    byte mask = 0x01<<6; 
    byte pwr_write = (pwr_read & ~mask) | (0<<6); 
    Serial.println(pwr_write, BIN); 
    write_reg(MPU9250_REGISTER_PWR_MGMT_1, pwr_write); 
    return MPU9250_OK;
}


MPU9250_status_t MPU9250::reset(){
    // reset the device 
    // send a hardware reset and wait for the bit to clear 
    // hardware reset so we don't care about stored values 
    byte command = 1<<7; 
    write_reg(MPU9250_REGISTER_PWR_MGMT_1, command); 
    delay(1); 
    byte read; 
    read_reg(MPU9250_REGISTER_PWR_MGMT_1, &read, 1); 
    // TODO sometime implement a max timeout 
    while((read & 0x80) != 0){
        Serial.println(read,BIN);
        delay(1); 
        read_reg(MPU9250_REGISTER_PWR_MGMT_1, &read, 1); 
    }
    return MPU9250_OK; 
}

MPU9250_status_t MPU9250::check_id(){
    // read the WHO_AM_I reg to check dev id 
    byte id; 
    MPU9250_status_t status = read_reg(MPU9250_REGISTER_WHO_AM_I, &id, 1); 
    if(status != MPU9250_OK) return MPU9250_OK; 
    // compare the id 
    if(id != 0x71){
        // not defualt 
        #ifdef MPU9250_SERIAL_DEBUG 
            Serial.print("\nDefault ID not found! Expected ");
            Serial.print(0x71, HEX); 
            Serial.print(" Recieved: "); 
            Serial.print(id, HEX); 
        #endif 
        if(id == 0 | id == 0xFF){
            // likely connection issue 
            return MPU9250_COMMUNICATION_FAIL; 
        }
        return MPU9250_ID_MISMATCH; 
    }
    return MPU9250_OK; 
}

MPU9250_status_t MPU9250::set_clock(MPU9250_clck_sel clk_sel){
    // set the clock 
    // need to preserve other settings 
    byte pwr_read; 
    read_reg(MPU9250_REGISTER_PWR_MGMT_1, &pwr_read, 1); 
    // append 
    byte mask = 0x03<<0; 
    byte pwr_write = (pwr_read & ~mask) | (clk_sel<<0); 
    Serial.println(pwr_write, BIN); 
    write_reg(MPU9250_REGISTER_PWR_MGMT_1, pwr_write); 
    return MPU9250_OK;
}

MPU9250_status_t MPU9250::enable_mag(){
    // enable the mag 
    // need to enable i2c master 
    Serial.println("Enable!");
    byte user_cntrl_read; 
    read_reg(MPU9250_REGISTER_USER_CTRL, &user_cntrl_read, 1); 
    Serial.println(user_cntrl_read, BIN); 
    // append 
    byte mask = 0x01<<5; 
    byte user_cntrl_write = (user_cntrl_read & ~mask) | (1<<5); 
    user_cntrl_write |= 1 << 1; 
    Serial.println(user_cntrl_write, BIN); 
    write_reg(MPU9250_REGISTER_USER_CTRL, user_cntrl_write); 
    delay(10);

    // set master control 
    byte mst_ctrl = 0x1D; 
    write_reg(MPU9250_REGISTER_I2C_MST_CTRL, mst_ctrl); 
    // check the id 
    delay(10);
    byte mag_id; 
    read_mag_reg(MPU9250_AK8963_REGISTER_DEVICE_ID, &mag_id, 1);
    Serial.println("Mag ID: "); 
    Serial.print(mag_id, HEX);  
    // if(mag_id != 0x48) return MPU9250_MAG_ID_MISMATCH; 
    Serial.println("Good to Go!");

    // reset the mag 
    // write_mag_reg(MPU9250_AK8963_REGISTER_CNTL_2, 0x01); 
    // wait a bit 
    delay(10); 
    // read the adjustment values 
    // write_mag_reg(MPU9250_AK8963_REGISTER_CNTL, 0x0F); 
    // byte adj[3]; 
    // read_mag_reg(MPU9250_AK8963_REGISTER_X_SENSITIVITY_ADJUSTMENT, adj, 3); 
    // for(uint i = 0; i < 3; i ++){
    //     _mag_adjustments[i] = (float)(adj[i] - 128)/256.0 + 1.0; 
    //     Serial.println(_mag_adjustments[i]); 
    // }
    // // go into continuous mode 
    // // power down first 
    // write_mag_reg(MPU9250_AK8963_REGISTER_CNTL,0); 
    // delay(50); 
    // continuous mode 
    write_mag_reg(MPU9250_AK8963_REGISTER_CNTL,0x16); 
    delay(50); 
    // setup read mode 
    write_reg(MPU9250_REGISTER_I2C_MST_DELAY_CTRL, 0x81); 
    write_reg(MPU9250_REGISTER_I2C_SLV4_CTRL, 0x01); 
    mag_setup_read(MPU9250_AK8963_REGISTER_STATUS_1, 8); 
    delay(10);
    byte st; 
    read_reg(MPU9250_REGISTER_I2C_MST_STATUS, &st, 1); 
    Serial.print("I2C MASTER STATUS: "); 
    Serial.println(st, BIN);
    return MPU9250_OK;
}


MPU9250_status_t MPU9250::write_reg(byte reg, byte *buffer, uint length){
    // write the device 
    select(); 
    SPI.transfer(reg); 
    while(length > 0){
        SPI.transfer(*buffer);
        *buffer ++; 
        length --; 
    }
    release(); 
    return MPU9250_OK; 
}

MPU9250_status_t MPU9250::write_reg(byte reg, byte value){
    // write the device 
    select(); 
    SPI.transfer(reg); 
    SPI.transfer(value); 
    release(); 
    return MPU9250_OK; 
}


MPU9250_status_t MPU9250::read_reg(byte reg, byte *buffer, uint length){
    // write the device 
    select(); 
    SPI.transfer(reg | 0x80); // first bit must be a 1 for a read op 
    while(length > 0){
        *buffer = SPI.transfer(0);
        *buffer ++; 
        length --; 
    }
    release(); 
    return MPU9250_OK; 
}


MPU9250_status_t MPU9250::write_mag_reg(byte reg, byte value){
    // write to the mag 
    // a little complicated due to the pass through mode 
    // set the address, reg, value, then trigger a write 
    byte write_byte = MPU9250_AK8963_I2C_ADDR; 
    write_reg(MPU9250_REGISTER_I2C_SLV0_ADDR, write_byte);
    write_reg(MPU9250_REGISTER_I2C_SLV0_REG, reg); 
    write_reg(MPU9250_REGISTER_I2C_SLV0_DO, value);  
    // trigger 
    byte init = 1<<7|1; 
    write_reg(MPU9250_REGISTER_I2C_SLV0_CTRL, init); 
    return MPU9250_OK;
}

MPU9250_status_t MPU9250::read_mag_reg(byte reg, byte *buffer, uint length){
    // read from the mag 
    // a little complicated due to the pass through mode 
    // set the address, reg, length, then trigger a read 
    mag_setup_read(reg, length); 
    // delay some 
    delay(10); 
    read_reg(MPU9250_REGISTER_EXT_SENS_DATA_00, buffer, length); 
    return MPU9250_OK; 
}

MPU9250_status_t MPU9250::mag_setup_read(byte reg, uint length){
    // setup and trigger a read operation 
    write_reg(MPU9250_REGISTER_I2C_SLV0_ADDR, 1<<7|MPU9250_AK8963_I2C_ADDR);
    write_reg(MPU9250_REGISTER_I2C_SLV0_REG, reg); 
    write_reg(MPU9250_REGISTER_I2C_SLV0_CTRL, (1<<7|length)); 
    return MPU9250_OK; 
} 
