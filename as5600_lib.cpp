/**
 * @file as5600.cpp
 * @author Denys Khmil
 * @brief This file contents all the as5600 library functions
 */
#include "as5600_lib.h"

/**
 * @brief as5600 sensor constructor
 * @param _i2c: i2c instance pointer.
 * @param _address: sensor address.
 */
as5600::as5600(I2C_HandleTypeDef *_i2c, uint8_t _address){
    /*Save i2c instsnce and sensor address*/
    this->i2c = _i2c;
    this->address = _address;
    
    /*read and save limit values*/
    this->getStartPos();
    this->getStopPos();
    this->getMaxAngle();
}


/**
 * @brief as5600 sensor constructor
 * @param _i2c: i2c instance pointer.
 */
as5600::as5600(I2C_HandleTypeDef *_i2c){
    /*Save i2c instsnce*/
    this->i2c = _i2c;
    this->address = 0x36;

    /*read and save limit values*/
    this->getStartPos();
    this->getStopPos();
    this->getMaxAngle();
}


/**
 * @brief Reads 12 bits from sensor memory
 * @param address: address in memory.
 */
uint16_t as5600::read12bit(uint8_t address){
    uint8_t data[2];
    HAL_I2C_Mem_Read(this->i2c, (this->address << 1), address, I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
    uint16_t value = ((data[0]&0x0f) << 8)|(data[1]);
    return value;
}


/**
 * @brief Set actual angle as zero point
 */
void as5600::setAngleAsStart(){
    /*read actual angle value*/
    uint8_t data[2];
    HAL_I2C_Mem_Read(this->i2c, (this->address << 1), RAW_ANGLE_ADR, I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
    data[0] = data[0]&0x0f;
    /*write to startPos register*/
    HAL_I2C_Mem_Write(this->i2c, (this->address << 1), ZPOS_ADR, I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
}

/**
 * @brief Set actual angle as stop position
 */
void as5600::setAngleAsStop(){
    /*read actual angle value*/
    uint8_t data[2];
    HAL_I2C_Mem_Read(this->i2c, (this->address << 1), RAW_ANGLE_ADR, I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
    data[0] = data[0]&0x0f;
    /*Write to stopPos register*/
    HAL_I2C_Mem_Write(this->i2c, (this->address << 1), MPOS_ADR, I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
}


/**
 * @brief Read configuration register from the sensor.
 * @retval 16bit register.
 */
uint16_t as5600::readConfig(){
    uint8_t buf[2];
    HAL_I2C_Mem_Read(this->i2c, (this->address << 1), CONF_ADR, I2C_MEMADD_SIZE_8BIT, buf, 2, 1000);
    return ((buf[0] << 8)|(buf[1]));
}


/**
 * @brief Write sensor configuration 
 * @param conf_msb: 8 MSB bits.
 * @param conf_lsb: 8 LSB bits.
 */
void as5600::writeConfig(uint8_t conf_msb, uint8_t conf_lsb){
    uint8_t buf[2];
    buf[0] = conf_msb;
    buf[1] = conf_lsb;
    HAL_I2C_Mem_Write(this->i2c, (this->address << 1), CONF_ADR, I2C_MEMADD_SIZE_8BIT, buf, 2, 1000);
}


/**
 * @brief Check magnet function
 * @retval 0 - Magnet was detected, 1 - magnet is too weak, 2 - magnet is too strong 
 */
uint8_t as5600::checkMagnet(){
    uint8_t buffer;
    HAL_I2C_Mem_Read(this->i2c, (this->address << 1), STATUS_ADR, I2C_MEMADD_SIZE_8BIT, &buffer, 1, 1000);
    if (buffer & 0x20){
        return 0;
    }
    else if (buffer & 0x10){
        return 1;
    }
    else{
        return 2;
    }
}


/**
 * @brief Read ZPOS register (Start position)
 * @retval 16bit register value 
 */
uint16_t as5600::getStartPos(){
    uint16_t val = this->read12bit(ZPOS_ADR);
    /*save value for future calculations*/
    this->startPos = val;
    return val;
}


/**
 * @brief Read MPOS register (Stop position)
 * @retval 16bit register value 
 */
uint16_t as5600::getStopPos(){
    uint16_t val = this->read12bit(MPOS_ADR);
    /*save value for future calculations*/
    this->stopPos = val;
    return val;
}


/**
 * @brief Read MANG register (Max angle)
 * @retval 16bit register value 
 */
uint16_t as5600::getMaxAngle(){
    uint16_t val = this->read12bit(MANG_ADR);
    /*save value for future calculations*/
    this->maxAngle = val;
    return val;
}


/**
 * @brief Get actual angle function
 * @retval Actual angle value (float)
 */
float as5600::getAngle(){
    uint16_t ang = this->read12bit(ANGLE_ADR);
    float multiplier = 0;

    /*calculate multiplier based on settings*/
    if (this->maxAngle > 0)
    {
        if (this->startPos == 0){ //start position isnt set
            multiplier = (this->maxAngle*0.088)/4096;
        }
        else{
            multiplier = (this->maxAngle*0.088)/4096;
        }
    }
    else{
        if((this->startPos == 0) && (this->stopPos == 0)){
            multiplier = 0.0878;
        }
        else if ((this->startPos > 0 ) && (this->stopPos == 0)){
            multiplier = ((360 * 0.0878) - (this->startPos * 0.0878)) / 4096;
        }
        else if ((this->startPos == 0 ) && (this->stopPos > 0)){
            multiplier = (this->stopPos*0.0878) / 4096;
        }
        else if ((this->startPos > 0 ) && (this->stopPos > 0)){
            multiplier = ((this->stopPos*0.0878)-(this->startPos * 0.0878))/ 4096;
        }
            
    }
    return (float)ang*multiplier;
}


/**
 * @brief Get raw angle function
 * @retval Angle value (float)
 */
float as5600::getRawAngle(){
    uint16_t ang = this->read12bit(RAW_ANGLE_ADR);
    /*data range is 0 - 4095, 360deg/4095 = 0.088deg*/
    float angle = (float)ang*0.088;
    return angle;
}