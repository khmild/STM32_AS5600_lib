/**
 * @file as5600.h
 * @author Denys Khmil
 * @brief This file contents the as5600 class
 */
#ifndef AS5600_LIB
#define AS5600_LIB

#include "main.h"

typedef enum{
    ZMCO_ADR = 0x00,
    ZPOS_ADR = 0x01,
    MPOS_ADR = 0x03,
    MANG_ADR = 0x05,
    CONF_ADR= 0x07
}ConfigRegisters;

typedef enum{
    RAW_ANGLE_ADR = 0x0c,
    ANGLE_ADR = 0x0e
}OutputRegisters;

typedef enum{
    STATUS_ADR = 0x0b,
    AGC_ADR = 0x1a,
    MAGNITUDE_ADR = 0x1b
}StatusRegisters;



class as5600{
    public:
    as5600(I2C_HandleTypeDef *_i2c, uint8_t _address);
    as5600(I2C_HandleTypeDef *_i2c);
    void setAngleAsStart();
    void setAngleAsStop();
    uint8_t checkMagnet();
    float getAngle();
    float getRawAngle();

    private:
    uint16_t readConfig();
    void writeConfig(uint8_t conf_msb, uint8_t conf_lsb);
    void burnSettings();
    void burnAngle();
    uint16_t getStartPos();
    uint16_t getStopPos();
    uint16_t getMaxAngle();
    uint16_t read12bit(uint8_t address);

    I2C_HandleTypeDef *i2c;
    uint8_t address;
    uint16_t startPos;
    uint16_t stopPos;
    uint16_t maxAngle;

};

#endif