#ifndef LTC3219_H__
#define LTC3219_H__

#include "Arduino.h"
#include "Wire.h"

#define LED_DRV_ADDR        0x1b //ltc3219

//ltc3219 register defines
#define LTC3219_COMMAND_REGISTER  0x00
#define LTC3219_LED1_REGISTER     0x01
#define LTC3219_LED2_REGISTER     0x02
#define LTC3219_LED3_REGISTER     0x03
#define LTC3219_LED4_REGISTER     0x04
#define LTC3219_LED5_REGISTER     0x05
#define LTC3219_LED6_REGISTER     0x06
#define LTC3219_LED7_REGISTER     0x07
#define LTC3219_LED8_REGISTER     0x08
#define LTC3219_LED9_REGISTER     0x09

class LTC3219
{

public:
    LTC3219();
    bool begin(TwoWire &wirePort = Wire);
    bool writeByteToRegister(uint8_t address, uint8_t value);
    void turnOnLED(uint8_t address);
    void turnOffLED(uint8_t address);
    void setLEDBlink(uint8_t address);
private:
    TwoWire *_i2cPort;
};

#endif