#ifndef TCA9554_HPP__
#define TCA9554_HPP__

#include "Arduino.h"
#include "Wire.h"

#define TCA9554_ADDR        0x21

//TCA9554 register defines
#define TCA9554_INPUT_REGISTER      0x00
#define TCA9554_OUTPUT_REGISTER     0x01
#define TCA9554_POLARITY_REGISTER   0x02
#define TCA9554_CONFIG_REGISTER     0x03
#define TCA9554_ACDC_MASK           0x10
#define TCA9554_BAT1_MASK           0x20
#define TCA9554_BAT2_MASK           0x40


class TCA9554
{

public:
    TCA9554();
    bool begin(TwoWire &wirePort = Wire);
    bool writeByteToRegister(uint8_t address, uint8_t value);
    void readInputs(uint8_t * value);
    uint8_t readByteFromRegister(uint8_t address);
    void getBatteryValid(bool * valid);

private:
    TwoWire *_i2cPort;
};

#endif