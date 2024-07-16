#include "LTC3219.h"

LTC3219::LTC3219()
{
}

void LTC3219::turnOnLED(uint8_t address){
    writeByteToRegister(address, 0x0F);
}

void LTC3219::turnOffLED(uint8_t address){
    writeByteToRegister(address, 0x00);
}

void LTC3219::setLEDBlink(uint8_t address){
	writeByteToRegister(address, 0x7F);
}

bool LTC3219::begin(TwoWire &wirePort) {
	// Wire.begin() should be called in the application code in advance
	_i2cPort = &wirePort;

    writeByteToRegister(LTC3219_COMMAND_REGISTER, 0x02); // select command register, write to all register simulatously for init, auto chargepump mode
    writeByteToRegister(LTC3219_LED1_REGISTER, 0x00); // select LED1 register, will be written to all simulatously, normal LED mode, all LEDs off
    writeByteToRegister(LTC3219_COMMAND_REGISTER, 0x00);  // select command register, normal individual register access mode, auto chargepump mode

	return true;
}


bool LTC3219::writeByteToRegister(uint8_t address, uint8_t value) {
	_i2cPort->beginTransmission(LED_DRV_ADDR);
	_i2cPort->write(address);
	_i2cPort->write(value);
	return (_i2cPort->endTransmission() == 0);
}
