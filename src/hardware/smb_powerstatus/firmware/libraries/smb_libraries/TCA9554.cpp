#include "TCA9554.hpp"

TCA9554::TCA9554()
{
}

bool TCA9554::begin(TwoWire &wirePort) {
	_i2cPort = &wirePort;

    writeByteToRegister(TCA9554_OUTPUT_REGISTER, 0x00); //digitalWrite(LED_BUILTIN, HIGH); // select output port register, set all outputs to 0 
    writeByteToRegister(TCA9554_CONFIG_REGISTER, 0xf8); // select config register, set P0, P1 and P2 as outputs (0 = output, 1 = input)

	return true;
}

void TCA9554::readInputs(uint8_t * value){

    *value = readByteFromRegister(TCA9554_INPUT_REGISTER);

}

void TCA9554::getBatteryValid(bool * valid){
    uint8_t sensor_value = 0;
    readInputs(&sensor_value);
    valid[0] = (sensor_value & TCA9554_ACDC_MASK) == 0;
    valid[1] = (sensor_value & TCA9554_BAT1_MASK) == 0;
    valid[2] = (sensor_value & TCA9554_BAT2_MASK) == 0;
}


bool TCA9554::writeByteToRegister(uint8_t address, uint8_t value) {
	_i2cPort->beginTransmission(TCA9554_ADDR);
	_i2cPort->write(address);
	_i2cPort->write(value);
	return (_i2cPort->endTransmission() == 0);
}

uint8_t TCA9554::readByteFromRegister(uint8_t address) {
	uint8_t value = 0;

	_i2cPort->beginTransmission(TCA9554_ADDR);
	_i2cPort->write(address);
	_i2cPort->endTransmission(false);

	_i2cPort->requestFrom(TCA9554_ADDR, 1);
	value = _i2cPort->read();
	_i2cPort->endTransmission();

	return value;
}

