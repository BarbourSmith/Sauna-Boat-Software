/*
  This is an Arduino library written for the TCA9548A/PCA9548A 8-bit multiplexer.
  By Nathan Seidle @ SparkFun Electronics, May 16th, 2020
  Source: https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library
  Used by Maslow CNC (https://github.com/MaslowCNC/Maslow_4)
*/

#include "SparkFun_I2C_Mux_Arduino_Library.h"

bool QWIICMUX::begin(uint8_t deviceAddress, TwoWire& wirePort) {
    _i2cPort       = &wirePort;
    _deviceAddress = deviceAddress;

    if (isConnected() == false)
        return (false);
    return (true);
}

bool QWIICMUX::isConnected() {
    _i2cPort->beginTransmission(_deviceAddress);
    if (_i2cPort->endTransmission() != 0)
        return (false);

    setPortState(0xA4);
    uint8_t response = getPortState();
    setPortState(0x00);
    if (response == 0xA4)
        return (true);
    return (false);
}

bool QWIICMUX::setPort(uint8_t portNumber) {
    uint8_t portValue = 0;

    if (portNumber > 7)
        portValue = 0;
    else
        portValue = 1 << portNumber;

    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(portValue);
    if (_i2cPort->endTransmission() != 0)
        return (false);
    return (true);
}

uint8_t QWIICMUX::getPort() {
    _i2cPort->requestFrom(_deviceAddress, uint8_t(1));
    if (!_i2cPort->available())
        return (254);
    uint8_t portBits = _i2cPort->read();

    for (uint8_t x = 0; x < 8; x++) {
        if (portBits & (1 << x))
            return (x);
    }
    return (255);
}

bool QWIICMUX::setPortState(uint8_t portBits) {
    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(portBits);
    if (_i2cPort->endTransmission() != 0)
        return (false);
    return (true);
}

uint8_t QWIICMUX::getPortState() {
    _i2cPort->requestFrom(_deviceAddress, uint8_t(1));
    return (_i2cPort->read());
}

bool QWIICMUX::enablePort(uint8_t portNumber) {
    if (portNumber > 7)
        portNumber = 7;

    uint8_t settings = getPortState();
    settings |= (1 << portNumber);
    return (setPortState(settings));
}

bool QWIICMUX::disablePort(uint8_t portNumber) {
    if (portNumber > 7)
        portNumber = 7;

    uint8_t settings = getPortState();
    settings &= ~(1 << portNumber);
    return (setPortState(settings));
}
