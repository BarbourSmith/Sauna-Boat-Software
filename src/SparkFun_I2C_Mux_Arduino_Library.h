/*
  This is an Arduino library written for the TCA9548A/PCA9548A 8-bit multiplexer.
  By Nathan Seidle @ SparkFun Electronics, May 16th, 2020
  Source: https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library
  Used by Maslow CNC (https://github.com/MaslowCNC/Maslow_4)

  The TCA9548A/PCA9548A allows for up to 8 devices to be attached to a single
  I2C bus. This is helpful for I2C devices that have a single I2C address.
*/

#ifndef SparkFun_I2C_Mux_Arduino_Library_h
#define SparkFun_I2C_Mux_Arduino_Library_h

#include "Arduino.h"
#include <Wire.h>

#define QWIIC_MUX_DEFAULT_ADDRESS 0x70

class QWIICMUX {
public:
    bool    begin(uint8_t deviceAddress = QWIIC_MUX_DEFAULT_ADDRESS, TwoWire& wirePort = Wire);
    bool    isConnected();
    bool    setPort(uint8_t portNumber);
    bool    setPortState(uint8_t portBits);
    uint8_t getPort();
    uint8_t getPortState();
    bool    enablePort(uint8_t portNumber);
    bool    disablePort(uint8_t portNumber);

private:
    TwoWire* _i2cPort;
    uint8_t  _deviceAddress = QWIIC_MUX_DEFAULT_ADDRESS;
};
#endif
