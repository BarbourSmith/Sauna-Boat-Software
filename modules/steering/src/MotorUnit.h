// MotorUnit.h - Adapted from Maslow CNC (https://github.com/MaslowCNC/Maslow_4)
// Adapted for speed-based motor control for sauna boat steering.
//
// Original copyright (c) 2024 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license.

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "DCMotor.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

class MotorUnit {
public:
    // Initialise the motor unit using pins from the Maslow 4 bottom-right port.
    // @param forwardPin   GPIO driving motor forward  (BR_FORWARD_PIN  = 3)
    // @param backwardPin  GPIO driving motor backward (BR_BACKWARD_PIN = 9)
    // @param adcPin       GPIO for current sensing    (BR_ADC_PIN      = 7)
    // @param encoderChannel  I2C-mux channel (unused by speed-based control, kept for hardware compat)
    // @param pwmChannel1  LEDC channel for forward pin  (BR_PWM_CHANNEL1 = 6)
    // @param pwmChannel2  LEDC channel for backward pin (BR_PWM_CHANNEL2 = 7)
    // @param mux          Reference to the initialised I2C multiplexer
    void  begin(int forwardPin, int backwardPin, int adcPin,
                int encoderChannel, int pwmChannel1, int pwmChannel2,
                QWIICMUX& mux);

    // Drive the motor at a normalized speed in the range [-1.0, 1.0].
    // Negative = port/left, positive = starboard/right.
    // The speed is stored as a target; call updateRamp() each loop to ramp
    // the actual output toward it.
    void  setSpeed(float normalizedSpeed);

    // Step the output speed toward the target at the configured ramp rate.
    // Call this every loop iteration (~100 Hz).
    void  updateRamp();

    // Set the ramp rate in normalized-units per second.
    // 0 = instant (no ramping).  Example: 2.0 = 0→100% in 500 ms.
    void  setRampRate(float unitsPerSec);
    float getRampRate() const;

    // Set/get the maximum PWM magnitude used by setSpeed() (0–1023).
    void  setMaxSpeed(int maxPWM);
    int   getMaxSpeed() const;

    // Stop the motor immediately, snapping both target and ramped speed to 0.
    void  stop();

    // Returns the instantaneous motor current in ADC counts.
    double getMotorCurrent();

private:
    QWIICMUX* _mux            = nullptr;
    int       _encoderChannel = 0;
    DCMotor   _motor;

    // Maximum PWM magnitude used by setSpeed() (0–1023)
    int   _maxSpeed     = 512;

    // Ramp state
    float         _targetSpeed  = 0.0f;   // normalized speed commanded by setSpeed()
    float         _rampedSpeed  = 0.0f;   // normalized speed currently applied to motor
    float         _rampRate     = 2.0f;   // units/s; 0 = instant
    unsigned long _lastRampTime = 0;
};
