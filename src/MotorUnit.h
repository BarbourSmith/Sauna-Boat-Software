// MotorUnit.h - Adapted from Maslow CNC (https://github.com/MaslowCNC/Maslow_4)
// Adapted for single-motor angle control (0-360 degrees) for boat auto-steering.
//
// Original copyright (c) 2024 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license.

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "AS5600.h"
#include "DCMotor.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

// PID gains – tune these for your motor/load combination.
// Kp: proportional gain; higher = faster response but may overshoot.
// Ki: integral gain; eliminates steady-state error.
// Kd: derivative gain; helps dampen oscillation.
#define STEERING_KP 3.5f
#define STEERING_KI 0.005f
#define STEERING_KD 0.3f

// Motor stops when angle error is within this deadband (degrees).
#define ANGLE_DEADBAND_DEG 3.0f

class MotorUnit {
public:
    // Initialise the motor unit using pins from the Maslow 4 bottom-right port.
    // @param forwardPin   GPIO driving motor forward (brIn1Pin  = 9)
    // @param backwardPin  GPIO driving motor backward (brIn2Pin = 3)
    // @param adcPin       GPIO for current sensing (brADCPin = 7)
    // @param encoderChannel  I2C-mux channel for the AS5600 encoder (BREncoderLine = 0)
    // @param pwmChannel1  LEDC channel for forward pin  (brIn1Channel = 6)
    // @param pwmChannel2  LEDC channel for backward pin (brIn2Channel = 7)
    // @param mux          Reference to the initialised I2C multiplexer
    void  begin(int forwardPin, int backwardPin, int adcPin,
                int encoderChannel, int pwmChannel1, int pwmChannel2,
                QWIICMUX& mux);

    // Read the encoder and update the cached current angle. Returns true on success.
    bool  updateEncoderPosition();

    // Set the desired steering angle (0–360 degrees).
    void  setTargetAngle(float degrees);

    // Returns the current target angle (0–360 degrees).
    float getTargetAngle() const;

    // Returns the last measured angle (0–360 degrees).
    float getCurrentAngle() const;

    // Run one PID iteration and drive the motor. Returns the PWM output value.
    double recomputePID();

    // Stop the motor and reset the PID integrator.
    void  stop();

    // Returns true if the AS5600 magnet is detected.
    bool  hasMagnet();

    // Returns true if the AS5600 encoder is responding on I2C.
    bool  isEncoderConnected();

    // Returns the instantaneous motor current in ADC counts.
    double getMotorCurrent();

private:
    // Compute the shortest-path error between target and current angle.
    // Result is in the range [-180, 180] degrees.
    float _computeWrappedError() const;

    QWIICMUX* _mux            = nullptr;
    int       _encoderChannel = 0;
    AS5600    _encoder;
    DCMotor   _motor;

    float _targetAngle  = 0.0f;  // degrees, 0–360
    float _currentAngle = 0.0f;  // degrees, 0–360

    // PID state
    float         _integral    = 0.0f;
    float         _lastError   = 0.0f;
    unsigned long _lastPIDTime = 0;
};
