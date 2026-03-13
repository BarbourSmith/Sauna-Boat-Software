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

    // Drive the motor at a normalized speed in the range [-1.0, 1.0].
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

    // Returns true if the AS5600 magnet is detected.
    bool  hasMagnet();

    // Returns true if the AS5600 encoder is responding on I2C.
    bool  isEncoderConnected();

    // Returns the instantaneous motor current in ADC counts.
    double getMotorCurrent();

    // Runtime-adjustable PID gains and deadband.
    void  setKp(float kp);
    void  setKi(float ki);
    void  setKd(float kd);
    void  setDeadband(float deadbandDeg);

    float getKp()       const;
    float getKi()       const;
    float getKd()       const;
    float getDeadband() const;

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

    // Runtime-configurable PID gains and deadband (default values match original defines)
    float _kp       = 3.5f;
    float _ki       = 0.005f;
    float _kd       = 0.3f;
    float _deadband = 3.0f;

    // Maximum PWM magnitude used by setSpeed() (0–1023)
    int   _maxSpeed = 512;

    // Ramp state
    float         _targetSpeed  = 0.0f;   // normalized speed commanded by setSpeed()
    float         _rampedSpeed  = 0.0f;   // normalized speed currently applied to motor
    float         _rampRate     = 2.0f;   // units/s; 0 = instant
    unsigned long _lastRampTime = 0;
};
