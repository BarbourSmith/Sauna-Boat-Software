// MotorUnit.cpp - Adapted from Maslow CNC (https://github.com/MaslowCNC/Maslow_4)
// Adapted for speed-based motor control for sauna boat steering.
//
// Original copyright (c) 2024 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license.

#include "MotorUnit.h"

void MotorUnit::begin(int forwardPin, int backwardPin, int adcPin,
                      int encoderChannel, int pwmChannel1, int pwmChannel2,
                      QWIICMUX& mux) {
    _mux            = &mux;
    _encoderChannel = encoderChannel;

    // Initialise the DC motor driver
    _motor.begin(forwardPin, backwardPin, adcPin, pwmChannel1, pwmChannel2);
    _motor.stop();

    _lastRampTime = millis();
    Serial.println("[MotorUnit] Initialised (speed-based mode).");
}

void MotorUnit::setSpeed(float normalizedSpeed) {
    _targetSpeed = constrain(normalizedSpeed, -1.0f, 1.0f);
}

void MotorUnit::updateRamp() {
    unsigned long now = millis();
    float dt = (now - _lastRampTime) / 1000.0f;
    _lastRampTime = now;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

    if (_rampRate <= 0.0f) {
        _rampedSpeed = _targetSpeed;
    } else {
        float diff     = _targetSpeed - _rampedSpeed;
        float maxDelta = _rampRate * dt;
        if (fabsf(diff) <= maxDelta) {
            _rampedSpeed = _targetSpeed;
        } else {
            _rampedSpeed += copysignf(maxDelta, diff);
        }
    }

    if (_rampedSpeed == 0.0f) {
        _motor.stop();
    } else {
        _motor.runAtPWM(static_cast<long>(_rampedSpeed * _maxSpeed));
    }
}

void  MotorUnit::setRampRate(float unitsPerSec) { _rampRate = (unitsPerSec >= 0.0f) ? unitsPerSec : 0.0f; }
float MotorUnit::getRampRate() const             { return _rampRate; }

void MotorUnit::setMaxSpeed(int maxPWM) {
    _maxSpeed = constrain(maxPWM, 0, 1023);
}

int MotorUnit::getMaxSpeed() const {
    return _maxSpeed;
}

void MotorUnit::stop() {
    _motor.stop();
    _targetSpeed = 0.0f;
    _rampedSpeed = 0.0f;
}

double MotorUnit::getMotorCurrent() {
    return _motor.readCurrent();
}
