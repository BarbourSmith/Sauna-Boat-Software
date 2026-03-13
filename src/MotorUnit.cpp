// MotorUnit.cpp - Adapted from Maslow CNC (https://github.com/MaslowCNC/Maslow_4)
// Adapted for single-motor angle control (0-360 degrees) for boat auto-steering.
//
// Original copyright (c) 2024 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license.

#include "MotorUnit.h"

void MotorUnit::begin(int forwardPin, int backwardPin, int adcPin,
                      int encoderChannel, int pwmChannel1, int pwmChannel2,
                      QWIICMUX& mux) {
    _mux            = &mux;
    _encoderChannel = encoderChannel;

    // Initialise the AS5600 encoder through the I2C mux
    _mux->setPort(_encoderChannel);
    if (!_encoder.begin()) {
        Serial.printf("[MotorUnit] WARNING: Encoder not found on mux channel %d\n", _encoderChannel);
    } else {
        Serial.printf("[MotorUnit] Encoder ready on mux channel %d\n", _encoderChannel);
        if (_encoder.detectMagnet()) {
            Serial.println("[MotorUnit] Magnet detected.");
        } else {
            Serial.println("[MotorUnit] WARNING: Magnet not detected – check encoder placement.");
        }
    }

    // Initialise the DC motor driver
    _motor.begin(forwardPin, backwardPin, adcPin, pwmChannel1, pwmChannel2);
    _motor.stop();

    _lastPIDTime = millis();
    Serial.println("[MotorUnit] Initialised.");
}

bool MotorUnit::updateEncoderPosition() {
    if (!_mux->setPort(_encoderChannel)) {
        return false;
    }

    if (!_encoder.isConnected()) {
        return false;
    }

    // readAngle() returns 0–4095; convert to 0–360 degrees
    uint16_t raw = _encoder.readAngle();
    if (_encoder.lastError() != AS5600_OK) {
        return false;
    }
    _currentAngle = (raw / 4096.0f) * 360.0f;
    return true;
}

void MotorUnit::setTargetAngle(float degrees) {
    // Normalise to [0, 360)
    degrees = fmodf(degrees, 360.0f);
    if (degrees < 0.0f) degrees += 360.0f;
    _targetAngle = degrees;
}

float MotorUnit::getTargetAngle() const {
    return _targetAngle;
}

float MotorUnit::getCurrentAngle() const {
    return _currentAngle;
}

bool MotorUnit::hasMagnet() {
    _mux->setPort(_encoderChannel);
    return _encoder.detectMagnet();
}

bool MotorUnit::isEncoderConnected() {
    _mux->setPort(_encoderChannel);
    return _encoder.isConnected();
}

double MotorUnit::getMotorCurrent() {
    return _motor.readCurrent();
}

void MotorUnit::setKp(float kp)             { _kp       = kp; }
void MotorUnit::setKi(float ki)             { _ki       = ki; }
void MotorUnit::setKd(float kd)             { _kd       = kd; }
void MotorUnit::setDeadband(float deadband) { _deadband = deadband; }

float MotorUnit::getKp()       const { return _kp; }
float MotorUnit::getKi()       const { return _ki; }
float MotorUnit::getKd()       const { return _kd; }
float MotorUnit::getDeadband() const { return _deadband; }

void MotorUnit::stop() {
    _motor.stop();
    _integral  = 0.0f;
    _lastError = 0.0f;
}

void MotorUnit::setSpeed(float normalizedSpeed) {
    normalizedSpeed = constrain(normalizedSpeed, -1.0f, 1.0f);
    long pwm = static_cast<long>(normalizedSpeed * _maxSpeed);
    _motor.runAtPWM(pwm);
}

void MotorUnit::setMaxSpeed(int maxPWM) {
    _maxSpeed = constrain(maxPWM, 0, 1023);
}

int MotorUnit::getMaxSpeed() const {
    return _maxSpeed;
}

float MotorUnit::_computeWrappedError() const {
    float error = _targetAngle - _currentAngle;
    // Normalise to [-180, 180] for shortest-path steering
    while (error >  180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

double MotorUnit::recomputePID() {
    float error = _computeWrappedError();

    // Within the deadband: stop and reset integrator to prevent hunting
    if (fabsf(error) < _deadband) {
        _motor.stop();
        _integral  = 0.0f;
        _lastError = 0.0f;
        return 0.0;
    }

    unsigned long now = millis();
    float dt = (now - _lastPIDTime) / 1000.0f;
    _lastPIDTime = now;
    // Clamp dt to avoid large jumps on first call or after pauses
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

    // Integral with anti-windup clamping
    _integral += error * dt;
    _integral = constrain(_integral, -500.0f, 500.0f);

    // Derivative (on error, not measurement, to handle setpoint changes)
    float derivative = (error - _lastError) / dt;
    _lastError = error;

    // PID output in PWM range [-1023, 1023]
    float output = _kp * error
                 + _ki * _integral
                 + _kd * derivative;
    output = constrain(output, -1023.0f, 1023.0f);

    static unsigned long _lastPIDLog = 0;
    if (millis() - _lastPIDLog >= 500) {
        _lastPIDLog = millis();
        Serial.printf("[PID] error=%.1f output=%.0f\n", error, output);
    }
    _motor.runAtPWM(static_cast<long>(output));
    return static_cast<double>(output);
}
