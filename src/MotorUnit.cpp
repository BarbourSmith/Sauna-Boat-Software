// MotorUnit.cpp – Steering unit wrapper for an RC servo.

#include "MotorUnit.h"

void MotorUnit::begin(int servoPin) {
    _servo.begin(servoPin);
    _lastRampTime = millis();
    Serial.println("[MotorUnit] Servo steering initialised.");
}

void MotorUnit::stop() {
    _targetSpeed = 0.0f;
    _rampedSpeed = 0.0f;
    _servo.center();
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

    // Scale by maxSpeed (0–1023 → 0.0–1.0 fraction of full servo travel)
    float position = _rampedSpeed * (_maxSpeed / 1023.0f);
    _servo.write(position);
}

void  MotorUnit::setRampRate(float unitsPerSec) { _rampRate = (unitsPerSec >= 0.0f) ? unitsPerSec : 0.0f; }
float MotorUnit::getRampRate() const             { return _rampRate; }

void MotorUnit::setMaxSpeed(int maxPWM) {
    _maxSpeed = constrain(maxPWM, 0, 1023);
}

int MotorUnit::getMaxSpeed() const {
    return _maxSpeed;
}

double MotorUnit::getMotorCurrent() {
    return 0.0;  // RC servo has no current sense
}
