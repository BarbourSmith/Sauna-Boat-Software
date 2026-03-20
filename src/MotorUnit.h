// MotorUnit.h – Steering unit wrapper for an RC servo.
// Provides the same speed-control interface as before but drives
// a standard RC servo instead of a brushed DC motor.

#pragma once

#include <Arduino.h>
#include "RCServo.h"

class MotorUnit {
public:
    // Initialise the steering servo.
    // @param servoPin  GPIO connected to the servo signal wire (e.g. GPIO 48)
    void  begin(int servoPin);

    // Center the servo and clear the ramp state.
    void  stop();

    // Set the desired normalized speed/position in [-1.0, 1.0].
    // -1.0 = full left, 0.0 = center, +1.0 = full right.
    // The value is stored and applied gradually by updateRamp().
    void  setSpeed(float normalizedSpeed);

    // Step the servo position toward the commanded speed at the configured
    // ramp rate. Call this every loop iteration (~100 Hz).
    void  updateRamp();

    // Set the ramp rate in normalized-units per second.
    // 0 = instant (no ramping). Example: 2.0 = 0→100% in 500 ms.
    void  setRampRate(float unitsPerSec);
    float getRampRate() const;

    // Scale the servo travel range. maxPWM in [0, 1023]:
    //   1023 = full servo range (±500 µs from center)
    //    512 = half range
    //      0 = locked to center
    void  setMaxSpeed(int maxPWM);
    int   getMaxSpeed() const;

    // Reverse the servo direction (negate the commanded position).
    void  setReversed(bool reversed);
    bool  getReversed() const;

    // Trim offset in µs added to the servo position after direction inversion.
    // Range: -500 to +500 (±500 µs = ±full travel).  Typical use: ±50 µs.
    void  setTrimUs(int trimUs);
    int   getTrimUs() const;

    // Always returns 0.0 – RC servos have no current sense output.
    double getMotorCurrent();

private:
    RCServo _servo;

    float         _targetSpeed  = 0.0f;
    float         _rampedSpeed  = 0.0f;
    float         _rampRate     = 2.0f;   // units/s; 0 = instant
    unsigned long _lastRampTime = 0;

    int           _maxSpeed     = 1023;   // scales servo travel (0–1023); 1023 = full range
    bool          _reversed     = false;  // negate position when true
    float         _trim         = 0.0f;   // normalized offset added after inversion
};
