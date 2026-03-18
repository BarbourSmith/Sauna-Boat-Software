// RCServo.h – RC servo driver using ESP32 LEDC peripheral (Arduino-ESP32 v2.x API).
// Generates a 50 Hz PWM signal with pulse widths between 1000–2000 µs,
// which is the standard protocol for RC servos.

#pragma once

#include <Arduino.h>

class RCServo {
public:
    // Attach the servo to a GPIO pin.
    void begin(int pin);

    // Write a normalized position in [-1.0, 1.0].
    //   -1.0 → minimum pulse (1000 µs)
    //    0.0 → center pulse  (1500 µs)
    //   +1.0 → maximum pulse (2000 µs)
    // Values are clamped to [-1.0, 1.0].
    void write(float position);

    // Move the servo to the center position (1500 µs).
    void center();

private:
    int _pin = -1;

    // LEDC channel reserved for the steering servo.
    // Kept internal – callers only specify the GPIO pin.
    static constexpr int  LEDC_CHANNEL    = 0;

    static constexpr int  FREQ_HZ         = 50;
    static constexpr int  RESOLUTION_BITS = 14;                        // ESP32-S3 LEDC max is 14-bit
    static constexpr long PERIOD_COUNTS   = (1L << RESOLUTION_BITS);  // 16384
    static constexpr int  MIN_PULSE_US    = 1000;
    static constexpr int  MAX_PULSE_US    = 2000;
    static constexpr int  PERIOD_US       = 20000;  // 1 / 50 Hz
};
