// RCServo.cpp – RC servo driver using ESP32 LEDC peripheral (Arduino-ESP32 v2.x API).

#include "RCServo.h"
#include "driver/gpio.h"    // gpio_reset_pin()

void RCServo::begin(int pin) {
    _pin = pin;

    // Force-release the pin from any peripheral that claimed it during board
    // init (e.g. the RMT-driven RGB LED on esp32-s3-devkitc-1 which maps to
    // GPIO 48 via RGB_BUILTIN). Without this, ledcAttachPin silently loses the
    // GPIO matrix slot and no signal ever reaches the physical pin.
    gpio_reset_pin((gpio_num_t)_pin);

    // v2.x LEDC API: configure the timer, then route it to the GPIO
    double actualFreq = ledcSetup(LEDC_CHANNEL, FREQ_HZ, RESOLUTION_BITS);
    if (actualFreq == 0.0) {
        Serial.printf("[RCServo] ERROR: ledcSetup failed for ch %d\n", LEDC_CHANNEL);
        return;
    }
    Serial.printf("[RCServo] ledcSetup: ch=%d actual=%.2f Hz (target=%d Hz)\n",
                  LEDC_CHANNEL, actualFreq, FREQ_HZ);

    ledcAttachPin(_pin, LEDC_CHANNEL);
    Serial.printf("[RCServo] Attached to GPIO %d\n", _pin);

    center();
    Serial.printf("[RCServo] Ready – GPIO %d, LEDC ch %d (%d Hz, %d-bit)\n",
                  _pin, LEDC_CHANNEL, FREQ_HZ, RESOLUTION_BITS);
}

void RCServo::write(float position) {
    position = constrain(position, -1.0f, 1.0f);

    // Map [-1, 1] → [MIN_PULSE_US, MAX_PULSE_US]
    float pulseUs = MIN_PULSE_US
                  + (position + 1.0f) * 0.5f * (MAX_PULSE_US - MIN_PULSE_US);

    // Convert µs to LEDC duty counts (16-bit, 50 Hz)
    uint32_t duty = (uint32_t)((pulseUs / PERIOD_US) * PERIOD_COUNTS);

    ledcWrite(LEDC_CHANNEL, duty);

    static unsigned long lastLog = 0;
    if (millis() - lastLog >= 500) {
        lastLog = millis();
        Serial.printf("[RCServo] pos=%.2f pulseUs=%.0f duty=%lu\n",
                      position, pulseUs, (unsigned long)duty);
    }
}

void RCServo::center() {
    write(0.0f);
}
