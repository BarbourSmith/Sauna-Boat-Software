// Copyright (c) 2024 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

/***************************************************
 *   This is a library to interact with A DC Motor
 *
 *  By Bar Smith for Maslow CNC
 ****************************************************/

#include "DCMotor.h"

#define motorPWMFreq 16000
#define motorPWMRes 10

DCMotor::DCMotor() {}

void DCMotor::begin(uint8_t forwardPin, uint8_t backwardPin, int readbackPin, int channel1, int channel2) {
    _forward  = forwardPin;
    _back     = backwardPin;
    _readback = readbackPin;
    _channel1 = channel1;
    _channel2 = channel2;

    //Setup the motor controllers
    ledcSetup(channel1, motorPWMFreq, motorPWMRes);  // configure PWM functionalities...this uses timer 0 (channel, freq, resolution)
    ledcAttachPin(_forward, channel1);               // attach the channel to the GPIO to be controlled
    ledcWrite(channel1, 0);                          //Turn the motor off

    ledcSetup(channel2, motorPWMFreq, motorPWMRes);
    ledcAttachPin(_back, channel2);
    ledcWrite(channel2, 0);
}

void DCMotor::forward(uint16_t speed) {
    runAtSpeed(FORWARD, speed);
}

void DCMotor::fullOut() {
    runAtSpeed(FORWARD, _maxSpeed);
}

void DCMotor::backward(uint16_t speed) {
    runAtSpeed(BACKWARD, speed);
}

void DCMotor::fullIn() {
    runAtSpeed(BACKWARD, _maxSpeed);
}

void DCMotor::halfIn() {
    runAtPWM(_maxSpeed / -2);
}

void DCMotor::runAtPWM(long signed_speed) {
    //Motor driver accepts -maxPWMvalue to maxPWMvalue but doesn't begin moving until motorStartsToMovePWM so we scale
    int  motorStartsToMovePWM = 75;
    int  maxPWMvalue          = 1023;
    long scaledSpeed          = map(abs(signed_speed), 0, maxPWMvalue, motorStartsToMovePWM, _maxSpeed);

    if (signed_speed < 0) {
        runAtSpeed(BACKWARD, scaledSpeed);
    } else {
        runAtSpeed(FORWARD, scaledSpeed);
    }
}

void DCMotor::runAtSpeed(uint8_t direction, uint16_t speed) {
    if (direction == 0) {
        ledcWrite(_channel1, _maxSpeed);
        ledcWrite(_channel2, _maxSpeed - speed);

    } else {
        ledcWrite(_channel2, _maxSpeed);
        ledcWrite(_channel1, _maxSpeed - speed);
    }
}

void DCMotor::stop() {
    //These could be set to 1023 to allow coasting
    ledcWrite(_channel1, 0);  //Stop
    ledcWrite(_channel2, 0);
}

void DCMotor::highZ() {
    ledcWrite(_channel1, 0);  //Stop
    ledcWrite(_channel2, 0);
}

double DCMotor::readCurrent() {
    return analogRead(_readback);
}
