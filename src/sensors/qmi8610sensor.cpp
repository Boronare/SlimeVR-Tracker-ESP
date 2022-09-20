/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 S.J. Remington, SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "qmi8610sensor.h"

// Typical sensitivity at 25C
// See p. 9 of https://www.mouser.com/datasheet/2/783/BST-BMI160-DS000-1509569.pdf
// 65.6 LSB/deg/s = 500 deg/s
#define TYPICAL_SENSITIVITY_LSB 65.6

// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float GSCALE = ((32768. / TYPICAL_SENSITIVITY_LSB) / 32768.) * (PI / 180.0);

void QMI8610Sensor::motionSetup() {
    // initialize device
    imu.initialize(addr);
    if(!imu.testConnection()) {
        Serial.print("[ERR] Can't communicate with QMI8610, response 0x");
        Serial.println(imu.getDeviceID(), HEX);
        LEDManager::signalAssert();
        return;
    }

    Serial.print("[OK] Connected to QMI8610, ID 0x");
    Serial.println(imu.getDeviceID(), HEX);

    int16_t ax, ay, az;

    working = true;
}

void QMI8610Sensor::motionLoop() {
    int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
    imu.getMotion9(&ax,&ay,&az,&gx,&gy,&gz,&mx,&my,&mz);

    Serial.printf("%+5d %+5d %+5d / %+5d %+5d %+5d / %+5d %+5d %+5d\n",ax,ay,az,gx,gy,gz,mx,my,mz);
}

float QMI8610Sensor::getTemperature()
{
    return imu.getTemperature()*1.0;
}