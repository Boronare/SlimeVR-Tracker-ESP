/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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
#include "sensor.h"
#include "logging/Logger.h"

#include <QMI8658.h>

#define CaliSamples 240
#define GyroTolerance 500
#define MagTolerance 250

class QMI8658Sensor : public Sensor
{
public:
    QMI8658Sensor(uint8_t id, uint8_t address, float rotation) : Sensor("QMI8658Sensor", IMU_QMI8658, id, address, rotation){};
    ~QMI8658Sensor(){};
    void motionSetup() override final;
    void motionLoop() override final;
    float getTemperature();
    void getValueScaled();
    void AutoCalibrate(int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz);
    void AutoCalibrateGyro(int16_t gx, int16_t gy, int16_t gz);
    void AutoCalibrateMag(int16_t mx, int16_t my, int16_t mz);

private:
    QMI8658 imu{};
    SlimeVR::Configuration::QMI8658CalibrationConfig m_Calibration;
    float Axyz[3]{};
    float Gxyz[3]{};
    float Mxyz[3]{};
    float A_B[3]{0.0f,0.0f,0.0f};
    int16_t prevM[3]{};
    int16_t Cx[CaliSamples]{};
    int16_t Cy[CaliSamples]{};
    int16_t Cz[CaliSamples]{};
    int8_t ignoreList[CaliSamples]{};
    uint8_t Gr = CaliSamples , Mr = CaliSamples - 1, Gf = 0, Mf = 0;
    float MagStr = 0;
    float q[4]{1.0f, 0.0f, 0.0f, 0.0f};
    // Loop timing globals
    uint32_t now = 0, last = 0; // micros() timers
    float deltat = 0;           // loop time in seconds

    SlimeVR::Configuration::QMI8658CalibrationConfig getMagCalibration();
    bool verifyMagCali(SlimeVR::Configuration::QMI8658CalibrationConfig cali);
};