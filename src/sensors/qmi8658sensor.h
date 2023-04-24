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
#include<vqf.h>
#include "../motionprocessing/types.h"

#include "../motionprocessing/GyroTemperatureCalibrator.h"
#include "../motionprocessing/RestDetection.h"
// #include <ekf.h>

#define CaliSamples 240
#define GyroTolerance 250
#define MagTolerance 250
#define AccTolerance 30

#define QMI8658_TIMESTAMP_RESOLUTION_MICROS 14285.f
#define QMI8658_MAP_ODR_MICROS(micros) ((uint16_t)((micros) / QMI8658_TIMESTAMP_RESOLUTION_MICROS) * QMI8658_TIMESTAMP_RESOLUTION_MICROS)
constexpr float QMI8658_ODR_HZ = 70.0f;
constexpr float QMI8658_ODR_MICROS = QMI8658_MAP_ODR_MICROS(1.0f / QMI8658_ODR_HZ * 1e6f);

struct QMI8658VQFParams: VQFParams {
    QMI8658VQFParams() : VQFParams() {
        tauAcc = 2.0f;
        restMinT = 2.0f;
        restThGyr = 0.6f; // 400 norm
        restThAcc = 0.06f; // 100 norm
    }
};

class QMI8658Sensor : public Sensor
{
public:
    QMI8658Sensor(uint8_t id, uint8_t address, float rotation) : 
    Sensor("QMI8658Sensor", IMU_QMI8658, id, address, rotation)
            , vqf(vqfParams, QMI8658_ODR_MICROS / 1e6f, QMI8658_ODR_MICROS / 1e6f, 20000 / 1e6f){};
    ~QMI8658Sensor(){};
    void motionSetup() override final;
    void motionLoop() override final;
    float getTemperature();
    void getValueScaled();
    void AutoCalibrateGyro(int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz);
    void CalibrateGyro(int16_t gx, int16_t gy, int16_t gz);
    void CalibrateMag(int16_t mx, int16_t my, int16_t mz);
    void CalibrateAcc(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
    void startCalibration(int calibrationType);

private:
    QMI8658 imu{};
    SlimeVR::Configuration::QMI8658CalibrationConfig m_Calibration;
    float Axyz[3] = {0};
    float Gxyz[3] = {0};
    float Mxyz[3] = {0};
    float lastAxyz[3] = {0};
    bool fusionUpdated = false;
    
    // clock sync and sample timestamping
    uint32_t sensorTime0 = 0;
    uint32_t sensorTime1 = 0;
    uint32_t localTime0 = 0;
    uint32_t localTime1 = 0;
    int32_t dtMicros;
    double sensorTimeRatio = 1;
    double sensorTimeRatioEma = 1;
    double sampleDtMicros = 20000;
    uint32_t syncLatencyMicros = 0;
    uint32_t samplesSinceClockSync = 0;
    uint32_t timestamp0 = 0;
    uint32_t timestamp1 = 0;
    
    QMI8658VQFParams vqfParams {};
    VQF vqf;
    
    // scheduling
    uint32_t lastPollTime = micros();
    uint32_t lastClockPollTime = micros();
    uint32_t lastRotationPacketSent = 0;
    uint32_t lastTemperaturePacketSent = 0;

    GyroTemperatureCalibrator* gyroTempCalibrator = nullptr;
    float temperature = 0;
    int16_t prevM[3]{};
    int16_t Cx[CaliSamples]{};
    int16_t Cy[CaliSamples]{};
    int16_t Cz[CaliSamples]{};
    int8_t ignoreList[CaliSamples]{};
    float q[4]{1.0f, 0.0f, 0.0f, 0.0f};
    // EKF Kalman{q};
    uint8_t Cr = CaliSamples, Cf = 0;
    uint8_t accelDupCnt = 0;
    float MagStr = 0;
    // Loop timing globals
    float GOxyzStaticTempCompensated[3];
    SlimeVR::Configuration::QMI8658CalibrationConfig getMagAccCalibration();
    bool verifyMagAccCali(SlimeVR::Configuration::QMI8658CalibrationConfig cali);
};