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
#ifndef SENSORS_QMI8658SENSOR_H
#define SENSORS_QMI8658SENSOR_H

#include "sensor.h"
#include "sensors/axisremap.h"
#include "magneto1.4.h"

#include <QMI8658.h>
#include "SensorFusionRestDetect.h"
#include "../motionprocessing/types.h"

#include "../motionprocessing/GyroTemperatureCalibrator.h"
#include "../motionprocessing/RestDetection.h"
// #include <ekf.h>

#define CaliSamples 180
#define GyroTolerance 30
#define MagTolerance 250
#define AccTolerance 30

#define QMI8658_TIMESTAMP_RESOLUTION_MICROS 1.0f
#define QMI8658_MAP_ODR_MICROS(micros) ((uint16_t)((micros) / QMI8658_TIMESTAMP_RESOLUTION_MICROS) * QMI8658_TIMESTAMP_RESOLUTION_MICROS)
// constexpr float QMI8658_ODR_HZ = 58.75f;//QMI8658C
constexpr float QMI8658_ODR_HZ = 56.05f;//QMI8658A
// #define QMI8658_USE_TEMPCAL true

constexpr float QMI8658_ODR_MICROS = 1.0f / QMI8658_ODR_HZ * 1e6f;

struct QMI8658VQFParams: VQFParams {
    QMI8658VQFParams() : VQFParams() {
        magDistRejectionEnabled = true;
        tauAcc = 2.0f;
        restMinT = 2.0f;
        restThGyr = 0.6f; // 400 norm
        restThAcc = 0.06f; // 100 norm
        tauMag = 2.0f;
    }
};

class QMI8658Sensor : public Sensor
{
    public:
        QMI8658Sensor(uint8_t id, uint8_t address, float rotation, uint8_t sclPin, uint8_t sdaPin, int axisRemap=AXIS_REMAP_DEFAULT) :
            Sensor("BMI160Sensor", IMU_BMI160, id, address, rotation, sclPin, sdaPin),
            axisRemap(axisRemap),
            sfusion(QMI8658_ODR_MICROS / 1e6f, QMI8658_ODR_MICROS / 1e6f, QMI8658_ODR_MICROS / 1e6f)
        {
        };
        ~QMI8658Sensor(){};
        void motionSetup() override final;
        void motionLoop() override final;
        float getTemperature();
        void getValueScaled();
        void AutoCalibrateGyro(int16_t gx, int16_t gy, int16_t gz,int16_t ax, int16_t ay, int16_t az);
        void CalibrateGyro(int16_t gx, int16_t gy, int16_t gz, int16_t acx, int16_t acy, int16_t acz, uint8_t save = 0);
        void CalibrateMag(int16_t mx, int16_t my, int16_t mz);
        void CalibrateAcc(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
        void startCalibration(int calibrationType);

    private:
        QMI8658 imu{};
        int axisRemap;

        SlimeVR::Sensors::SensorFusionRestDetect sfusion;
        float Axyz[3] = {0};
        float Gxyz[3] = {0};
        float Mxyz[3] = {0};
        float lastAxyz[3] = {0};

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

        SlimeVR::Configuration::QMI8658CalibrationConfig m_Calibration;
};
#endif