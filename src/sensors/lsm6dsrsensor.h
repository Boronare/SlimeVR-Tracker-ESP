/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 SlimeVR Contributors

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

#ifndef SENSORS_LSM6DSRSENSOR_H
#define SENSORS_LSM6DSRSENSOR_H

#include "sensor.h"
#include "sensors/axisremap.h"
#include "magneto1.4.h"

#include <LSM6DSR.h>
#include "SensorFusionRestDetect.h"
#include "../motionprocessing/types.h"

#include "../motionprocessing/GyroTemperatureCalibrator.h"
#include "../motionprocessing/RestDetection.h"

#define LSM6DSR_GYRO_RATE LSM6DSR_GY_ODR_417Hz
#define LSM6DSR_GYRO_RANGE LSM6DSR_1000dps

#define LSM6DSR_ACCEL_RATE LSM6DSR_XL_ODR_52Hz
#define LSM6DSR_ACCEL_RANGE LSM6DSR_4g

// note: if changing ODR or filter modes - adjust rest detection params and buffer size

constexpr float LSM6DSR_ODR_GYR_HZ = 417;
constexpr float LSM6DSR_ODR_ACC_HZ = 52;
constexpr float LSM6DSR_ODR_GYR_MICROS = 1.0f / LSM6DSR_ODR_GYR_HZ * 1e6f;
constexpr float LSM6DSR_ODR_ACC_MICROS = 1.0f / LSM6DSR_ODR_ACC_HZ * 1e6f;
// note: this value only sets polling and fusion update rate - HMC is internally sampled at 75hz, QMC at 200hz
#define LSM6DSR_MAG_RATE LSM6DSR_SH_ODR_26Hz
constexpr float LSM6DSR_ODR_MAG_HZ = 26;
constexpr float LSM6DSR_ODR_MAG_MICROS = 1.0f / LSM6DSR_ODR_MAG_HZ * 1e6f;


// Typical sensitivity
constexpr double LSM6DSR_GYRO_TYPICAL_SENSITIVITY_MDPS = 35.0f;

constexpr std::pair<uint8_t, float> LSM6DSR_ACCEL_SENSITIVITY_LSB_MAP[] = {
    {LSM6DSR_2g, 16384.0f},
    {LSM6DSR_4g, 8192.0f},
    {LSM6DSR_8g, 4096.0f},
    {LSM6DSR_16g, 2048.0f}
};
constexpr double LSM6DSR_ACCEL_TYPICAL_SENSITIVITY_LSB = LSM6DSR_ACCEL_SENSITIVITY_LSB_MAP[LSM6DSR_ACCEL_RANGE / 4].second;
constexpr double LSM6DSR_ASCALE = CONST_EARTH_GRAVITY / LSM6DSR_ACCEL_TYPICAL_SENSITIVITY_LSB;

// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr double LSM6DSR_GSCALE = ((LSM6DSR_GYRO_TYPICAL_SENSITIVITY_MDPS*1e-3)) * (PI / 180.0);

constexpr uint32_t LSM6DSR_TEMP_CALIBRATION_REQUIRED_SAMPLES_PER_STEP =
    TEMP_CALIBRATION_SECONDS_PER_STEP / (LSM6DSR_ODR_GYR_MICROS / 1e6);
static_assert(0x7FFF * LSM6DSR_TEMP_CALIBRATION_REQUIRED_SAMPLES_PER_STEP < 0x7FFFFFFF, "Temperature calibration sum overflow");

class LSM6DSRSensor : public Sensor {
    public:
        LSM6DSRSensor(uint8_t id, uint8_t address, float rotation, uint8_t sclPin, uint8_t sdaPin, int axisRemap=AXIS_REMAP_DEFAULT) :
            Sensor("LSM6DSRSensor", IMU_LSM6DSR, id, address, rotation, sclPin, sdaPin),
            axisRemap(axisRemap),
            sfusion(LSM6DSR_ODR_GYR_MICROS / 1e6f, LSM6DSR_ODR_ACC_MICROS / 1e6f, LSM6DSR_ODR_MAG_MICROS / 1e6f)
        {
        };
        ~LSM6DSRSensor(){};
        void initMMC();

        void motionSetup() override final;
        void motionLoop() override final;
        void startCalibration(int calibrationType) override final;
        void maybeCalibrateGyro();
        void maybeCalibrateAccel();
        void maybeCalibrateMag();
        
        void printTemperatureCalibrationState() override final;
        void printDebugTemperatureCalibrationState() override final;
        void resetTemperatureCalibrationState() override final {
            gyroTempCalibrator->reset();
            m_Logger.info("Temperature calibration state has been reset for sensorId:%i", sensorId);
        };
        void saveTemperatureCalibration() override final;

        void applyAccelCalibrationAndScale(sensor_real_t Axyz[3]);
        void applyMagCalibrationAndScale(sensor_real_t Mxyz[3]);

        bool hasGyroCalibration();
        bool hasAccelCalibration();
        bool hasMagCalibration();

        void onGyroRawSample(uint32_t dtMicros, int16_t x, int16_t y, int16_t z);
        void onAccelRawSample(uint32_t dtMicros, int16_t x, int16_t y, int16_t z);
        void onMagRawSample(uint32_t dtMicros, int16_t x, int16_t y, int16_t z);
        void readFIFO();

        void getMagnetometerXYZFromBuffer(uint8_t* data, int16_t* x, int16_t* y, int16_t* z);

        void remapGyroAccel(sensor_real_t* x, sensor_real_t* y, sensor_real_t* z);
        void remapMagnetometer(sensor_real_t* x, sensor_real_t* y, sensor_real_t* z);
        void getRemappedRotation(int16_t* x, int16_t* y, int16_t* z);
        void getRemappedAcceleration(int16_t* x, int16_t* y, int16_t* z);

        bool getTemperature(float* out);
    private:
        LSM6DSR imu {};
        int axisRemap;

        SlimeVR::Sensors::SensorFusionRestDetect sfusion;

        // clock sync and sample timestamping
        uint32_t sensorTime0 = 0;
        uint32_t sensorTime1 = 0;
        uint32_t localTime0 = 0;
        uint32_t localTime1 = 0;
        double sensorTimeRatio = 1;
        double sensorTimeRatioEma = 1;
        double sampleDtMicros = LSM6DSR_ODR_GYR_MICROS;
        uint32_t syncLatencyMicros = 0;
        uint32_t samplesSinceClockSync = 0;
        uint32_t timestamp0 = 0;
        uint32_t timestamp1 = 0;

        // scheduling
        uint32_t lastPollTime = micros();
        uint32_t lastClockPollTime = micros();
        #if LSM6DSR_DEBUG
        uint32_t cpuUsageMicros = 0;
        uint32_t lastCpuUsagePrinted = 0;
        uint32_t gyrReads = 0;
        uint32_t accReads = 0;
        uint32_t magReads = 0;
        uint16_t numFIFODropped = 0;
        uint16_t numFIFOFailedReads = 0;
        #endif

        uint32_t lastRotationPacketSent = 0;
        uint32_t lastTemperaturePacketSent = 0;

        struct LSM6DSRFIFO {
            uint8_t data[7];
            uint16_t length;
        } fifo {};
        float temperature = 0;
        GyroTemperatureCalibrator* gyroTempCalibrator = nullptr;
        sensor_real_t Gxyz[3] = {0};
        sensor_real_t Axyz[3] = {0};
        sensor_real_t Mxyz[3] = {0};

        double gscaleX = LSM6DSR_GSCALE;
        double gscaleY = LSM6DSR_GSCALE;
        double gscaleZ = LSM6DSR_GSCALE;

        double GOxyzStaticTempCompensated[3] = {0.0, 0.0, 0.0};

        bool isGyroCalibrated = false;
        bool isAccelCalibrated = false;
        bool isMagCalibrated = false;

        SlimeVR::Configuration::BMI160CalibrationConfig m_Calibration = {};
};

#endif
