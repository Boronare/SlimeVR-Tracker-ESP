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

#include "GlobalVars.h"
#include "magneto1.4.h"
#include <LSM6DSR.h>
#include "sensor.h"
#include "RestCalibrationDetector.h"

#include "SensorFusionRestDetect.h"
#include "../motionprocessing/types.h"
#include "../motionprocessing/RestDetection.h"

#define LSM6DSR_GYRO_RATE LSM6DSR_GY_ODR_417Hz
#define LSM6DSR_GYRO_RANGE LSM6DSR_1000dps

#define LSM6DSR_ACCEL_RATE LSM6DSR_XL_ODR_104Hz
#define LSM6DSR_ACCEL_RANGE LSM6DSR_4g

// note: if changing ODR or filter modes - adjust rest detection params and buffer size
#define LSM6DSR_TIMESTAMP_RESOLUTION_MICROS 25.0f

#define LSM6DSR_MAP_ODR_MICROS(micros) ((uint16_t)((micros) / LSM6DSR_TIMESTAMP_RESOLUTION_MICROS) * LSM6DSR_TIMESTAMP_RESOLUTION_MICROS)

constexpr float LSM6DSR_ODR_GYR_HZ = 416;
constexpr float LSM6DSR_ODR_ACC_HZ = 104;
constexpr float LSM6DSR_ODR_GYR_MICROS = LSM6DSR_MAP_ODR_MICROS(1.0f / LSM6DSR_ODR_GYR_HZ * 1e6f);
constexpr float LSM6DSR_ODR_ACC_MICROS = LSM6DSR_MAP_ODR_MICROS(1.0f / LSM6DSR_ODR_ACC_HZ * 1e6f);
// note: this value only sets polling and fusion update rate - HMC is internally sampled at 75hz, QMC at 200hz
#define LSM6DSR_MAG_RATE LSM6DSR_SH_ODR_13Hz
constexpr float LSM6DSR_ODR_MAG_HZ = 13;
constexpr float LSM6DSR_ODR_MAG_MICROS = 1.0f / LSM6DSR_ODR_MAG_HZ * 1e6f;


// Typical sensitivity
constexpr double LSM6DSR_GYRO_TYPICAL_SENSITIVITY_MDPS = 35.f;

constexpr std::pair<uint8_t, float> LSM6DSR_ACCEL_SENSITIVITY_LSB_MAP[] = {
    {LSM6DSR_2g, 16393.0f},
    {LSM6DSR_16g, 2049.0f},
    {LSM6DSR_4g, 8196.0f},
    {LSM6DSR_8g, 4098.0f}
};
constexpr double LSM6DSR_ACCEL_TYPICAL_SENSITIVITY_LSB = LSM6DSR_ACCEL_SENSITIVITY_LSB_MAP[LSM6DSR_ACCEL_RANGE].second;
constexpr double LSM6DSR_ASCALE = CONST_EARTH_GRAVITY / LSM6DSR_ACCEL_TYPICAL_SENSITIVITY_LSB;

// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr double LSM6DSR_GSCALE = ((LSM6DSR_GYRO_TYPICAL_SENSITIVITY_MDPS*1e-3)) * (PI / 180.0);
constexpr double gscaleX = LSM6DSR_GSCALE;
constexpr double gscaleY = LSM6DSR_GSCALE;
constexpr double gscaleZ = LSM6DSR_GSCALE;

class LSM6DSRSensor : public Sensor {
    public:
	static constexpr auto TypeID = ImuID::LSM6DSR;
	static constexpr uint8_t Address = 0x6A;
        LSM6DSRSensor(
            uint8_t id,
            uint8_t address,
            float rotation,
            uint8_t sclPin,
            uint8_t sdaPin,
            uint8_t
        )
            : Sensor(
                "LSM6DSRSensor",
                ImuID::LSM6DSR,
                id,
                address,
                rotation,
                sclPin,
                sdaPin
            )
            , sfusion(
                LSM6DSR_ODR_GYR_MICROS / 1e6f,
                LSM6DSR_ODR_ACC_MICROS / 1e6f,
                LSM6DSR_ODR_MAG_MICROS / 1e6f
            ) {
        };
        ~LSM6DSRSensor(){};
        void initMMC();

        void motionSetup() override final;
        void postSetup() override {};
        void motionLoop() override final;
        void startCalibration(int calibrationType) override final;
        SensorStatus getSensorState() override final { return m_status; }
	    void setFlag(uint16_t flagId, bool state) override {
            if (flagId == 1) {
                if(m_Config.flags&1 == state) return;
                m_Config.flags = (m_Config.flags&0b11111110) + state;
                magStatus = state ? MagnetometerStatus::MAG_ENABLED
                                : MagnetometerStatus::MAG_DISABLED;

                SlimeVR::Configuration::SensorConfig config;
                config.type = SlimeVR::Configuration::SensorConfigType::LSM6DSR;
                config.data.lsm6dsr = m_Config;
                configuration.setSensor(sensorId, config);
                configuration.save();

                // Reinitialize the sensor
                motionSetup();
            }
        };

        void maybeCalibrateGyro();
        void maybeCalibrateAccel();
        void maybeCalibrateMag();

        void applyAccelCalibrationAndScale(sensor_real_t Axyz[3]);
        void applyMagCalibrationAndScale(sensor_real_t Mxyz[3]);

        bool hasGyroCalibration();
        bool hasAccelCalibration();
        bool hasMagCalibration();

        void onGyroRawSample(uint32_t dtMicros, float x, float y, float z);
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
        double sampleDtMicros = LSM6DSR_ODR_GYR_MICROS;
        uint32_t timestamp1 = 0;
        uint32_t actualSensorMicros = 0;
        bool magCalibrating = false;
        bool CaliDebug = false;
        int16_t *Cx,*Cy,*Cz;
        int8_t *ignoreList;
        uint8_t Cf = 0, Cr = 0;

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
        sensor_real_t Gxyz[3] = {0};
        sensor_real_t Axyz[3] = {0};
        sensor_real_t Mxyz[3] = {0};

        double GOxyzStaticTempCompensated[3] = {0.0, 0.0, 0.0};

        bool isGyroCalibrated = false;
        bool isAccelCalibrated = false;
        bool isMagCalibrated = false;
        SensorStatus m_status = SensorStatus::SENSOR_OFFLINE;

        SlimeVR::Configuration::LSM6DSRSensorConfig m_Config = {};

        SlimeVR::Sensors::RestCalibrationDetector calibrationDetector;
};

#endif
