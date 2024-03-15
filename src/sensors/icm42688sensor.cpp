/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain, S.J. Remington & SlimeVR contributors

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

#include "icm42688sensor.h"
#include "globals.h"
#include "helper_3dmath.h"
#include <i2cscan.h>
#include "calibration.h"
#include "magneto1.4.h"
#include "GlobalVars.h"

constexpr float gscale = (1000. / 32768.0) * (PI / 180.0); // gyro LSB/d/s -> rad/s
constexpr float ascale = (4. / 32768.) * CONST_EARTH_GRAVITY; // accel LSB/G -> m/s^2
constexpr float godr = 200.0;
#define GODR GODR_200Hz

void ICM42688Sensor::motionSetup() {
    // initialize device
    uint8_t temp;
    I2Cdev::readByte(addr, ICM42688_WHO_AM_I, &temp);
    if(!(temp == 0x47 || temp == 0xDB)) {
        m_Logger.fatal("Can't connect to ICM42688 (reported device ID 0x%02x) at address 0x%02x", temp, addr);
        return;
    }

    m_Logger.info("Connected to ICM42688 (reported device ID 0x%02x) at address 0x%02x", temp, addr);

    if (I2CSCAN::hasDevOnBus(addr_mag)) {
        I2Cdev::readByte(addr_mag, MMC5983MA_PRODUCT_ID, &temp);
        if(!(temp == 0x30)) {
            m_Logger.fatal("Can't connect to MMC5983MA (reported device ID 0x%02x) at address 0x%02x", temp, addr_mag);
            m_Logger.info("Magnetometer unavailable!");
            magExists = false;
        } else {
            m_Logger.info("Connected to MMC5983MA (reported device ID 0x%02x) at address 0x%02x", temp, addr_mag);
            magExists = true;
        }
    } else {
        m_Logger.info("Magnetometer unavailable!");
        magExists = false;
    }

    if (magExists) {
        I2Cdev::writeByte(addr_mag, MMC5983MA_CONTROL_1, 0x80); // Reset MMC now
    }

    I2Cdev::writeByte(addr, ICM42688_DEVICE_CONFIG, 1); // reset
    delay(2); // wait 1ms for reset
    I2Cdev::readByte(addr, ICM42688_INT_STATUS, &temp); // clear reset done int flag
    I2Cdev::writeByte(addr, ICM42688_INT_SOURCE0, 0); // disable ints
    I2Cdev::writeByte(addr, ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
    I2Cdev::writeByte(addr, ICM42688_PWR_MGMT0, gMode_LN << 2 | aMode_LP); // set accel and gyro modes (low noise)
    delay(1); // wait >200us (datasheet 14.36)
    I2Cdev::writeByte(addr, ICM42688_ACCEL_CONFIG0, AFS_4G << 5 | AODR_50Hz); // set accel ODR and FS (200hz, 8g)
    I2Cdev::writeByte(addr, ICM42688_GYRO_CONFIG0, GFS_1000DPS << 5 | GODR); // set gyro ODR and FS (1khz, 2000dps)
    I2Cdev::writeByte(addr, ICM42688_GYRO_ACCEL_CONFIG0, 0x44); // set gyro and accel bandwidth to ODR/10
	delay(100); // 10ms Accel, 30ms Gyro startup

    if (magExists) {
        I2Cdev::writeByte(addr_mag, MMC5983MA_CONTROL_0, 0x08); // SET
        delayMicroseconds(1); // auto clear after 500ns
        I2Cdev::writeByte(addr_mag, MMC5983MA_CONTROL_0, 0x20); // auto SET/RESET
        I2Cdev::writeByte(addr_mag, MMC5983MA_CONTROL_1, MBW_400Hz); // set mag BW (400Hz or ~50% duty cycle with 200Hz ODR)
        I2Cdev::writeByte(addr_mag, MMC5983MA_CONTROL_2, 0x80 | (MSET_2000 << 4) | 0x08 | MODR_200Hz); // continuous measurement mode, set sample rate, auto SET/RESET, set SET/RESET rate (200Hz ODR, 2000 samples between SET/RESET)
    }

    // turn on while flip back to calibrate. then, flip again after 5 seconds.
    // TODO: Move calibration invoke after calibrate button on slimeVR server available
    accel_read();
    if(Axyz[2] < -0.75f) {
        m_Logger.info("Flip front to confirm start calibration");
        ledManager.pattern(500,500,3);

        accel_read();
        if(Axyz[2] > 0.75f) {
            m_Logger.debug("Starting calibration...");
            startCalibration(0);
        }
    }

    // Initialize the configuration
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type) {
        case SlimeVR::Configuration::CalibrationConfigType::ICM42688:
            m_Calibration = sensorCalibration.data.icm42688;
            break;

        case SlimeVR::Configuration::CalibrationConfigType::NONE:
            m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
            break;

        default:
            m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
        }
    }

    I2Cdev::writeByte(addr, ICM42688_FIFO_CONFIG, 0x00); // FIFO bypass mode
    I2Cdev::writeByte(addr, ICM42688_FSYNC_CONFIG, 0x00); // disable FSYNC
    I2Cdev::readByte(addr, ICM42688_TMST_CONFIG, &temp); // disable FSYNC
    I2Cdev::writeByte(addr, ICM42688_TMST_CONFIG, temp & 0xfd); // disable FSYNC
    I2Cdev::writeByte(addr, ICM42688_FIFO_CONFIG1, 0x02); // enable FIFO gyro only
    I2Cdev::writeByte(addr, ICM42688_FIFO_CONFIG, 0x40); // begin FIFO stream

    working = true;
    configured = true;
}

void ICM42688Sensor::motionLoop() {
	uint8_t rawCount[2];
	I2Cdev::readBytes(addr, ICM42688_FIFO_COUNTH, 2, &rawCount[0]);
	uint16_t count = (uint16_t)(rawCount[0] << 8 | rawCount[1]); // Turn the 16 bits into a unsigned 16-bit value
	// count += 32; // Add a few read buffer packets (4 ms)
	uint16_t packets = count / 8;								 // Packet size 8 bytes
	uint8_t rawData[8];
    // I2Cdev::readBytes(addr, ICM42688_FIFO_DATA, count, &rawData[0]); // Read buffer

    accel_read();
    parseAccelData();

    if (magExists) {
        mag_read();
        parseMagData();
    }

	for (uint16_t i = 0; i < packets; i++) {
        I2Cdev::readBytes(addr, ICM42688_FIFO_DATA, 8, &rawData[0]); // Read buffer
		constexpr uint16_t index = 0; // Packet size 8 bytes
            // Serial.printf("\nHdr : %x ",rawData[index]);
		if ((rawData[index] & 0x80) == 0x80) {
			continue; // Skip empty packets
		}
        if ((rawData[index] & 0x20)){
            // combine into 16 bit values
            int16_t raw0 = (int16_t)((((int16_t)rawData[index + 1]) << 8) | rawData[index + 2]); // gx
            int16_t raw1 = (int16_t)((((int16_t)rawData[index + 3]) << 8) | rawData[index + 4]); // gy
            int16_t raw2 = (int16_t)((((int16_t)rawData[index + 5]) << 8) | rawData[index + 6]); // gz
            if (raw0 < -32766 || raw1 < -32766 || raw2 < -32766) {
                continue; // Skip invalid data
            }
            // Serial.printf("Gyr : %+5d / %+5d / %+5d",raw0,raw1,raw2);
            Gxyz[0] = raw0 * gscale; //gres
            Gxyz[1] = raw1 * gscale; //gres
            Gxyz[2] = raw2 * gscale; //gres
            for(uint8_t k=0;k<3;k++){
                Gavg[k]=0.99*Gavg[k]+0.01*Gxyz[k];
                Gdev[k]=0.99*Gdev[k]+0.01*sq(Gavg[k]-Gxyz[k]);
                if(Gdev[k]<0.0001) m_Calibration.G_off[k]=m_Calibration.G_off[k]*0.99+Gavg[k]*0.01;
            }
            parseGyroData();

            // TODO: mag axes will be different, make sure to change them???
            sfusion.updateGyro(Gxyz,1.0/godr);
        }
    }

    // sfusion.updateAcc(Axyz);

    if (magExists)
        sfusion.updateMag(Mxyz);
    
    fusedRotation = sfusion.getQuaternionQuat();
    fusedRotation *= sensorOffset;
    acceleration = sfusion.getLinearAccVec();
    setFusedRotationReady();
    setAccelerationReady();
}

void ICM42688Sensor::accel_read() {
    uint8_t rawAccel[6];
    I2Cdev::readBytes(addr, ICM42688_ACCEL_DATA_X1, 6, &rawAccel[0]);
	float raw0 = (int16_t)((((int16_t)rawAccel[0]) << 8) | rawAccel[1]);
	float raw1 = (int16_t)((((int16_t)rawAccel[2]) << 8) | rawAccel[3]);
	float raw2 = (int16_t)((((int16_t)rawAccel[4]) << 8) | rawAccel[5]);
	Axyz[0] = raw0 * ascale;
	Axyz[1] = raw1 * ascale;
	Axyz[2] = raw2 * ascale;
}

void ICM42688Sensor::gyro_read() {
    uint8_t rawGyro[6];
    I2Cdev::readBytes(addr, ICM42688_GYRO_DATA_X1, 6, &rawGyro[0]);
	float raw0 = (int16_t)((((int16_t)rawGyro[0]) << 8) | rawGyro[1]);
	float raw1 = (int16_t)((((int16_t)rawGyro[2]) << 8) | rawGyro[3]);
	float raw2 = (int16_t)((((int16_t)rawGyro[4]) << 8) | rawGyro[5]);
	Gxyz[0] = raw0 * gscale;
	Gxyz[1] = raw1 * gscale;
	Gxyz[2] = raw2 * gscale;
}

void ICM42688Sensor::mag_read() {
    if (!magExists) return;
    uint8_t rawMag[7];
    I2Cdev::readBytes(addr_mag, MMC5983MA_XOUT_0, 7, &rawMag[0]);
    double raw0 = (uint32_t)(rawMag[0] << 10 | rawMag[1] << 2 | (rawMag[6] & 0xC0) >> 6);
    double raw1 = (uint32_t)(rawMag[2] << 10 | rawMag[3] << 2 | (rawMag[6] & 0x30) >> 4);
    double raw2 = (uint32_t)(rawMag[4] << 10 | rawMag[5] << 2 | (rawMag[6] & 0x0C) >> 2);
	Mxyz[0] = (raw0 - MMC5983MA_offset) * MMC5983MA_mRes;
	Mxyz[1] = (raw1 - MMC5983MA_offset) * MMC5983MA_mRes;
	Mxyz[2] = (raw2 - MMC5983MA_offset) * MMC5983MA_mRes;
}

void ICM42688Sensor::startCalibration(int calibrationType) {
    ledManager.on();
    m_Logger.debug("Gathering raw data for device calibration...");
    constexpr int calibrationSamples = godr*5;
    double GxyzC[3] = {0, 0, 0};

    // Wait for sensor to calm down before calibration
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
    ledManager.pattern(100,100,5);

    for (int i = 0; i < calibrationSamples; i++) {
        uint8_t raw;
        do{
            I2Cdev::readByte(addr, ICM42688_INT_STATUS, &raw);
            delayMicroseconds(100);
        }while(!(raw&0x08));
        gyro_read();
        GxyzC[0] += Gxyz[0];
        GxyzC[1] += Gxyz[1];
        GxyzC[2] += Gxyz[2];
    }
    GxyzC[0] /= calibrationSamples;
    GxyzC[1] /= calibrationSamples;
    GxyzC[2] /= calibrationSamples;
    ledManager.pattern(100,100,5);

#ifdef DEBUG_SENSOR
    m_Logger.trace("Gyro calibration results: %f %f %f", GxyzC[0], GxyzC[1], GxyzC[2]);
#endif

    // TODO: use offset registers?
    m_Calibration.G_off[0] = GxyzC[0];
    m_Calibration.G_off[1] = GxyzC[1];
    m_Calibration.G_off[2] = GxyzC[2];
    //    MagnetoCalibration* magneto = new MagnetoCalibration();

    // m_Logger.info("Put the device into 6 unique orientations (all sides), leave it still and do not hold/touch for 3 seconds each");

    //     RestDetectionParams calibrationRestDetectionParams;
    //     calibrationRestDetectionParams.restMinTimeMicros = 0.5 * 1e6;
    //     calibrationRestDetectionParams.restThAcc = 0.25f;
    //     RestDetection calibrationRestDetection(
    //         calibrationRestDetectionParams,
    //         BMI160_ODR_GYR_MICROS / 1e6f,
    //         BMI160_ODR_ACC_MICROS / 1e6f
    //     );

    //     constexpr uint16_t expectedPositions = 6;
    //     constexpr uint16_t numSamplesPerPosition = 32;

    //     uint16_t numPositionsRecorded = 0;
    //     uint16_t numCurrentPositionSamples = 0;
    //     bool waitForMotion = true;

    //     float* accelCalibrationChunk = new float[numSamplesPerPosition * 3];
    //     ledManager.pattern(100, 100, 6);
    //     ledManager.on();
    //     m_Logger.info("Gathering accelerometer data...");
    //     m_Logger.info("Waiting for position %i, you can leave the device as is...", numPositionsRecorded + 1);
    //     while (true) {
    //         int16_t ax, ay, az;
    //         imu.getAcceleration(&ax, &ay, &az);
    //         sensor_real_t scaled[3];
    //         scaled[0] = ax * BMI160_ASCALE;
    //         scaled[1] = ay * BMI160_ASCALE;
    //         scaled[2] = az * BMI160_ASCALE;

    //         calibrationRestDetection.updateAcc(BMI160_ODR_ACC_MICROS, scaled);

    //         if (waitForMotion) {
    //             if (!calibrationRestDetection.getRestDetected()) {
    //                 waitForMotion = false;
    //             }
    //             delayMicroseconds(BMI160_ODR_ACC_MICROS);
    //             continue;
    //         }

    //         if (calibrationRestDetection.getRestDetected()) {
    //             ledManager.on();
    //             const uint16_t i = numCurrentPositionSamples * 3;
    //             accelCalibrationChunk[i + 0] = ax;
    //             accelCalibrationChunk[i + 1] = ay;
    //             accelCalibrationChunk[i + 2] = az;
    //             numCurrentPositionSamples++;

    //             if (numCurrentPositionSamples >= numSamplesPerPosition) {
    //                 for (int i = 0; i < numSamplesPerPosition; i++) {
    //                     magneto->sample(
    //                         accelCalibrationChunk[i * 3 + 0],
    //                         accelCalibrationChunk[i * 3 + 1],
    //                         accelCalibrationChunk[i * 3 + 2]
    //                     );
    //                 }
    //                 numPositionsRecorded++;
    //                 numCurrentPositionSamples = 0;
    //                 if (numPositionsRecorded < expectedPositions) {
    //                     ledManager.pattern(50, 50, 2);
    //                     m_Logger.info("Recorded, waiting for position %i...", numPositionsRecorded + 1);
    //                     waitForMotion = true;
    //                 }
    //             }
    //         } else {
    //             numCurrentPositionSamples = 0;
    //         }

    //         if (numPositionsRecorded >= expectedPositions) break;

    //         delayMicroseconds(BMI160_ODR_ACC_MICROS);
    //     }
    //     ledManager.off();
    //     m_Logger.debug("Calculating accelerometer calibration data...");
    //     delete[] accelCalibrationChunk;
    // #endif

    // float A_BAinv[4][3];
    // magneto->current_calibration(A_BAinv);
    // delete magneto;

    // m_Logger.debug("Finished calculating accelerometer calibration");
    // m_Logger.debug("Accelerometer calibration matrix:");
    // m_Logger.debug("{");
    // for (int i = 0; i < 3; i++) {
    //     m_Calibration.A_B[i] = A_BAinv[0][i];
    //     m_Calibration.A_Ainv[0][i] = A_BAinv[1][i];
    //     m_Calibration.A_Ainv[1][i] = A_BAinv[2][i];
    //     m_Calibration.A_Ainv[2][i] = A_BAinv[3][i];
    //     m_Logger.debug("  %f, %f, %f, %f", A_BAinv[0][i], A_BAinv[1][i], A_BAinv[2][i], A_BAinv[3][i]);
    // }
    // m_Logger.debug("}");

    m_Logger.debug("Saving the calibration data");

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::ICM42688;
    calibration.data.icm42688 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    ledManager.off();
    m_Logger.debug("Saved the calibration data");

    m_Logger.info("Calibration data gathered");
    I2Cdev::writeByte(addr, ICM42688_SIGNAL_PATH_RESET, 0x02); // flush FIFO before return
}

void ICM42688Sensor::parseMagData() {
    float temp[3];

    //apply offsets and scale factors from Magneto
    for (unsigned i = 0; i < 3; i++) {
        temp[i] = (Mxyz[i] - m_Calibration.M_B[i]);
        #if useFullCalibrationMatrix == true
            Mxyz[i] = m_Calibration.M_Ainv[i][0] * temp[0] + m_Calibration.M_Ainv[i][1] * temp[1] + m_Calibration.M_Ainv[i][2] * temp[2];
        #else
            Mxyz[i] = temp[i];
        #endif
    }
}

void ICM42688Sensor::parseAccelData() {
    float temp[3];

    //apply offsets (bias) and scale factors from Magneto
    for (unsigned i = 0; i < 3; i++) {
        temp[i] = (Axyz[i] - m_Calibration.A_B[i]);
        #if useFullCalibrationMatrix == true
            Axyz[i] = m_Calibration.A_Ainv[i][0] * temp[0] + m_Calibration.A_Ainv[i][1] * temp[1] + m_Calibration.A_Ainv[i][2] * temp[2];
        #else
            Axyz[i] = temp[i];
        #endif
    }
}

void ICM42688Sensor::parseGyroData() {
    Gxyz[0] = (Gxyz[0] - m_Calibration.G_off[0]);
    Gxyz[1] = (Gxyz[1] - m_Calibration.G_off[1]);
    Gxyz[2] = (Gxyz[2] - m_Calibration.G_off[2]);
}
