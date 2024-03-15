/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 S.J. Remington & SlimeVR contributors

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

#include "lsm6dsrsensor.h"
#include "GlobalVars.h"
#include <map>

void LSM6DSRSensor::initMMC(){    /* Configure MAG interface and setup mode */

    imu.setMagDevice(0x30,0x00);
    delay(3);
    
    /* Configure MMC5603NJ Sensor */

    //Reboot Magnetometer
    imu.setMagRegister(0x1C, 0x80);
    delay(30);
    //Set BW=00(6.6ms measurment time)
    imu.setMagRegister(0x1C, 0x00);
    delay(3);
    //Set ODR
    imu.setMagRegister(0x1A, 26);
    delay(3);
    //Enable Continuous mode & Auto SR
    imu.setMagRegister(0x1B, 0xA0);
    delay(3);
    //Start Continuous mode
    imu.setMagRegister(0x1D, 0x10);
}

void LSM6DSRSensor::motionSetup() {
    // initialize device
    imu.initialize(
        addr,
        LSM6DSR_GYRO_RATE,
        LSM6DSR_GYRO_RANGE,
        LSM6DSR_ACCEL_RATE,
        LSM6DSR_ACCEL_RANGE
    );
    #if !USE_6_AXIS
            initMMC();
    #endif

    if (!imu.testConnection()) {
        m_Logger.fatal("Can't connect to LSM6DSR (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    m_Logger.info("Connected to LSM6DSR (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);

    // Initialize the configuration
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type) {
        case SlimeVR::Configuration::CalibrationConfigType::BMI160:
            m_Calibration = sensorCalibration.data.bmi160;
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

    int16_t ax, ay, az;
    getRemappedAcceleration(&ax, &ay, &az);
    Serial.printf("Acc : %d %d %d\n",ax,ay,az);
    float g_az = (float)az / LSM6DSR_ACCEL_TYPICAL_SENSITIVITY_LSB;
    if (g_az < -0.75f) {
        m_Logger.info("Flip front to confirm start calibration");
        ledManager.off();
        delay(500);

        ledManager.pattern(500,500,3);
        getRemappedAcceleration(&ax, &ay, &az);
        g_az = (float)az / LSM6DSR_ACCEL_TYPICAL_SENSITIVITY_LSB;
        if (g_az > 0.75f) {
            m_Logger.debug("Starting calibration...");
            startCalibration(0);
        }

        ledManager.off();
    }

    {
        #define IS_INT16_CLIPPED(value) (value == INT16_MIN || value == INT16_MAX)
        const bool anyClipped = IS_INT16_CLIPPED(ax) || IS_INT16_CLIPPED(ay) || IS_INT16_CLIPPED(az);
        const bool anyZero = ax == 0 || ay == 0 || az == 0;
        if (anyClipped || anyZero) {
            m_Logger.warn("---------------- WARNING -----------------");
            m_Logger.warn("One or more accelerometer axes may be dead");
            m_Logger.warn("Acceleration: %i %i %i (Z = %f G)",
                ax, ay, az, (float)az / LSM6DSR_ACCEL_TYPICAL_SENSITIVITY_LSB);
            m_Logger.warn("---------------- WARNING -----------------");
        }
    }

    // allocate temperature memory after calibration because OOM
    gyroTempCalibrator = new GyroTemperatureCalibrator(
        SlimeVR::Configuration::CalibrationConfigType::BMI160,
        sensorId,
        1000/LSM6DSR_GYRO_TYPICAL_SENSITIVITY_MDPS,
        LSM6DSR_TEMP_CALIBRATION_REQUIRED_SAMPLES_PER_STEP
    );

    #if BMI160_USE_TEMPCAL
        gyroTempCalibrator->loadConfig(1000/LSM6DSR_GYRO_TYPICAL_SENSITIVITY_MDPS);
        if (gyroTempCalibrator->config.hasCoeffs) {
            float GOxyzAtTemp[3];
            gyroTempCalibrator->approximateOffset(m_Calibration.temperature, GOxyzAtTemp);
            for (uint32_t i = 0; i < 3; i++) {
                GOxyzStaticTempCompensated[i] = m_Calibration.G_off[i] - GOxyzAtTemp[i];
            }
        }
    #endif

    #if BMI160_USE_SENSCAL
    {
        String localDevice = WiFi.macAddress();
        for (auto const& offsets : sensitivityOffsets) {
            if (!localDevice.equals(offsets.mac)) continue;
            if (offsets.sensorId != sensorId) continue;

            #define LSM6DSR_CALCULATE_SENSITIVTY_MUL(degrees) (1.0 / (1.0 - ((degrees)/(360.0 * offsets.spins))))

            gscaleX = LSM6DSR_GSCALE * LSM6DSR_CALCULATE_SENSITIVTY_MUL(offsets.x);
            gscaleY = LSM6DSR_GSCALE * LSM6DSR_CALCULATE_SENSITIVTY_MUL(offsets.y);
            gscaleZ = LSM6DSR_GSCALE * LSM6DSR_CALCULATE_SENSITIVTY_MUL(offsets.z);
            m_Logger.debug("Custom sensitivity offset enabled: %s %s",
                offsets.mac,
                offsets.sensorId == SENSORID_PRIMARY ? "primary" : "aux"
            );
        }
    }
    #endif

    isGyroCalibrated = hasGyroCalibration();
    isAccelCalibrated = hasAccelCalibration();
    isMagCalibrated = hasMagCalibration();
    m_Logger.info("Calibration data for gyro: %s", isGyroCalibrated ? "found" : "not found");
    m_Logger.info("Calibration data for accel: %s", isAccelCalibrated ? "found" : "not found");
    m_Logger.info("Calibration data for mag: %s", isMagCalibrated ? "found" : "not found");

    imu.resetFIFO();
    delay(2);

    working = true;
}

void LSM6DSRSensor::motionLoop() {
    #if ENABLE_INSPECTION
    {
        int16_t rX, rY, rZ, aX, aY, aZ;
        getRemappedRotation(&rX, &rY, &rZ);
        getRemappedAcceleration(&aX, &aY, &aZ);

        networkConnection.sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, 0, 0, 0, 255);
    }
    #endif
            getTemperature(&temperature);

    {
        uint32_t now = micros();
        constexpr uint32_t LSM6DSR_TARGET_POLL_INTERVAL_MICROS = 6000;
        uint32_t elapsed = now - lastPollTime;
        if (elapsed >= LSM6DSR_TARGET_POLL_INTERVAL_MICROS) {
            lastPollTime = now - (elapsed - LSM6DSR_TARGET_POLL_INTERVAL_MICROS);

            readFIFO();
            optimistic_yield(100);
            if (!sfusion.isUpdated()) return;
            sfusion.clearUpdated();
        }
    }

    {
        uint32_t now = micros();
        constexpr float maxSendRateHz = 0.5f;
        constexpr uint32_t sendInterval = 1.0f/maxSendRateHz * 1e6;
        uint32_t elapsed = now - lastTemperaturePacketSent;
        if (elapsed >= sendInterval) {
            lastTemperaturePacketSent = now - (elapsed - sendInterval);
            #if BMI160_TEMPCAL_DEBUG
                uint32_t isCalibrating = gyroTempCalibrator->isCalibrating() ? 10000 : 0;
                networkConnection.sendTemperature(sensorId, isCalibrating + 10000 + (gyroTempCalibrator->config.samplesTotal * 100) + temperature);
            #else
                networkConnection.sendTemperature(sensorId, temperature);
            #endif
            optimistic_yield(100);
        }
    }

    {
        uint32_t now = micros();
        constexpr float maxSendRateHz = 60.0f;
        constexpr uint32_t sendInterval = 1.0f/maxSendRateHz * 1e6;
        uint32_t elapsed = now - lastRotationPacketSent;
        if (elapsed >= sendInterval) {
            lastRotationPacketSent = now - (elapsed - sendInterval);

            fusedRotation = sfusion.getQuaternionQuat();
            float lastAcceleration[]{acceleration.x,acceleration.y,acceleration.z};
            acceleration=sfusion.getLinearAccVec();
            if(!OPTIMIZE_UPDATES ||!(abs(lastAcceleration[0])<0.15 &&
                                    abs(lastAcceleration[1])<0.15 && 
                                    abs(lastAcceleration[2])<0.15 &&
                                    abs(acceleration.x)<0.15 &&
                                    abs(acceleration.y)<0.15 && 
                                    abs(acceleration.z)<0.15))
                setAccelerationReady();

            fusedRotation *= sensorOffset;
            if (!OPTIMIZE_UPDATES || !lastFusedRotationSent.equalsWithEpsilon(fusedRotation))
            {
                newFusedRotation = true;
                lastFusedRotationSent = fusedRotation;
            }

            optimistic_yield(100);
        }
    }
}

void LSM6DSRSensor::readFIFO() {
    if (!imu.getFIFOCount(&fifo.length)) {
        #if BMI160_DEBUG
            numFIFOFailedReads++;
        #endif
        return;
    }

    if (fifo.length <= 1){
        
        if (!imu.getFIFOEnabled()) {
            // initialize device
            imu.initialize(
                addr,
                LSM6DSR_GYRO_RATE,
                LSM6DSR_GYRO_RANGE,
                LSM6DSR_ACCEL_RATE,
                LSM6DSR_ACCEL_RANGE
            );
            #if !USE_6_AXIS
                    initMMC();
            #endif
            imu.resetFIFO();
            delay(2);
        }
        return;
    }

    optimistic_yield(100);

    int16_t gx, gy, gz;
    int32_t sgx = 0, sgy = 0, sgz = 0;
    int16_t ax, ay, az;
    int16_t mx, my, mz;
    uint8_t samples = 0;

    // #if !USE_6_AXIS
    //     imu.getMagnetometer(&mx,&my,&mz);
    //     onMagRawSample(samplingRateInMillis*1000, mx, my, mz);
    // #endif
    // imu.getAcceleration(&ax,&ay,&az);
    // onAccelRawSample(samplingRateInMillis*1000, ax, ay, az);

    for (uint32_t i = 0; i < fifo.length;i++) {
        if (!imu.getFIFOBytes(fifo.data)) {
            #if BMI160_DEBUG
                numFIFOFailedReads++;
            #endif
            return;
        }
        // Serial.printf("\nTag : %x , CNT/PAR: %d DATA:%2x%2x%2x%2x%2x%2x ",fifo.data[0]>>3,fifo.data[0]&6>>1,fifo.data[1],fifo.data[2],fifo.data[3],fifo.data[4],fifo.data[5],fifo.data[6]);

        // ignore interrupt tags in header
        uint8_t header = fifo.data[0]>>3;
        switch(header){
        case LSM6DSR_TIMESTAMP_TAG:
            timestamp1 = (uint32_t)(
                (((uint32_t)fifo.data[4]) << 24) |
                (((uint32_t)fifo.data[3]) << 16) |
                (((uint32_t)fifo.data[2]) << 8)  |
                ((uint32_t)fifo.data[1])
            );
            // Serial.printf("Timestamp:%d\n",timestamp1);
            break;
        case LSM6DSR_GYRO_NC_TAG:
            gx = (((int16_t)fifo.data[2]) << 8) | fifo.data[1];
            gy = (((int16_t)fifo.data[4]) << 8) | fifo.data[3];
            gz = (((int16_t)fifo.data[6]) << 8) | fifo.data[5];
            // Serial.printf("Gx:%d, Gy:%d, Gz:%d\n",gx,gy,gz);
            sgx += gx;
            sgy += gy;
            sgz += gz;
            samples++;

            break;
        case LSM6DSR_SENSORHUB_SLAVE1_TAG:
            mx = ((((int16_t)fifo.data[1]) << 8) | fifo.data[2])-32768;
            my = ((((int16_t)fifo.data[3]) << 8) | fifo.data[4])-32768;
            mz = ((((int16_t)fifo.data[5]) << 8) | fifo.data[6])-32768;
            // Serial.printf("Mx:%d, My:%d, Mz:%d\n",mx,my,mz);
            onMagRawSample(LSM6DSR_ODR_MAG_MICROS,mx,my,mz);
            break;
        case LSM6DSR_XL_NC_TAG:
            ax = (((int16_t)fifo.data[2]) << 8) | fifo.data[1];
            ay = (((int16_t)fifo.data[4]) << 8) | fifo.data[3];
            az = (((int16_t)fifo.data[6]) << 8) | fifo.data[5];
            // Serial.printf("Ax:%d, Ay:%d, Az:%d\n",ax,ay,az);
            onAccelRawSample(LSM6DSR_ODR_ACC_MICROS,ax,ay,az);
            break;
        default:
            Serial.printf("UNKNOWN::%d\n",header);
        }
    }
    if(samples){
        // gx = (sgx+lx)/samples;
        // lx = (sgx+lx)%samples;
        // gy = (sgy+ly)/samples;
        // ly = (sgy+ly)%samples;
        // gz = (sgz+lz)/samples;
        // lz = (sgz+lz)%samples;
        // onGyroRawSample(sampleDtMicros*samples, gx, gy, gz);
        onGyroRawSample((timestamp1-timestamp0)*25,(float)sgx/samples,(float)sgy/samples,(float)sgz/samples);
        timestamp0 = timestamp1;
    }
}

void LSM6DSRSensor::onGyroRawSample(uint32_t dtMicros, float x, float y, float z) {
    #if BMI160_DEBUG
        gyrReads++;
    #endif

    #if BMI160_USE_TEMPCAL
        bool restDetected = sfusion.getRestDetected();
        gyroTempCalibrator->updateGyroTemperatureCalibration(temperature, restDetected, x, y, z);
    #endif

    sensor_real_t gyroCalibratedStatic[3];
    gyroCalibratedStatic[0] = (sensor_real_t)((((double)x - m_Calibration.G_off[0]) * gscaleX));
    gyroCalibratedStatic[1] = (sensor_real_t)((((double)y - m_Calibration.G_off[1]) * gscaleY));
    gyroCalibratedStatic[2] = (sensor_real_t)((((double)z - m_Calibration.G_off[2]) * gscaleZ));

    #if BMI160_USE_TEMPCAL
    float GOxyz[3];
    if (gyroTempCalibrator->approximateOffset(temperature, GOxyz)) {
        Gxyz[0] = (sensor_real_t)((((double)x - GOxyz[0] - GOxyzStaticTempCompensated[0]) * gscaleX));
        Gxyz[1] = (sensor_real_t)((((double)y - GOxyz[1] - GOxyzStaticTempCompensated[1]) * gscaleY));
        Gxyz[2] = (sensor_real_t)((((double)z - GOxyz[2] - GOxyzStaticTempCompensated[2]) * gscaleZ));
    }
    else
    #endif
    {
        Gxyz[0] = gyroCalibratedStatic[0];
        Gxyz[1] = gyroCalibratedStatic[1];
        Gxyz[2] = gyroCalibratedStatic[2];
    }
    remapGyroAccel(&Gxyz[0], &Gxyz[1], &Gxyz[2]);

    sfusion.updateGyro(Gxyz, (double)dtMicros * 1.0e-6);

    optimistic_yield(100);
}
void LSM6DSRSensor::onAccelRawSample(uint32_t dtMicros, int16_t x, int16_t y, int16_t z) {
    #if BMI160_DEBUG
        accReads++;
    #endif
    Axyz[0] = (sensor_real_t)x;
    Axyz[1] = (sensor_real_t)y;
    Axyz[2] = (sensor_real_t)z;
    applyAccelCalibrationAndScale(Axyz);
    remapGyroAccel(&Axyz[0], &Axyz[1], &Axyz[2]);
    sfusion.updateAcc(Axyz, dtMicros);

    optimistic_yield(100);
}
void LSM6DSRSensor::onMagRawSample(uint32_t dtMicros, int16_t x, int16_t y, int16_t z) {
    #if BMI160_DEBUG
        magReads++;
    #endif

    #if !USE_6_AXIS
    Mxyz[0] = (sensor_real_t)x;
    Mxyz[1] = (sensor_real_t)y;
    Mxyz[2] = (sensor_real_t)z;
    applyMagCalibrationAndScale(Mxyz);
    remapMagnetometer(&Mxyz[0], &Mxyz[1], &Mxyz[2]);
    sfusion.updateMag(Mxyz);
    #endif
}

void LSM6DSRSensor::printTemperatureCalibrationState() {
    const auto degCtoF = [](float degC) { return (degC * 9.0f/5.0f) + 32.0f; };

    m_Logger.info("Sensor %i temperature calibration state:", sensorId);
    m_Logger.info("  current temp: %0.4f C (%0.4f F)", temperature, degCtoF(temperature));
    auto printTemperatureRange = [&](const char* label, float min, float max) {
        m_Logger.info("  %s: min %0.4f C max %0.4f C (min %0.4f F max %0.4f F)",
            label, min, max, degCtoF(min), degCtoF(max)
        );
    };
    printTemperatureRange("total range",
        TEMP_CALIBRATION_MIN,
        TEMP_CALIBRATION_MAX
    );
    printTemperatureRange("calibrated range",
        gyroTempCalibrator->config.minTemperatureRange,
        gyroTempCalibrator->config.maxTemperatureRange
    );
    m_Logger.info("  done: %0.1f%", gyroTempCalibrator->config.getCalibrationDonePercent());
}
void LSM6DSRSensor::printDebugTemperatureCalibrationState() {
    m_Logger.info("Sensor %i gyro odr %f hz, sensitivity %f lsb",
        sensorId,
        LSM6DSR_ODR_GYR_HZ,
        1000/LSM6DSR_GYRO_TYPICAL_SENSITIVITY_MDPS
    );
    m_Logger.info("Sensor %i temperature calibration matrix (tempC x y z):", sensorId);
    m_Logger.info("BUF %i %i", sensorId, TEMP_CALIBRATION_BUFFER_SIZE);
    m_Logger.info("SENS %i %f", sensorId, 1000/LSM6DSR_GYRO_TYPICAL_SENSITIVITY_MDPS);
    m_Logger.info("DATA %i", sensorId);
    for (int i = 0; i < TEMP_CALIBRATION_BUFFER_SIZE; i++) {
        m_Logger.info("%f %f %f %f",
            gyroTempCalibrator->config.samples[i].t,
            gyroTempCalibrator->config.samples[i].x,
            gyroTempCalibrator->config.samples[i].y,
            gyroTempCalibrator->config.samples[i].z
        );
    }
    m_Logger.info("END %i", sensorId);
    m_Logger.info("y = %f + (%fx) + (%fxx) + (%fxxx)", UNPACK_VECTOR_ARRAY(gyroTempCalibrator->config.cx), gyroTempCalibrator->config.cx[3]);
    m_Logger.info("y = %f + (%fx) + (%fxx) + (%fxxx)", UNPACK_VECTOR_ARRAY(gyroTempCalibrator->config.cy), gyroTempCalibrator->config.cy[3]);
    m_Logger.info("y = %f + (%fx) + (%fxx) + (%fxxx)", UNPACK_VECTOR_ARRAY(gyroTempCalibrator->config.cz), gyroTempCalibrator->config.cz[3]);
}
void LSM6DSRSensor::saveTemperatureCalibration() {
    gyroTempCalibrator->saveConfig();
}

bool LSM6DSRSensor::getTemperature(float* out) {
    // Middle value is 23 degrees C (0x0000)
    #define LSM6DSR_ZERO_TEMP_OFFSET 23
    // Temperature per step from -41 + 1/2^9 degrees C (0x8001) to 87 - 1/2^9 degrees C (0x7FFF)
    constexpr float TEMP_STEP = 128. / 65535;
    int16_t temp;
    if (imu.getTemperature(&temp)) {
        *out = (temp * TEMP_STEP) + LSM6DSR_ZERO_TEMP_OFFSET;
        return true;
    }
    return false;
}

void LSM6DSRSensor::applyAccelCalibrationAndScale(sensor_real_t Axyz[3]) {
    //apply offsets (bias) and scale factors from Magneto
    if (isAccelCalibrated) {
        #if useFullCalibrationMatrix == true
            float tmp[3];
            for (uint8_t i = 0; i < 3; i++)
                tmp[i] = (Axyz[i] - m_Calibration.A_B[i]);
            Axyz[0] = m_Calibration.A_Ainv[0][0] * tmp[0] + m_Calibration.A_Ainv[0][1] * tmp[1] + m_Calibration.A_Ainv[0][2] * tmp[2];
            Axyz[1] = m_Calibration.A_Ainv[1][0] * tmp[0] + m_Calibration.A_Ainv[1][1] * tmp[1] + m_Calibration.A_Ainv[1][2] * tmp[2];
            Axyz[2] = m_Calibration.A_Ainv[2][0] * tmp[0] + m_Calibration.A_Ainv[2][1] * tmp[1] + m_Calibration.A_Ainv[2][2] * tmp[2];
        #else
            for (uint8_t i = 0; i < 3; i++)
                Axyz[i] = (Axyz[i] - m_Calibration.A_B[i]);
        #endif
    }
    Axyz[0] *= LSM6DSR_ASCALE;
    Axyz[1] *= LSM6DSR_ASCALE;
    Axyz[2] *= LSM6DSR_ASCALE;
}

void LSM6DSRSensor::applyMagCalibrationAndScale(sensor_real_t Mxyz[3]) {
    #if !USE_6_AXIS
        //apply offsets and scale factors from Magneto
        #if useFullCalibrationMatrix == true
            float temp[3];
            for (uint8_t i = 0; i < 3; i++)
                temp[i] = (Mxyz[i] - m_Calibration.M_B[i]);
            Mxyz[0] = m_Calibration.M_Ainv[0][0] * temp[0] + m_Calibration.M_Ainv[0][1] * temp[1] + m_Calibration.M_Ainv[0][2] * temp[2];
            Mxyz[1] = m_Calibration.M_Ainv[1][0] * temp[0] + m_Calibration.M_Ainv[1][1] * temp[1] + m_Calibration.M_Ainv[1][2] * temp[2];
            Mxyz[2] = m_Calibration.M_Ainv[2][0] * temp[0] + m_Calibration.M_Ainv[2][1] * temp[1] + m_Calibration.M_Ainv[2][2] * temp[2];
        #else
            for (uint8_t i = 0; i < 3; i++)
                Mxyz[i] = (Mxyz[i] - m_Calibration.M_B[i]);
        #endif
    #endif
}

bool LSM6DSRSensor::hasGyroCalibration() {
    for (int i = 0; i < 3; i++) {
        if (m_Calibration.G_off[i] != 0.0)
            return true;
    }
    return false;
}

bool LSM6DSRSensor::hasAccelCalibration() {
    for (int i = 0; i < 3; i++) {
        if (m_Calibration.A_B[i] != 0.0 ||
            m_Calibration.A_Ainv[0][i] != 0.0 ||
            m_Calibration.A_Ainv[1][i] != 0.0 ||
            m_Calibration.A_Ainv[2][i] != 0.0)
            return true;
    }
    return false;
}

bool LSM6DSRSensor::hasMagCalibration() {
    for (int i = 0; i < 3; i++) {
        if (m_Calibration.M_B[i] != 0.0 ||
            m_Calibration.M_Ainv[0][i] != 0.0 ||
            m_Calibration.M_Ainv[1][i] != 0.0 ||
            m_Calibration.M_Ainv[2][i] != 0.0)
            return true;
    }
    return false;
}

void LSM6DSRSensor::startCalibration(int calibrationType) {
    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::BMI160;
    ledManager.on();
    #if(USE_6_AXIS)
    maybeCalibrateGyro();
    calibration.data.bmi160 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();
    maybeCalibrateAccel();
    #else
    if(!hasGyroCalibration()){
        maybeCalibrateGyro();
        maybeCalibrateAccel();
    }
    maybeCalibrateMag();
    #endif
    m_Logger.debug("Saving the calibration data");

    calibration.data.bmi160 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();
    
    if(!hasMagCalibration()){
        initMMC();
        delay(10);
        int16_t mx,my,mz;
        imu.getMagnetometer(&mx,&my,&mz);
        int16_t px=mx,py=my,pz=mz;
        while(mx==px && my==py && mz==pz){
            imu.getMagnetometer(&mx,&my,&mz);
        }
    };

    m_Logger.debug("Saved the calibration data");

    m_Logger.info("Calibration data gathered, exiting calibration mode in...");
    constexpr uint8_t POST_CALIBRATION_DELAY_SEC = 0;
    ledManager.on();
    for (uint8_t i = POST_CALIBRATION_DELAY_SEC; i > 0; i--) {
        m_Logger.info("%i...", i);
        delay(1000);
    }
}

void LSM6DSRSensor::maybeCalibrateGyro() {
    #ifndef BMI160_CALIBRATION_GYRO_SECONDS
        static_assert(false, "BMI160_CALIBRATION_GYRO_SECONDS not set in defines");
    #endif

    #if BMI160_CALIBRATION_GYRO_SECONDS == 0
        m_Logger.debug("Skipping gyro calibration");
        return;
    #endif

    // Wait for sensor to calm down before calibration
    constexpr uint8_t GYRO_CALIBRATION_DELAY_SEC = 0;
    constexpr float GYRO_CALIBRATION_DURATION_SEC = BMI160_CALIBRATION_GYRO_SECONDS;
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration (%.1f seconds)", GYRO_CALIBRATION_DURATION_SEC);
    ledManager.on();
    for (uint8_t i = GYRO_CALIBRATION_DELAY_SEC; i > 0; i--) {
        m_Logger.info("%i...", i);
        delay(1000);
    }
    ledManager.off();

    if (!getTemperature(&temperature)) {
        m_Logger.error("Error: can't read temperature");
    }
    m_Calibration.temperature = temperature;

    #ifdef DEBUG_SENSOR
        m_Logger.trace("Calibration temperature: %f", temperature);
    #endif

    if (!imu.getGyroDrdy()) {
        m_Logger.error("Fatal error: gyroscope drdy = 0 (dead?)");
        return;
    }

    ledManager.pattern(100, 100, 3);
    ledManager.on();
    m_Logger.info("Gyro calibration started...");

    constexpr uint16_t gyroCalibrationSamples =
        GYRO_CALIBRATION_DURATION_SEC / (LSM6DSR_ODR_GYR_MICROS / 1e6);
    int32_t rawGxyz[3] = {0};
    for (int i = 0; i < gyroCalibrationSamples; i++) {
        imu.waitForGyroDrdy();

        int16_t gx, gy, gz;
        imu.getRotation(&gx, &gy, &gz);
        rawGxyz[0] += gx;
        rawGxyz[1] += gy;
        rawGxyz[2] += gz;
    }
    ledManager.off();
    m_Calibration.G_off[0] = ((double)rawGxyz[0]) / gyroCalibrationSamples;
    m_Calibration.G_off[1] = ((double)rawGxyz[1]) / gyroCalibrationSamples;
    m_Calibration.G_off[2] = ((double)rawGxyz[2]) / gyroCalibrationSamples;

    #ifdef DEBUG_SENSOR
        m_Logger.trace("Gyro calibration results: %f %f %f", UNPACK_VECTOR_ARRAY(m_Calibration.G_off));
    #endif
}

void LSM6DSRSensor::maybeCalibrateAccel() {
    #ifndef BMI160_ACCEL_CALIBRATION_METHOD
        static_assert(false, "BMI160_ACCEL_CALIBRATION_METHOD not set in defines");
    #endif

    #if BMI160_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_SKIP
        m_Logger.debug("Skipping accelerometer calibration");
        return;
    #endif

    MagnetoCalibration* magneto = new MagnetoCalibration();

    // Blink calibrating led before user should rotate the sensor
    #if BMI160_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_ROTATION
        m_Logger.info("After 3 seconds, Gently rotate the device while it's gathering data");
    #elif BMI160_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_6POINT
        m_Logger.info("Put the device into 6 unique orientations (all sides), leave it still and do not hold/touch for 3 seconds each");
    #endif
    constexpr uint8_t ACCEL_CALIBRATION_DELAY_SEC = 0;
    ledManager.on();
    for (uint8_t i = ACCEL_CALIBRATION_DELAY_SEC; i > 0; i--) {
        m_Logger.info("%i...", i);
        delay(1000);
    }
    ledManager.off();

    #if BMI160_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_ROTATION
        uint16_t accelCalibrationSamples = 200;
        ledManager.pattern(100, 100, 6);
        delay(100);
        ledManager.on();
        m_Logger.debug("Gathering accelerometer data...");
        for (int i = 0; i < accelCalibrationSamples; i++)
        {
            int16_t ax, ay, az;
            imu.getAcceleration(&ax, &ay, &az);
            magneto->sample(ax, ay, az);

            delay(100);
        }
        ledManager.off();
        m_Logger.debug("Calculating accelerometer calibration data...");
    #elif BMI160_ACCEL_CALIBRATION_METHOD == ACCEL_CALIBRATION_METHOD_6POINT
        RestDetectionParams calibrationRestDetectionParams;
        calibrationRestDetectionParams.restMinTimeMicros = 0.5 * 1e6;
        calibrationRestDetectionParams.restThAcc = 0.25f;
        RestDetection calibrationRestDetection(
            calibrationRestDetectionParams,
            LSM6DSR_ODR_GYR_MICROS / 1e6f,
            LSM6DSR_ODR_ACC_MICROS / 1e6f
        );

        constexpr uint16_t expectedPositions = 6;
        constexpr uint16_t numSamplesPerPosition = 32;

        uint16_t numPositionsRecorded = 0;
        uint16_t numCurrentPositionSamples = 0;
        bool waitForMotion = true;

        float* accelCalibrationChunk = new float[numSamplesPerPosition * 3];
        ledManager.pattern(100, 100, 6);
        ledManager.on();
        m_Logger.info("Gathering accelerometer data...");
        m_Logger.info("Waiting for position %i, you can leave the device as is...", numPositionsRecorded + 1);
        while (true) {
            int16_t ax, ay, az;
            imu.getAcceleration(&ax, &ay, &az);
            sensor_real_t scaled[3];
            scaled[0] = ax * LSM6DSR_ASCALE;
            scaled[1] = ay * LSM6DSR_ASCALE;
            scaled[2] = az * LSM6DSR_ASCALE;

            calibrationRestDetection.updateAcc(LSM6DSR_ODR_ACC_MICROS, scaled);

            if (waitForMotion) {
                if (!calibrationRestDetection.getRestDetected()) {
                    waitForMotion = false;
                }
                delayMicroseconds(LSM6DSR_ODR_ACC_MICROS);
                continue;
            }

            if (calibrationRestDetection.getRestDetected()) {
                ledManager.on();
                const uint16_t i = numCurrentPositionSamples * 3;
                accelCalibrationChunk[i + 0] = ax;
                accelCalibrationChunk[i + 1] = ay;
                accelCalibrationChunk[i + 2] = az;
                numCurrentPositionSamples++;

                if (numCurrentPositionSamples >= numSamplesPerPosition) {
                    for (int i = 0; i < numSamplesPerPosition; i++) {
                        magneto->sample(
                            accelCalibrationChunk[i * 3 + 0],
                            accelCalibrationChunk[i * 3 + 1],
                            accelCalibrationChunk[i * 3 + 2]
                        );
                    }
                    numPositionsRecorded++;
                    numCurrentPositionSamples = 0;
                    if (numPositionsRecorded < expectedPositions) {
                        ledManager.pattern(50, 50, 2);
                        m_Logger.info("Recorded, waiting for position %i...", numPositionsRecorded + 1);
                        waitForMotion = true;
                    }
                }
            } else {
                numCurrentPositionSamples = 0;
            }

            if (numPositionsRecorded >= expectedPositions) break;

            delayMicroseconds(LSM6DSR_ODR_ACC_MICROS);
        }
        ledManager.off();
        m_Logger.debug("Calculating accelerometer calibration data...");
        delete[] accelCalibrationChunk;
    #endif

    float A_BAinv[4][3];
    magneto->current_calibration(A_BAinv);
    delete magneto;

    m_Logger.debug("Finished calculating accelerometer calibration");
    m_Logger.debug("Accelerometer calibration matrix:");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.A_B[i] = A_BAinv[0][i];
        m_Calibration.A_Ainv[0][i] = A_BAinv[1][i];
        m_Calibration.A_Ainv[1][i] = A_BAinv[2][i];
        m_Calibration.A_Ainv[2][i] = A_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", A_BAinv[0][i], A_BAinv[1][i], A_BAinv[2][i], A_BAinv[3][i]);
    }
    m_Logger.debug("}");
}

void LSM6DSRSensor::maybeCalibrateMag() {
    #ifndef BMI160_CALIBRATION_MAG_SECONDS
        static_assert(false, "BMI160_CALIBRATION_MAG_SECONDS not set in defines");
    #endif

    #if BMI160_CALIBRATION_MAG_SECONDS == 0
        m_Logger.debug("Skipping magnetometer calibration");
        return;
    #endif

    ledManager.off();
    constexpr uint8_t CaliSamples=240;
    constexpr int MagTolerance=40;
    int16_t Cx[CaliSamples]={},Cy[CaliSamples]={},Cz[CaliSamples]={};
    uint8_t Cf=0, Cr=CaliSamples-1;
    int8_t ignoreList[CaliSamples]={};
    float cali[4][3];
    int16_t mx,my,mz;
    MagnetoCalibration* magneto = new MagnetoCalibration();
    imu.getMagnetometer(&mx,&my,&mz);
    Cx[0]=mx; Cy[0]=my;Cz[0]=mz;

    while(Cf!=Cr){
        imu.getMagnetometer(&mx,&my,&mz);
        ledManager.on();
        if (abs(Cx[Cf] - mx) > MagTolerance || abs(Cy[Cf] - my) > MagTolerance || abs(Cz[Cf] - mz) > MagTolerance)
        {
            ledManager.off();
            if(Cf==0 && Cr == CaliSamples-1) Serial.print("Starting Magneto Calibration");
            Cf++;
            if (Cf >= CaliSamples)
                Cf = 0;
            Cx[Cf] = mx;
            Cy[Cf] = my;
            Cz[Cf] = mz;
            if (Cf == Cr)
            {
                delete magneto;
                magneto = new MagnetoCalibration();
                for (uint8_t i = 0; i < CaliSamples; i++)
                {
                    if(!ignoreList[i])
                        magneto->sample(Cx[i],Cy[i],Cz[i]);
                }
                magneto->current_calibration(cali);

                //VerifyMagCali
                    uint8_t invalidCnt = 0;
                    float avgstr = 0.0f;
                    for (uint8_t i = 0; i < CaliSamples; i++)
                    {
                        float tx, ty, tz;
                        float x, y, z;
                        tx = Cx[i] - cali[0][0];
                        ty = Cy[i] - cali[0][1];
                        tz = Cz[i] - cali[0][2];
                #if useFullCalibrationMatrix == true
                        x = cali.M_Ainv[0][0] * tx + cali.M_Ainv[0][1] * ty + cali.M_Ainv[0][2] * tz;
                        y = cali.M_Ainv[1][0] * tx + cali.M_Ainv[1][1] * ty + cali.M_Ainv[1][2] * tz;
                        z = cali.M_Ainv[2][0] * tx + cali.M_Ainv[2][1] * ty + cali.M_Ainv[2][2] * tz;
                #else
                        x = tx;
                        y = ty;
                        z = tz;
                #endif
                        avgstr += sqrt(sq(x) + sq(y) + sq(z)) / CaliSamples;
                    }
                    if(isnan(avgstr)) continue;

                    m_Logger.debug("Average Mag strength with given calibration : %.1f\n", avgstr);
                    for (uint8_t i = 0; i < CaliSamples; i++)
                    {
                        float tx, ty, tz;
                        float x, y, z;
                        tx = Cx[i] - cali[0][0];
                        ty = Cy[i] - cali[0][1];
                        tz = Cz[i] - cali[0][2];
                #if useFullCalibrationMatrix == true
                        x = cali.M_Ainv[0][0] * tx + cali.M_Ainv[0][1] * ty + cali.M_Ainv[0][2] * tz;
                        y = cali.M_Ainv[1][0] * tx + cali.M_Ainv[1][1] * ty + cali.M_Ainv[1][2] * tz;
                        z = cali.M_Ainv[2][0] * tx + cali.M_Ainv[2][1] * ty + cali.M_Ainv[2][2] * tz;
                #else
                        x = tx;
                        y = ty;
                        z = tz;
                #endif
                        // Serial.printf(" %.1f \n", magstr[i]);
                        if (!(abs(avgstr - sqrt(sq(x) + sq(y) + sq(z))) < 20)){
                            invalidCnt++;
                            ignoreList[i]=1;
                        }
                        else ignoreList[i]=0;
                    }
                    if (invalidCnt < CaliSamples / 6)
                    {
                        Serial.print("New calibration valid\n");
                        delete magneto;
                        magneto = new MagnetoCalibration();
                        for (uint8_t i = 0; i < CaliSamples; i++)
                        {
                            if(!ignoreList[i])
                                magneto->sample(Cx[i],Cy[i],Cz[i]);
                        }
                        magneto->current_calibration(cali);
                        break;
                    }
                    else{
                        for(uint8_t i=0; i< CaliSamples; i++)
                            ignoreList[i]=0;
                        delete magneto;
                        magneto = new MagnetoCalibration();
                    }
                Cr += CaliSamples / 4;
                if (Cr >= CaliSamples)
                    Cr -= CaliSamples;
            }
        }
        delay(10);
    }
    delete magneto;

    m_Logger.debug("[INFO] Magnetometer calibration matrix:");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.M_B[i] = cali[0][i];
        m_Calibration.M_Ainv[0][i] = cali[1][i];
        m_Calibration.M_Ainv[1][i] = cali[2][i];
        m_Calibration.M_Ainv[2][i] = cali[3][i];
        m_Logger.debug("  %f, %f, %f, %f", cali[0][i], cali[1][i], cali[2][i], cali[3][i]);
    }
    m_Logger.debug("}");
}

void LSM6DSRSensor::remapGyroAccel(sensor_real_t* x, sensor_real_t* y, sensor_real_t* z) {
    remapAllAxis(AXIS_REMAP_GET_ALL_IMU(axisRemap), x, y, z);
}

void LSM6DSRSensor::remapMagnetometer(sensor_real_t* x, sensor_real_t* y, sensor_real_t* z) {
    remapAllAxis(AXIS_REMAP_GET_ALL_MAG(axisRemap), x, y, z);
}

void LSM6DSRSensor::getRemappedRotation(int16_t* x, int16_t* y, int16_t* z) {
    imu.getRotation(x, y, z);
    remapAllAxis(AXIS_REMAP_GET_ALL_IMU(axisRemap), x, y, z);
}
void LSM6DSRSensor::getRemappedAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    imu.getAcceleration(x, y, z);
    remapAllAxis(AXIS_REMAP_GET_ALL_IMU(axisRemap), x, y, z);
}