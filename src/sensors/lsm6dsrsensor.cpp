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

#define REMAPMAG(x,y,z,t) t=x;x=-y;y=t;

constexpr uint8_t CaliSamples=240;
constexpr int MagTolerance=40;

void LSM6DSRSensor::initMMC(){    /* Configure MAG interface and setup mode */


    /* Configure MMC5603NJ Sensor */

    //Reboot Magnetometer
    imu.setMagRegister(0x1C, 0x80);
    delay(100);
    //Set BW=00(6.6ms measurment time)
    imu.setMagRegister(0x1C, 0x00);
    //Set ODR
    imu.setMagRegister(0x1A, 13);
    //Enable Continuous mode & Auto SR
    imu.setMagRegister(0x1B, 0xA0);
    //Start Continuous mode
    imu.setMagRegister(0x1D, 0x10);
    imu.viaSensorhub();

    imu.setMagDevice(0x30,0x00);
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

    if (!imu.testConnection()) {
        m_Logger.fatal("Can't connect to LSM6DSR (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);
        ledManager.pattern(50, 50, 50);
        return;
    }

    m_Logger.info("Connected to LSM6DSR (reported device ID 0x%02x) at address 0x%02x", imu.getDeviceID(), addr);

    actualSensorMicros = 1000000/imu.getFinedGyroODR();
    // Initialize the configuration
    {
        SlimeVR::Configuration::SensorConfig sensorConfig = configuration.getSensor(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out

        switch (sensorConfig.type) {
        case SlimeVR::Configuration::SensorConfigType::LSM6DSR:
            m_Config = sensorConfig.data.lsm6dsr;
            magStatus = m_Config.flags&1 ? MagnetometerStatus::MAG_ENABLED
            : MagnetometerStatus::MAG_DISABLED;
            break;
        case SlimeVR::Configuration::SensorConfigType::BMI160:
            for(uint8_t i=0;i<3;i++){
                m_Config.G_off[i]=sensorConfig.data.bmi160.G_off[i];
                m_Config.A_B[i]=sensorConfig.data.bmi160.A_B[i];
                m_Config.M_B[i]=sensorConfig.data.bmi160.M_B[i];
                for(uint8_t j=0;j<3;j++){
                    m_Config.A_Ainv[i][j]=sensorConfig.data.bmi160.A_Ainv[i][j];
                    m_Config.M_Ainv[i][j]=sensorConfig.data.bmi160.M_Ainv[i][j];
                }
            }
            //save the config
            sensorConfig.type = SlimeVR::Configuration::SensorConfigType::LSM6DSR;
            sensorConfig.data.lsm6dsr = m_Config;
            configuration.setSensor(sensorId, sensorConfig);
            magStatus = MagnetometerStatus::MAG_DISABLED;
            break;
        case SlimeVR::Configuration::SensorConfigType::NONE:
            m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
            magStatus = MagnetometerStatus::MAG_DISABLED;
            break;

        default:
            m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
        }
    }

    int16_t ax, ay, az;
    imu.getAcceleration(&ax, &ay, &az);
    float g_az = (float)az / LSM6DSR_ACCEL_TYPICAL_SENSITIVITY_LSB;

    if (g_az < -0.75f) {
        m_Logger.info("Flip front to confirm start calibration");
        ledManager.off();
        delay(500);

        ledManager.pattern(500,500,3);
        imu.getAcceleration(&ax, &ay, &az);
        g_az = (float)az / LSM6DSR_ACCEL_TYPICAL_SENSITIVITY_LSB;
        if (g_az > 0.75f) {
            m_Logger.debug("Starting calibration...");
            startCalibration(0);
        }

        ledManager.off();
    }
    else{
        if(!hasGyroCalibration()){
            ledManager.pattern(150,150,5);
            startCalibration(0);
            ledManager.off();
        }
        if(magStatus == MagnetometerStatus::MAG_ENABLED){
            if(!hasMagCalibration()) startCalibration(0);
            else initMMC();
        }else{
            //if mag not enabled, turn off the magnetometer
            imu.setMagRegister(0x1D, 0x00);
            imu.viaSensorhub();
        }
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

    isGyroCalibrated = hasGyroCalibration();
    isAccelCalibrated = hasAccelCalibration();
    isMagCalibrated = hasMagCalibration();
    m_Logger.info("Calibration data for gyro: %s", isGyroCalibrated ? "found" : "not found");
    m_Logger.info("Calibration data for accel: %s", isAccelCalibrated ? "found" : "not found");
    m_Logger.info("Calibration data for mag: %s", isMagCalibrated ? "found" : "not found");

    imu.resetFIFO();
    delay(2);
    m_status = SensorStatus::SENSOR_OK;
    working = true;
}

void LSM6DSRSensor::motionLoop() {
    #if ENABLE_INSPECTION
    {
        int16_t rX, rY, rZ, aX, aY, aZ;

        networkConnection.sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, 0, 0, 0, 255);
    }
    #endif
            getTemperature(&temperature);

    {
        uint32_t now = micros();
        constexpr uint32_t LSM6DSR_TARGET_POLL_INTERVAL_MICROS = 16000;
        uint32_t elapsed = now - lastPollTime;
        if (elapsed >= LSM6DSR_TARGET_POLL_INTERVAL_MICROS) {
            lastPollTime = now - (elapsed - LSM6DSR_TARGET_POLL_INTERVAL_MICROS);

            readFIFO();
            optimistic_yield(100);
            if (!sfusion.isUpdated()) return;
            hadData = true;
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
                networkConnection.sendTemperature(sensorId, temperature);
            optimistic_yield(100);
        }
    }

    {
        uint32_t now = micros();
        constexpr float maxSendRateHz = 120.0f;
        constexpr uint32_t sendInterval = 1.0f/maxSendRateHz * 1e6;
        uint32_t elapsed = now - lastRotationPacketSent;
        if (elapsed >= sendInterval) {
            lastRotationPacketSent = now - (elapsed - sendInterval);

            setFusedRotation(sfusion.getQuaternionQuat());
            setAcceleration(sfusion.getLinearAccVec());
            if(CaliDebug){
                setFusedRotation(Quat(Vector3(1,0,0),PI/2)*Quat(Vector3(Gxyz[0],Gxyz[1],Gxyz[2])));
                if(magStatus == MagnetometerStatus::MAG_ENABLED)
                    setAcceleration(Vector3(Mxyz[0],Mxyz[1],Mxyz[2]));
                newFusedRotation = true;
            }
            optimistic_yield(100);
        }
        if(calibrationDetector.update(sfusion)){
            markRestCalibrationComplete();
        }
    }
}

void LSM6DSRSensor::readFIFO() {
    if (!imu.getFIFOCount(&fifo.length)) {
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
            if(magStatus == MagnetometerStatus::MAG_ENABLED)
                    initMMC();
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
        onGyroRawSample(actualSensorMicros*samples,(float)sgx/samples,(float)sgy/samples,(float)sgz/samples);
    }
}

void LSM6DSRSensor::onGyroRawSample(uint32_t dtMicros, float x, float y, float z) {

    sensor_real_t gyroCalibratedStatic[3];
    gyroCalibratedStatic[0] = (sensor_real_t)((x - m_Config.G_off[0]) * gscaleX);
    gyroCalibratedStatic[1] = (sensor_real_t)((y - m_Config.G_off[1]) * gscaleY);
    gyroCalibratedStatic[2] = (sensor_real_t)((z - m_Config.G_off[2]) * gscaleZ);

    {
        Gxyz[0] = gyroCalibratedStatic[0];
        Gxyz[1] = gyroCalibratedStatic[1];
        Gxyz[2] = gyroCalibratedStatic[2];
    }

    sfusion.updateGyro(Gxyz, (double)dtMicros * 1.0e-6);

    optimistic_yield(100);
}
void LSM6DSRSensor::onAccelRawSample(uint32_t dtMicros, int16_t x, int16_t y, int16_t z) {
    Axyz[0] = (sensor_real_t)x;
    Axyz[1] = (sensor_real_t)y;
    Axyz[2] = (sensor_real_t)z;
    applyAccelCalibrationAndScale(Axyz);
    sfusion.updateAcc(Axyz, dtMicros);

    optimistic_yield(100);
}
void LSM6DSRSensor::onMagRawSample(uint32_t dtMicros, int16_t x, int16_t y, int16_t z) {

    if(magCalibrating){
        Serial.print('.');
        ledManager.on();
        if (abs(Cx[Cf] - x) > MagTolerance || abs(Cy[Cf] - y) > MagTolerance || abs(Cz[Cf] - z) > MagTolerance)
        {
            ledManager.off();
            if(Cf==0 && Cr == CaliSamples-1) Serial.println("Starting Magneto Calibration");
            Cf++;
            if (Cf >= CaliSamples)
                Cf = 0;
            Cx[Cf] = x;
            Cy[Cf] = y;
            Cz[Cf] = z;
            Serial.printf("Mag Sample:%d %d %d\n",x,y,z);
            if (Cf == Cr)
            {
                MagnetoCalibration* magneto = new MagnetoCalibration();
                float cali[4][3];
                for (uint8_t i = 0; i < CaliSamples; i++)
                {
                    if(!ignoreList[i])
                        magneto->sample(Cx[i],Cy[i],Cz[i]);
                }
                magneto->current_calibration(cali);
                delete magneto;
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
                        x = cali[1][0] * tx + cali[1][1] * ty + cali[1][2] * tz;
                        y = cali[2][0] * tx + cali[2][1] * ty + cali[2][2] * tz;
                        z = cali[3][0] * tx + cali[3][1] * ty + cali[3][2] * tz;
                #else
                        x = tx;
                        y = ty;
                        z = tz;
                #endif
                        avgstr += sqrt(sq(x) + sq(y) + sq(z)) / CaliSamples;
                    }
                    if(isnan(avgstr)) return;

                    m_Logger.debug("Average Mag strength with given calibration : %.1f\n", avgstr);
                    for (uint8_t i = 0; i < CaliSamples; i++)
                    {
                        float tx, ty, tz;
                        float x, y, z;
                        tx = Cx[i] - cali[0][0];
                        ty = Cy[i] - cali[0][1];
                        tz = Cz[i] - cali[0][2];
                #if useFullCalibrationMatrix == true
                        x = cali[1][0] * tx + cali[1][1] * ty + cali[1][2] * tz;
                        y = cali[2][0] * tx + cali[2][1] * ty + cali[2][2] * tz;
                        z = cali[3][0] * tx + cali[3][1] * ty + cali[3][2] * tz;
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
                        magneto = new MagnetoCalibration();
                        for (uint8_t i = 0; i < CaliSamples; i++)
                        {
                            if(!ignoreList[i])
                                magneto->sample(Cx[i],Cy[i],Cz[i]);
                        }
                        magneto->current_calibration(cali);
                        delete magneto;

                        magCalibrating=false;
                        delete Cx;
                        delete Cy;
                        delete Cz;
                        delete ignoreList;

                        m_Logger.debug("[INFO] Magnetometer calibration matrix:");
                        m_Logger.debug("{");
                        for (int i = 0; i < 3; i++) {
                            m_Config.M_B[i] = cali[0][i];
                            m_Config.M_Ainv[0][i] = cali[1][i];
                            m_Config.M_Ainv[1][i] = cali[2][i];
                            m_Config.M_Ainv[2][i] = cali[3][i];
                            m_Logger.debug("  %f, %f, %f, %f", cali[0][i], cali[1][i], cali[2][i], cali[3][i]);
                        }
                        m_Logger.debug("}");

                        SlimeVR::Configuration::SensorConfig calibration;
                        calibration.type = SlimeVR::Configuration::SensorConfigType::LSM6DSR;
                        calibration.data.lsm6dsr = m_Config;
                        configuration.setSensor(sensorId, calibration);
                        configuration.save();
                    }
                    else{
                        for(uint8_t i=0; i< CaliSamples; i++)
                            ignoreList[i]=0;
                    }
                Cr += CaliSamples / 4;
                if (Cr >= CaliSamples)
                    Cr -= CaliSamples;
                imu.resetFIFO();
            }
        }
    }
    else{
        Mxyz[0] = (sensor_real_t)x;
        Mxyz[1] = (sensor_real_t)y;
        Mxyz[2] = (sensor_real_t)z;
        applyMagCalibrationAndScale(Mxyz);
        // remapMagnetometer(&Mxyz[0], &Mxyz[1], &Mxyz[2]);
        sensor_real_t temp;
        REMAPMAG(Mxyz[0],Mxyz[1],Mxyz[2],temp);
        sfusion.updateMag(Mxyz);
    }
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
                tmp[i] = (Axyz[i] - m_Config.A_B[i]);
            Axyz[0] = m_Config.A_Ainv[0][0] * tmp[0] + m_Config.A_Ainv[0][1] * tmp[1] + m_Config.A_Ainv[0][2] * tmp[2];
            Axyz[1] = m_Config.A_Ainv[1][0] * tmp[0] + m_Config.A_Ainv[1][1] * tmp[1] + m_Config.A_Ainv[1][2] * tmp[2];
            Axyz[2] = m_Config.A_Ainv[2][0] * tmp[0] + m_Config.A_Ainv[2][1] * tmp[1] + m_Config.A_Ainv[2][2] * tmp[2];
        #else
            for (uint8_t i = 0; i < 3; i++)
                Axyz[i] = (Axyz[i] - m_Config.A_B[i]);
        #endif
    }
    Axyz[0] *= LSM6DSR_ASCALE;
    Axyz[1] *= LSM6DSR_ASCALE;
    Axyz[2] *= LSM6DSR_ASCALE;
}

void LSM6DSRSensor::applyMagCalibrationAndScale(sensor_real_t Mxyz[3]) {
        //apply offsets and scale factors from Magneto
        #if useFullCalibrationMatrix == true
            float temp[3];
            for (uint8_t i = 0; i < 3; i++)
                temp[i] = (Mxyz[i] - m_Config.M_B[i]);
            Mxyz[0] = m_Config.M_Ainv[0][0] * temp[0] + m_Config.M_Ainv[0][1] * temp[1] + m_Config.M_Ainv[0][2] * temp[2];
            Mxyz[1] = m_Config.M_Ainv[1][0] * temp[0] + m_Config.M_Ainv[1][1] * temp[1] + m_Config.M_Ainv[1][2] * temp[2];
            Mxyz[2] = m_Config.M_Ainv[2][0] * temp[0] + m_Config.M_Ainv[2][1] * temp[1] + m_Config.M_Ainv[2][2] * temp[2];
        #else
            for (uint8_t i = 0; i < 3; i++)
                Mxyz[i] = (Mxyz[i] - m_Config.M_B[i]);
        #endif
}

bool LSM6DSRSensor::hasGyroCalibration() {
    for (int i = 0; i < 3; i++) {
        if (m_Config.G_off[i] != 0.0)
            return true;
    }
    return false;
}

bool LSM6DSRSensor::hasAccelCalibration() {
    for (int i = 0; i < 3; i++) {
        if (m_Config.A_B[i] != 0.0 ||
            m_Config.A_Ainv[0][i] != 0.0 ||
            m_Config.A_Ainv[1][i] != 0.0 ||
            m_Config.A_Ainv[2][i] != 0.0)
            return true;
    }
    return false;
}

bool LSM6DSRSensor::hasMagCalibration() {
    for (int i = 0; i < 3; i++) {
        if (m_Config.M_B[i] != 0.0 ||
            m_Config.M_Ainv[0][i] != 0.0 ||
            m_Config.M_Ainv[1][i] != 0.0 ||
            m_Config.M_Ainv[2][i] != 0.0)
            return true;
    }
    return false;
}

void LSM6DSRSensor::startCalibration(int calibrationType) {
    CaliDebug = true;
    SlimeVR::Configuration::SensorConfig calibration;
    calibration.type = SlimeVR::Configuration::SensorConfigType::LSM6DSR;
    ledManager.on();
    if(magStatus != MagnetometerStatus::MAG_ENABLED){
        if(!hasMagCalibration()){
            while(imu.getMagRegister(0x39)!=0x10){
                m_Logger.error("Error: MMC5603NJ not found : %02x",imu.getMagRegister(0x01));
                ledManager.pattern(100,100,3);
            }
            Serial.printf("MMC5603NJ found : %02x\n",imu.getMagRegister(0x39));
        };
        maybeCalibrateGyro();
        calibration.data.lsm6dsr = m_Config;
        configuration.setSensor(sensorId, calibration);
        configuration.save();
        maybeCalibrateAccel();
    }
    else{
        if(!hasGyroCalibration()){
            maybeCalibrateGyro();
            maybeCalibrateAccel();
        }
        initMMC();
        maybeCalibrateMag();
    }
    m_Logger.debug("Saving the calibration data");

    calibration.data.lsm6dsr = m_Config;
    configuration.setSensor(sensorId, calibration);
    configuration.save();

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
    constexpr float GYRO_CALIBRATION_DURATION_SEC = BMI160_CALIBRATION_GYRO_SECONDS;
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration (%.1f seconds)", GYRO_CALIBRATION_DURATION_SEC);
    ledManager.off();

    if (!getTemperature(&temperature)) {
        m_Logger.error("Error: can't read temperature");
    }
    m_Config.temperature = temperature;

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
    m_Config.G_off[0] = ((double)rawGxyz[0]) / gyroCalibrationSamples;
    m_Config.G_off[1] = ((double)rawGxyz[1]) / gyroCalibrationSamples;
    m_Config.G_off[2] = ((double)rawGxyz[2]) / gyroCalibrationSamples;

    #ifdef DEBUG_SENSOR
        m_Logger.trace("Gyro calibration results: %f %f %f", UNPACK_VECTOR_ARRAY(m_Config.G_off));
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
        calibrationRestDetectionParams.restMinTime = 1.0f;
        calibrationRestDetectionParams.restThAcc = 0.25f;
        RestDetection calibrationRestDetection(
            calibrationRestDetectionParams,
            LSM6DSR_ODR_GYR_MICROS / 1e6f,
            LSM6DSR_ODR_ACC_MICROS / 1e6f
        );

        constexpr uint16_t expectedPositions = 6;
        constexpr uint16_t numSamplesPerPosition = 64;

        uint16_t numPositionsRecorded = 0;
        uint16_t numCurrentPositionSamples = 0;
        bool waitForMotion = false;

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
        m_Config.A_B[i] = A_BAinv[0][i];
        m_Config.A_Ainv[0][i] = A_BAinv[1][i];
        m_Config.A_Ainv[1][i] = A_BAinv[2][i];
        m_Config.A_Ainv[2][i] = A_BAinv[3][i];
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
    magCalibrating = true;

    Cx = new int16_t[CaliSamples],Cy = new int16_t[CaliSamples],Cz = new int16_t[CaliSamples];
    Cf=0; Cr=CaliSamples-1;
    ignoreList = new int8_t[CaliSamples];
    imu.getMagnetometer(Cx,Cy,Cz);
}