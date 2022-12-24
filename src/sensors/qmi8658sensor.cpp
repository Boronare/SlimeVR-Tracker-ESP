#include "qmi8658sensor.h"
#include "mahony.h"
#include "GlobalVars.h"
#include "magneto1.4.h"

constexpr float gscale = (512. / 32768.0) * (PI / 180.0); // gyro default 2048 LSB per d/s -> rad/s




void QMI8658Sensor::motionSetup()
{
    // initialize device
    imu.initialize(addr, 0x2C + sensorId);
    if (!imu.testConnection())
    {
        Serial.print("[ERR] Can't communicate with QMI8658, response 0x");
        Serial.println(imu.getDeviceID(), HEX);
        ledManager.blink(30);
        return;
    }

    Serial.print("[OK] Connected to QMI8658, ID 0x");
    Serial.println(imu.getDeviceID(), HEX);

    SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
    // If no compatible calibration data is found, the calibration data will just be zero-ed out
    switch (sensorCalibration.type)
    {
    case SlimeVR::Configuration::CalibrationConfigType::QMI8658:
        m_Calibration = sensorCalibration.data.qmi8658;
        Serial.printf("Loaded Cali : G: %f,%f,%f / M: %f,%f,%f\n",m_Calibration.G_off[0],m_Calibration.G_off[1],m_Calibration.G_off[2],m_Calibration.M_B[0],m_Calibration.M_B[1],m_Calibration.M_B[2]);
        break;
    default:
        m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
        m_Logger.info("Calibration is advised");
        for(uint8_t i=0;i<3;i++){
            for(uint8_t j=0;j<3;j++){
                m_Calibration.M_Ainv[i][j] = i==j?1:0;
            }
            m_Calibration.M_B[i]=m_Calibration.G_off[i]=0;
        }
    }
    delay(1000);
        
    working = true;
}
void QMI8658Sensor::getValueScaled()
{
    float temp[3];
    uint8_t i;
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    // Serial.printf("A : %+6d %+6d %+6d / G: %+6d %+6d %+6d / M:%+6d %+6d %+6d\n",ax,ay,az,gx,gy,gz,mx,my,mz);
    AutoCalibrate(gx, gy, gz, mx, my, mz);

    Gxyz[0] = (gx-m_Calibration.G_off[0]) * gscale;
    Gxyz[1] = (gy-m_Calibration.G_off[1]) * gscale;
    Gxyz[2] = (gz-m_Calibration.G_off[2]) * gscale;

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;

    // Orientations of axes are set in accordance with the datasheet
    // See Section 9.1 Orientation of Axes
    // https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
    Mxyz[0] = (float)mx;
    Mxyz[1] = (float)my;
    Mxyz[2] = (float)mz;
// apply offsets and scale factors from Magneto
#if useFullCalibrationMatrix == true
    for (i = 0; i < 3; i++)
        temp[i] = (Mxyz[i] - m_Calibration.M_B[i]);
    Mxyz[0] = m_Calibration.M_Ainv[0][0] * temp[0] + m_Calibration.M_Ainv[0][1] * temp[1] + m_Calibration.M_Ainv[0][2] * temp[2];
    Mxyz[1] = m_Calibration.M_Ainv[1][0] * temp[0] + m_Calibration.M_Ainv[1][1] * temp[1] + m_Calibration.M_Ainv[1][2] * temp[2];
    Mxyz[2] = m_Calibration.M_Ainv[2][0] * temp[0] + m_Calibration.M_Ainv[2][1] * temp[1] + m_Calibration.M_Ainv[2][2] * temp[2];
#else
    for (i = 0; i < 3; i++)
        Mxyz[i] = (Mxyz[i] - m_Calibration.M_B[i]);
#endif
}
void QMI8658Sensor::motionLoop()
{
    unsigned long now = micros();
    unsigned long deltat = now - last; // seconds since last update
    last = now;
    getValueScaled();
    // m_Logger.debug("A : %+6.0f %+6.0f %+6.0f / G : %+f %+f %+f / M: %+6.0f %+6.0f %+6.0f / Q : %+f %+f %+f %+f", Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0],Mxyz[1], Mxyz[2], q[0], q[1], q[2], q[3]);
    mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
    
    quaternion = Quat(q[1],q[2],q[3],q[0]);
    quaternion *= sensorOffset;
    if (!lastQuatSent.equalsWithEpsilon(quaternion))
    {
        newData = true;
        lastQuatSent = quaternion;
    }
}

float QMI8658Sensor::getTemperature()
{
    return imu.getTemperature() * 1.0;
}
void QMI8658Sensor::AutoCalibrate(int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz){
    if (Gf != Gr) AutoCalibrateGyro(gx,gy,gz);
    else if (Mf != Mr) AutoCalibrateMag(mx,my,mz);
    else{
        Gr += CaliSamples / 4;
        if (Gr > CaliSamples || Gf >= CaliSamples)
        {
            Gf -= CaliSamples;
            Gr -= CaliSamples;
        }
    }
}
void QMI8658Sensor::AutoCalibrateGyro(int16_t gx, int16_t gy, int16_t gz)
{
        Cx[Gf] = gx;
        Cy[Gf] = gy;
        Cz[Gf] = gz;
        Gf++;
        if (Gf == Gr)
        {
            // m_Logger.debug("Gyro Samples Collected : X: %f Y:%f Z:%f",Gbias[0],Gbias[1],Gbias[2]);
            float vx = 0.0;
            float vy = 0.0;
            float vz = 0.0;
            float ax = 0.0;
            float ay = 0.0;
            float az = 0.0;
            for (uint8_t i = 0; i < CaliSamples; i++)
            {
                vx += sq(Cx[i] - m_Calibration.G_off[0]);
                vy += sq(Cy[i] - m_Calibration.G_off[1]);
                vz += sq(Cz[i] - m_Calibration.G_off[2]);
                ax += Cx[i] - m_Calibration.G_off[0];
                ay += Cy[i] - m_Calibration.G_off[1];
                az += Cz[i] - m_Calibration.G_off[2];
            }
            if (abs(ax) < (0.002 * CaliSamples/gscale) && abs(ay) <  0.002 * CaliSamples/gscale && abs(az) < 0.002 * CaliSamples/gscale
                && vx < sq(GyroTolerance) * (CaliSamples - 1) && vy < sq(GyroTolerance) * (CaliSamples - 1) && vz < sq(GyroTolerance) * (CaliSamples - 1))
                return;
            // m_Logger.debug("Gyro Calibration Failed : %f/%f/%f/%f/%f/%f",ax,ay,az,vx,vy,vz);
            vx = 0.0;
            vy = 0.0;
            vz = 0.0;
            for (uint8_t i = 0; i < CaliSamples; i++)
            {
                vx += sq(Cx[i] - m_Calibration.G_off[0] - ax/CaliSamples);
                vy += sq(Cy[i] - m_Calibration.G_off[1] - ay/CaliSamples);
                vz += sq(Cz[i] - m_Calibration.G_off[2] - az/CaliSamples);
            }
            //Update bias if Gyro for each axis is stable.
            if(vx < sq(GyroTolerance) * (CaliSamples - 1)){
                m_Calibration.G_off[0] += ax/CaliSamples;
            }
            if(vy < sq(GyroTolerance) * (CaliSamples - 1)){
                m_Calibration.G_off[1] += ay/CaliSamples;
            }
            if(vz < sq(GyroTolerance) * (CaliSamples - 1)){
                m_Calibration.G_off[2] += az/CaliSamples;
            }
            // Prepare for next gyro calibration.
            Gr += CaliSamples / 4;
            if (Gr > CaliSamples)
            {
                Gf -= CaliSamples;
                Gr -= CaliSamples;
            }
        }
}

void QMI8658Sensor::AutoCalibrateMag(int16_t mx, int16_t my, int16_t mz){
        ledManager.on();
        if (abs(Cx[Mf] - mx) > MagTolerance*2 || abs(Cy[Mf] - my) > MagTolerance*2 || abs(Cz[Mf] - mz) > MagTolerance*4)
        {
            ledManager.off();
            if(Mf==0 && Mr == CaliSamples-1) Serial.print("Starting Magneto Calibration");
            Mf++;
            if (Mf >= CaliSamples)
                Mf = 0;
            Cx[Mf] = mx;
            Cy[Mf] = my;
            Cz[Mf] = mz;
            if (Mf == Mr)
            {
                if (verifyMagCali(m_Calibration))
                {
                    Serial.print("Mag strength valid.\n");
                    delay(300);
                    return;
                }
                else{
                    SlimeVR::Configuration::QMI8658CalibrationConfig n_Calibration = getMagCalibration();
                if (verifyMagCali(n_Calibration))
                {
                    Serial.print("New calibration valid\n");
                    n_Calibration = getMagCalibration();
                    for (uint8_t i = 0; i < 3; i++)
                    {
                        for (uint8_t j = 0; j < 3; j++)
                        {
                            m_Calibration.M_Ainv[i][j] = n_Calibration.M_Ainv[i][j];
                        }
                        m_Calibration.M_B[i] = n_Calibration.M_B[i];
                    }
                    // delete[] Cx;
                    // delete[] Cy;
                    // delete[] Cz;
                    // delete[] ignoreList;
                    SlimeVR::Configuration::CalibrationConfig calibration;
                    calibration.type = SlimeVR::Configuration::CalibrationConfigType::QMI8658;
                    calibration.data.qmi8658 = m_Calibration;
                    configuration.setCalibration(sensorId, calibration);
                    configuration.save(sensorId);
                    delay(300);
                    return;
                }
                Mr += CaliSamples / 4;
                if (Mr >= CaliSamples)
                    Mr -= CaliSamples;
                }
            }
            delay(30);
        }
}

SlimeVR::Configuration::QMI8658CalibrationConfig QMI8658Sensor::getMagCalibration()
{
    SlimeVR::Configuration::QMI8658CalibrationConfig retVal;
    MagnetoCalibration magneto{};
    for (uint8_t i = 0; i < CaliSamples; i++)
    {
        if(!ignoreList[i])
            magneto.sample(Cx[i],Cy[i],Cz[i]);
    }
    float M_BAinv[4][3];
    magneto.current_calibration(M_BAinv);
    // m_Logger.debug("[INFO] Magnetometer calibration matrix:");
    // m_Logger.debug("{");
    for (int i = 0; i < 3; i++)
    {
        retVal.M_B[i] = M_BAinv[0][i];
        retVal.M_Ainv[0][i] = M_BAinv[1][i];
        retVal.M_Ainv[1][i] = M_BAinv[2][i];
        retVal.M_Ainv[2][i] = M_BAinv[3][i];
        // m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    }
    // m_Logger.debug("}");
    return retVal;
}
bool QMI8658Sensor::verifyMagCali(SlimeVR::Configuration::QMI8658CalibrationConfig cali)
{
    // Verify if previous calibration data valid.
    uint8_t invalidCnt = 0;
    float magstr[CaliSamples];
    float avgstr = 0.0f;
    for (uint8_t i = 0; i < CaliSamples; i++)
    {
        float tx, ty, tz;
        float x, y, z;
        tx = Cx[i] - cali.M_B[0];
        ty = Cy[i] - cali.M_B[1];
        tz = Cz[i] - cali.M_B[2];
#if useFullCalibrationMatrix == true
        x = cali.M_Ainv[0][0] * tx + cali.M_Ainv[0][1] * ty + cali.M_Ainv[0][2] * tz;
        y = cali.M_Ainv[1][0] * tx + cali.M_Ainv[1][1] * ty + cali.M_Ainv[1][2] * tz;
        z = cali.M_Ainv[2][0] * tx + cali.M_Ainv[2][1] * ty + cali.M_Ainv[2][2] * tz;
#else
        x = tx;
        y = ty;
        z = tz;
#endif
        magstr[i] = sqrt(sq(x) + sq(y) + sq(z));
        avgstr += magstr[i] / CaliSamples;
    }
    if(isnan(avgstr)) return false;

    // Serial.printf("Average Mag strength with given calibration : %.1f\nMag Strengths:\n", avgstr);
    for (uint8_t i = 0; i < CaliSamples; i++)
    {
        // Serial.printf(" %.1f \n", magstr[i]);
        if (!(abs(avgstr - magstr[i]) < MagTolerance)){
            invalidCnt++;
            ignoreList[i]=1;
        }
        else ignoreList[i]=0;
    }
    if (invalidCnt < CaliSamples / 4)
    {
        MagStr = avgstr;
        return true;
    }
    else{
        for(uint8_t i=0; i< CaliSamples; i++)
            ignoreList[i]=0;
        return false;
    }
}