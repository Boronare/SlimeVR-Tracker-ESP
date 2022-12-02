#include "qmi8658sensor.h"
#include "mahony.h"
#include "GlobalVars.h"
#include "magneto1.4.h"

constexpr float gscale = (2048. / 32768.0) * (PI / 180.0); // gyro default 2048 LSB per d/s -> rad/s
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

    Cx = new int16_t[CaliSamples];
    Cy = new int16_t[CaliSamples];
    Cz = new int16_t[CaliSamples];
    for(uint8_t i=0;i<CaliSamples;i++){
        Cx[i]=Cy[i]=Cz[i]=0;
    }

    SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
    // If no compatible calibration data is found, the calibration data will just be zero-ed out
    switch (sensorCalibration.type)
    {
    case SlimeVR::Configuration::CalibrationConfigType::QMI8658:
        m_Calibration = sensorCalibration.data.qmi8658;
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
    working = true;
}
void QMI8658Sensor::getValueScaled()
{
    float temp[3];
    uint8_t i;
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    // m_Logger.debug("A : %+6d %+6d %+6d / G: %+6d %+6d %+6d / M:%+6d %+6d %+6d",ax,ay,az,gx,gy,gz,mx,my,mz);
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

    m_Logger.debug("A : %+6.0f %+6.0f %+6.0f / G : %+f %+f %+f / M: %+6.0f %+6.0f %+6.0f / Q : %+f %+f %+f %+f\n", Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], q[0], q[1], q[2], q[3]);

    // if (Gf == Gr && abs(MagStr - (sq(Mxyz[0]) + sq(Mxyz[1]) + sq(Mxyz[2]))) > sq(MagTolerance)) // 6DoF mahony if Magneto unstable
    //     mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat * 1.0e-6);
    // else
    // mahonyQuaternionUpdate(q,0, 0, 0, Gxyz[0], Gxyz[1], -Gxyz[2], deltat * 1.0e-6);
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

void QMI8658Sensor::AutoCalibrate(int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz)
{
    if (Gf != Gr)
    {
        Gbias[0] += (gx - Cx[Gf]) * 1.0f / CaliSamples;
        Cx[Gf] = gx;
        Gbias[1] += (gy - Cy[Gf]) * 1.0f / CaliSamples;
        Cy[Gf] = gy;
        Gbias[2] += (gz - Cz[Gf]) * 1.0f / CaliSamples;
        Cz[Gf] = gz;
        Gf++;
        if (Gf == Gr)
        {
            m_Logger.debug("Gyro Samples Collected : X: %f Y:%f Z:%f",Gbias[0],Gbias[1],Gbias[2]);
            float vx = 0.0;
            float vy = 0.0;
            float vz = 0.0;
            for (uint8_t i = 0; i < CaliSamples; i++)
            {
                vx += sq(Cx[i] - Gbias[0]);
                vy += sq(Cy[i] - Gbias[1]);
                vz += sq(Cz[i] - Gbias[2]);
            }
            if (vx > sq(GyroTolerance) * (CaliSamples - 1) || vy > sq(GyroTolerance) * (CaliSamples - 1) || vz > sq(GyroTolerance) * (CaliSamples - 1))
            { // Gyro Value is unstable.
                Gr += CaliSamples / 4;
                if (Gr > CaliSamples)
                {
                    Gf -= CaliSamples;
                    Gr -= CaliSamples;
                }
                m_Logger.info("Gyro Calibration Failed : %f/%f/%f",vx/(CaliSamples-1),vy/(CaliSamples-1),vz/(CaliSamples-1));
            }
            else
            { // Gyro Value Stable. Complete Calibrating Gyro and get ready for calibrate Magneto.
                m_Logger.info("Gyro Calibration Done.");
                m_Calibration.G_off[0] = Gbias[0];
                m_Calibration.G_off[1] = Gbias[1];
                m_Calibration.G_off[2] = Gbias[2];
                Cx[0] = mx;
                Cy[0] = my;
                Cz[0] = mz;
                for (uint8_t i = 1; i < CaliSamples; i++)
                {
                    Cx[i] = Cy[i] = Cz[i] = 0;
                }
            }
        }
    }
    else if (Mf != Mr)
    {
        ledManager.on();
        if (abs(Cx[Mf] - mx) > MagIgnoreSample || abs(Cy[Mf] - my) > MagIgnoreSample || abs(Cz[Mf] - mz) > MagIgnoreSample)
        {
            Mf++;
            if (Mf > 63)
                Mf = 0;
            Cx[Mf] = mx;
            Cy[Mf] = my;
            Cz[Mf] = mz;
            if (Mf == Mr)
            {
                if (verifyMagCali(m_Calibration))
                {
                    ledManager.off();
                    m_Logger.info("Mag strength valid.\n");
                    delete[] Cx;
                    delete[] Cy;
                    delete[] Cz;
                    return;
                }
                SlimeVR::Configuration::QMI8658CalibrationConfig n_Calibration = getMagCalibration();
                if (verifyMagCali(n_Calibration))
                {
                    m_Logger.info("New calibration valid\n");
                    for (uint8_t i = 0; i < 3; i++)
                    {
                        for (uint8_t j = 0; j < 3; j++)
                        {
                            m_Calibration.M_Ainv[i][j] = n_Calibration.M_Ainv[i][j];
                        }
                        m_Calibration.M_B[i] = n_Calibration.M_B[i];
                    }
                    delete[] Cx;
                    delete[] Cy;
                    delete[] Cz;
                    SlimeVR::Configuration::CalibrationConfig calibration;
                    calibration.type = SlimeVR::Configuration::CalibrationConfigType::QMI8658;
                    calibration.data.qmi8658 = m_Calibration;
                    configuration.setCalibration(sensorId, calibration);
                    configuration.save();
                    ledManager.off();
                    return;
                }
                Mr += CaliSamples / 4;
                if (Mr >= CaliSamples)
                    Mr -= CaliSamples;
            }
        }
    }
}

SlimeVR::Configuration::QMI8658CalibrationConfig QMI8658Sensor::getMagCalibration()
{
    SlimeVR::Configuration::QMI8658CalibrationConfig retVal;
    float *calibrationDataMag = new float[CaliSamples * 3];
    for (uint8_t i = 0; i < CaliSamples; i++)
    {
        calibrationDataMag[i * 3 + 0] = Cx[i];
        calibrationDataMag[i * 3 + 1] = Cy[i];
        calibrationDataMag[i * 3 + 2] = Cz[i];
    }
    float M_BAinv[4][3];
    CalculateCalibration(calibrationDataMag, CaliSamples, M_BAinv);
    delete[] calibrationDataMag;
    m_Logger.debug("[INFO] Magnetometer calibration matrix:");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++)
    {
        retVal.M_B[i] = M_BAinv[0][i];
        retVal.M_Ainv[0][i] = M_BAinv[1][i];
        retVal.M_Ainv[1][i] = M_BAinv[2][i];
        retVal.M_Ainv[2][i] = M_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    }
    m_Logger.debug("}");
    return retVal;
}
bool QMI8658Sensor::verifyMagCali(SlimeVR::Configuration::QMI8658CalibrationConfig cali)
{
    // Verify if previous calibration data valid.
    uint8_t invalidCnt = 0;
    float *magstr = new float[CaliSamples];
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
    float toler = avgstr*0.05;

    m_Logger.debug("Average Mag strength with given calibration : %.1f\nMag Strengths:", avgstr);
    for (uint8_t i = 0; i < CaliSamples; i++)
    {
        m_Logger.debug(" %.1f ", magstr[i]);
        if (abs(avgstr - magstr[i]) > toler)
            invalidCnt++;
    }
    delete[] magstr;
    m_Logger.debug("\n");
    if (invalidCnt < CaliSamples / 16)
    {
        MagStr = avgstr;
        return true;
    }
    else
        return false;
}