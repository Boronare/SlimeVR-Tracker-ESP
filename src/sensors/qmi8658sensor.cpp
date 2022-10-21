#include "qmi8658sensor.h"
#include "mahony.h"
#include "GlobalVars.h"

constexpr float gscale = (32. / 32768.0) * (PI / 180.0); // gyro default 250 LSB per d/s -> rad/s
void QMI8658Sensor::motionSetup()
{
    // initialize device
    imu.initialize(addr);
    if (!imu.testConnection())
    {
        Serial.print("[ERR] Can't communicate with QMI8658, response 0x");
        Serial.println(imu.getDeviceID(), HEX);
        ledManager.blink(30);
        return;
    }

    Serial.print("[OK] Connected to QMI8658, ID 0x");
    Serial.println(imu.getDeviceID(), HEX);

    int16_t ax, ay, az;

    working = true;
}
void QMI8658Sensor::getValueScaled()
{
    float temp[3];
    int i;
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = (float)gx * gscale;
    Gxyz[1] = (float)gy * gscale;
    Gxyz[2] = (float)gz * gscale;

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
        temp[i] = (Mxyz[i] - calibration->M_B[i]);
    Mxyz[0] = calibration->M_Ainv[0][0] * temp[0] + calibration->M_Ainv[0][1] * temp[1] + calibration->M_Ainv[0][2] * temp[2];
    Mxyz[1] = calibration->M_Ainv[1][0] * temp[0] + calibration->M_Ainv[1][1] * temp[1] + calibration->M_Ainv[1][2] * temp[2];
    Mxyz[2] = calibration->M_Ainv[2][0] * temp[0] + calibration->M_Ainv[2][1] * temp[1] + calibration->M_Ainv[2][2] * temp[2];
// #else
//     for (i = 0; i < 3; i++)
//         Mxyz[i] = (Mxyz[i] - calibration->M_B[i]);
#endif
}
void QMI8658Sensor::motionLoop()
{
    // int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    // int16_t dqw, dqx, dqy, dqz;
    // imu.getQuatDiff(&dqw, &dqx, &dqy, &dqz);
    // imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    unsigned long now = micros();
    unsigned long deltat = now - last; // seconds since last update
    last = now;
    getValueScaled();
    Serial.printf("A : %+6.0f %+6.0f %+6.0f / G : %+f %+f %+f / M: %+6.0f %+6.0f %+6.0f / Q : %+f %+f %+f %+f\n", Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], q[0], q[1], q[2], q[3]);
    mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
    // imu.getMagneto(&mx,&my,&mz);
}

float QMI8658Sensor::getTemperature()
{
    return imu.getTemperature() * 1.0;
}