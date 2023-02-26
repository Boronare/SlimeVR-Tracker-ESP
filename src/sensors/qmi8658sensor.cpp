#include "qmi8658sensor.h"
#include "mahony.h"
#include "GlobalVars.h"
#include "magneto1.4.h"

#define ACCEL_SENSITIVITY_8G 4096.0f
constexpr float gscale = (512. / 32768.0) * (PI / 180.0); // gyro default 2048 LSB per d/s -> rad/s
constexpr float ASCALE_8G = ((32768. / ACCEL_SENSITIVITY_8G) / 32768.) * EARTH_GRAVITY;


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
    
    int16_t ax, ay, az;

    // If no compatible calibration data is found, the calibration data will just be zero-ed out
    switch (sensorCalibration.type)
    {
    case SlimeVR::Configuration::CalibrationConfigType::QMI8658:
        m_Calibration = sensorCalibration.data.qmi8658;
        Serial.printf("Loaded Cali : G: %f,%f,%f / M: %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n%f,%f,%f\n A: %f,%f,%f\n%f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
                m_Calibration.G_off[0],m_Calibration.G_off[1],m_Calibration.G_off[2],
                m_Calibration.M_B[0],m_Calibration.M_B[1],m_Calibration.M_B[2],
                m_Calibration.M_Ainv[0][0],m_Calibration.M_Ainv[0][1],m_Calibration.M_Ainv[0][2],
                m_Calibration.M_Ainv[1][0],m_Calibration.M_Ainv[1][1],m_Calibration.M_Ainv[1][2],
                m_Calibration.M_Ainv[2][0],m_Calibration.M_Ainv[2][1],m_Calibration.M_Ainv[2][2],
                m_Calibration.A_B[0],m_Calibration.A_B[1],m_Calibration.A_B[2],
                m_Calibration.A_Ainv[0][0],m_Calibration.A_Ainv[0][1],m_Calibration.A_Ainv[0][2],
                m_Calibration.A_Ainv[1][0],m_Calibration.A_Ainv[1][1],m_Calibration.A_Ainv[1][2],
                m_Calibration.A_Ainv[2][0],m_Calibration.A_Ainv[2][1],m_Calibration.A_Ainv[2][2]
        );
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
    imu.getAcceleration(&ax, &ay, &az);
    float g_az = (float)az / ACCEL_SENSITIVITY_8G;
    if(g_az < -0.75f) {
        ledManager.pattern(500,500,5);
        m_Logger.info("Flip front to confirm start calibration");
        imu.getAcceleration(&ax, &ay, &az);
        g_az = (float)az / ACCEL_SENSITIVITY_8G;
        if(g_az > 0.75f)
        {
            m_Logger.debug("Starting calibration...");
            startCalibration(0);
        }else{
            accelDupCnt = 255;
        }

        ledManager.off();
    }
    
    if(m_Calibration.M_B[0]==0.0 && m_Calibration.M_B[1]==0.0 && m_Calibration.M_B[2]==0.0){
        Cf=0;
        Cr=CaliSamples-1;
    }
        
    working = true;
}
void QMI8658Sensor::getValueScaled()
{
    float temp[3];
    uint8_t i;
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    AutoCalibrateGyro(gx,gy,gz,mx,my,mz);
    // Serial.printf("A : %+6d %+6d %+6d / G: %+6d %+6d %+6d / M:%+6d %+6d %+6d\n",ax,ay,az,gx,gy,gz,mx,my,mz);

    Gxyz[0] = (gx-m_Calibration.G_off[0]) * gscale;
    Gxyz[1] = (gy-m_Calibration.G_off[1]) * gscale;
    Gxyz[2] = (gz-m_Calibration.G_off[2]) * gscale;

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;
    //apply offsets (bias) and scale factors from Magneto
    #if useFullCalibrationMatrix == true
        for (uint8_t i = 0; i < 3; i++)
            temp[i] = (Axyz[i] - m_Calibration.A_B[i])/ACCEL_SENSITIVITY_8G;
        Axyz[0] = m_Calibration.A_Ainv[0][0] * temp[0] + m_Calibration.A_Ainv[0][1] * temp[1] + m_Calibration.A_Ainv[0][2] * temp[2];
        Axyz[1] = m_Calibration.A_Ainv[1][0] * temp[0] + m_Calibration.A_Ainv[1][1] * temp[1] + m_Calibration.A_Ainv[1][2] * temp[2];
        Axyz[2] = m_Calibration.A_Ainv[2][0] * temp[0] + m_Calibration.A_Ainv[2][1] * temp[1] + m_Calibration.A_Ainv[2][2] * temp[2];
    #else
        for (uint8_t i = 0; i < 3; i++)
            Axyz[i] = (Axyz[i] - m_Calibration.A_B[i])/ACCEL_SENSITIVITY_8G;
    #endif

    if((prevM[0] == mx && prevM[1] == my && prevM[2] == mz) || accelDupCnt == 255){
        Mxyz[0] = 0.0; Mxyz[1] = 0.0; Mxyz[2] = 0.0;
        return;
    }
    prevM[0] = mx; prevM[1] = my; prevM[2] = mz;
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
    // Serial.printf("Accel : %f\t%f\t%f, Raw : %d\t%d\t%d Bias : %f\t%f\t%f\n", Axyz[0], Axyz[1], Axyz[2], ax,ay,az, m_Calibration.A_B[0],m_Calibration.A_B[1],m_Calibration.A_B[2]);
}
void QMI8658Sensor::motionLoop()
{
    unsigned long now = micros();
    unsigned long deltat = now - last; // seconds since last update
    last = now;
    getValueScaled();
    // m_Logger.debug("A : %+6.0f %+6.0f %+6.0f / G : %+f %+f %+f / M: %+6.0f %+6.0f %+6.0f / Q : %+f %+f %+f %+f", Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0],Mxyz[1], Mxyz[2], q[0], q[1], q[2], q[3]);
    mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
    VectorFloat grav;
    grav.x = (q[1] * q[3] - q[0] * q[2]) * 2;
    grav.y = (q[0] * q[1] + q[2] * q[3]) * 2;
    grav.z = (sq(q[0]) - sq(q[1]) - sq(q[2]) + sq(q[3]));
    
    quaternion = Quat(q[1],q[2],q[3],q[0]);
    quaternion *= sensorOffset;
    // Serial.printf("Axyz:%+f %+f %+f, Grav:%+f %+f %+f\n",Axyz[0],Axyz[1],Axyz[2],grav.x,grav.y,grav.z);
#if SEND_ACCELERATION
    {
        // convert acceleration to m/s^2 (implicitly casts to float)
        acceleration[0] = (Axyz[0] - grav.x)*EARTH_GRAVITY;
        acceleration[1] = (Axyz[1] - grav.y)*EARTH_GRAVITY;
        acceleration[2] = (Axyz[2] - grav.z)*EARTH_GRAVITY;
    }
    if (!lastQuatSent.equalsWithEpsilon(quaternion))
    {
        newData = true;
        lastQuatSent = quaternion;
    }
#endif
}

float QMI8658Sensor::getTemperature()
{
    return imu.getTemperature() * 1.0;
}
void QMI8658Sensor::AutoCalibrateGyro(int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz){
    if (Cf != Cr){
        if(m_Calibration.M_B[0]==0.0 && m_Calibration.M_B[1] == 0.0 && m_Calibration.M_B[2] == 0.0)
            CalibrateMag(mx,my,mz);
        else
            CalibrateGyro(gx,gy,gz);
    }else{
        Cr += CaliSamples / 4;
        if (Cr > CaliSamples || Cf >= CaliSamples)
        {
            Cf -= CaliSamples;
            Cr -= CaliSamples;
        }
    }
}
void QMI8658Sensor::CalibrateGyro(int16_t gx, int16_t gy, int16_t gz)
{
    Cx[Cf] = gx;
    Cy[Cf] = gy;
    Cz[Cf] = gz;
    Cf++;
    if (Cf == Cr)
    {
        // m_Logger.debug("Gyro Samples Collected : X: %f Y:%f Z:%f",Gbias[0],Gbias[1],Gbias[2]);
        /**
         * Variance and Average
         */
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
            && vx < sq(GyroTolerance) * (CaliSamples - 1) && vy < sq(GyroTolerance) * (CaliSamples - 1) && vz < sq(GyroTolerance) * (CaliSamples - 1)){
                SlimeVR::Configuration::CalibrationConfig calibration;
                calibration.type = SlimeVR::Configuration::CalibrationConfigType::QMI8658;
                calibration.data.qmi8658 = m_Calibration;
                configuration.setCalibration(sensorId, calibration);
                configuration.save(sensorId);
                return;
            }
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
        Cr += CaliSamples / 4;
        if (Cr > CaliSamples)
        {
            Cf -= CaliSamples;
            Cr -= CaliSamples;
        }
    }
}

void QMI8658Sensor::CalibrateMag(int16_t mx, int16_t my, int16_t mz){
    ledManager.on();
    if (abs(Cx[Cf] - mx) > MagTolerance || abs(Cy[Cf] - my) > MagTolerance || abs(Cz[Cf] - mz) > MagTolerance*4)
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
            SlimeVR::Configuration::QMI8658CalibrationConfig n_Calibration = getMagAccCalibration();
            if (verifyMagAccCali(n_Calibration))
            {
                Serial.print("New calibration valid\n");
                n_Calibration = getMagAccCalibration();
                for (uint8_t i = 0; i < 3; i++)
                {
                    for (uint8_t j = 0; j < 3; j++)
                    {
                        m_Calibration.M_Ainv[i][j] = n_Calibration.M_Ainv[i][j];
                    }
                    m_Calibration.M_B[i] = n_Calibration.M_B[i];
                }
                SlimeVR::Configuration::CalibrationConfig calibration;
                calibration.type = SlimeVR::Configuration::CalibrationConfigType::QMI8658;
                calibration.data.qmi8658 = m_Calibration;
                configuration.setCalibration(sensorId, calibration);
                configuration.save(sensorId);
                return;
            }
            Cr += CaliSamples / 4;
            if (Cr >= CaliSamples)
                Cr -= CaliSamples;
        }
        delay(15);
    }
}

void QMI8658Sensor::CalibrateAcc(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz){
    if (abs(prevM[0] - ax) < AccTolerance && abs(prevM[1] - ay) < AccTolerance && abs(prevM[2] - az) < AccTolerance &&
    abs(gx-m_Calibration.G_off[0]) < GyroTolerance && abs(gy-m_Calibration.G_off[1]) < GyroTolerance && abs(gz-m_Calibration.G_off[2]) < GyroTolerance)
    {
        ledManager.off();
        if(accelDupCnt<=CaliSamples/12){
            accelDupCnt++;
            if(Cf==0 && Cr == CaliSamples-1) Serial.print("Starting Magneto Calibration");
            Cf++;
            if (Cf >= CaliSamples)
                Cf = 0;
            Cx[Cf] = ax;
            Cy[Cf] = ay;
            Cz[Cf] = az;
            if (Cf == Cr)
            {
                SlimeVR::Configuration::QMI8658CalibrationConfig n_Calibration = getMagAccCalibration();
                if (verifyMagAccCali(n_Calibration))
                {
                    Serial.print("New calibration valid\n");
                    n_Calibration = getMagAccCalibration();
                    for (uint8_t i = 0; i < 3; i++)
                    {
                        for (uint8_t j = 0; j < 3; j++)
                        {
                            m_Calibration.A_Ainv[i][j] = n_Calibration.M_Ainv[i][j];
                        }
                        m_Calibration.A_B[i] = n_Calibration.M_B[i];
                    }
                    SlimeVR::Configuration::CalibrationConfig calibration;
                    calibration.type = SlimeVR::Configuration::CalibrationConfigType::QMI8658;
                    calibration.data.qmi8658 = m_Calibration;
                    configuration.setCalibration(sensorId, calibration);
                    configuration.save(sensorId);
                    return;
                }
                Cr += CaliSamples / 4;
                if (Cr >= CaliSamples)
                    Cr -= CaliSamples;
            }
            delay(10);
        }
        else{
            ledManager.on();
        }
    }
    else if (abs(gx-m_Calibration.G_off[0]) > GyroTolerance*4 || abs(gy-m_Calibration.G_off[1]) > GyroTolerance*4 || abs(gz-m_Calibration.G_off[2]) > GyroTolerance*4){
        accelDupCnt = 0;
    }
    prevM[0] = ax;
    prevM[1] = ay;
    prevM[2] = az;
}

SlimeVR::Configuration::QMI8658CalibrationConfig QMI8658Sensor::getMagAccCalibration()
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
bool QMI8658Sensor::verifyMagAccCali(SlimeVR::Configuration::QMI8658CalibrationConfig cali)
{
    return true;
    // Verify if previous calibration data valid.
    uint8_t invalidCnt = 0;
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
        avgstr += sqrt(sq(x) + sq(y) + sq(z)) / CaliSamples;
    }
    if(isnan(avgstr)) return false;

    // Serial.printf("Average Mag strength with given calibration : %.1f\nMag Strengths:\n", avgstr);
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
        // Serial.printf(" %.1f \n", magstr[i]);
        if (!(abs(avgstr - sqrt(sq(x) + sq(y) + sq(z))) < MagTolerance*1.5)){
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

void QMI8658Sensor::startCalibration(int calibrationType) {
    acceleration[0]=acceleration[1]=acceleration[2]=0.0;
    while(Cf!=Cr){
        int16_t gx,gy,gz;
        imu.getGyro(&gx,&gy,&gz);
        CalibrateGyro(gx,gy,gz);
        delay(20);
    }
    m_Logger.debug("Gathering accelerometer data...");
    Cf=0; Cr=CaliSamples-1;
    while(Cf!=Cr){
        int16_t ax,ay,az,gx,gy,gz;
        imu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
        CalibrateAcc(ax,ay,az,gx,gy,gz);
        delay(10);
    }
    Cf=0; Cr=CaliSamples-1;
    while(Cf!=Cr){
        int16_t mx,my,mz;
        imu.getMagneto(&mx,&my,&mz);
        // Serial.printf("Mag Sample : %+6d %+6d %+6d\n",mx,my,mz);
        CalibrateMag(mx,my,mz);
        delay(20);
    }

    m_Logger.debug("Saved the calibration data");

    m_Logger.info("Calibration data gathered");
}
