#include "qmi8658sensor.h"
#include "mahony.h"
#include "GlobalVars.h"
#include "magneto1.4.h"

#define ACCEL_SENSITIVITY_8G 4096.0f
constexpr float gscale = (1024. / 32768.0) * (PI / 180.0) ; // gyro default 2048 LSB per d/s -> rad/s
constexpr float ASCALE_8G = ((32768. / ACCEL_SENSITIVITY_8G) / 32768.) * CONST_EARTH_GRAVITY;


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
        m_Logger.info("Flip front to confirm start calibration");
        ledManager.pattern(500,500,5);
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
void QMI8658Sensor::motionLoop()
{
    uint8_t buffer[1536];
    int16_t fifoCnt;
    
    if(!imu.isAlive()){
        if(imu.testConnection()) imu.initialize(addr, 0x2C+sensorId);
    }
    else{
        // uint8_t status = imu.getStatus0();
        uint8_t i;
        int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
        fifoCnt = imu.getFIFOCount();
        imu.getFIFOBytes(buffer,fifoCnt);
        // Serial.printf("Status : %x Cnt : %d\n",fifoStatus, fifoCnt);
        for(i=0;i<fifoCnt/12;i++){
            ax = buffer[i*12+0]|buffer[i*12+1]<<8;
            ay = buffer[i*12+2]|buffer[i*12+3]<<8;
            az = buffer[i*12+4]|buffer[i*12+5]<<8;
            gx = buffer[i*12+6]|buffer[i*12+7]<<8;
            gy = buffer[i*12+8]|buffer[i*12+9]<<8;
            gz = buffer[i*12+10]|buffer[i*12+11]<<8;
            // Serial.printf("i:%d G:%+6d %+6d %+6d A:%+6d %+6d %+6d\n",i,gx,gy,gz,ax,ay,az);

                
        // AutoCalibrateGyro(gx,gy,gz,ax,ay,az);
            Axyz[0] = (float)ax;
            Axyz[1] = (float)ay;
            Axyz[2] = (float)az;
            #if useFullCalibrationMatrix == true
                float tmp[3];
                for (uint8_t i = 0; i < 3; i++)
                    tmp[i] = (Axyz[i] - m_Calibration.A_B[i])*CONST_EARTH_GRAVITY/ACCEL_SENSITIVITY_8G;
                Axyz[0] = m_Calibration.A_Ainv[0][0] * tmp[0] + m_Calibration.A_Ainv[0][1] * tmp[1] + m_Calibration.A_Ainv[0][2] * tmp[2];
                Axyz[1] = m_Calibration.A_Ainv[1][0] * tmp[0] + m_Calibration.A_Ainv[1][1] * tmp[1] + m_Calibration.A_Ainv[1][2] * tmp[2];
                Axyz[2] = m_Calibration.A_Ainv[2][0] * tmp[0] + m_Calibration.A_Ainv[2][1] * tmp[1] + m_Calibration.A_Ainv[2][2] * tmp[2];
            #else
                for (uint8_t i = 0; i < 3; i++)
                    Axyz[i] = (Axyz[i] - m_Calibration.A_B[i])/ACCEL_SENSITIVITY_8G;
            #endif
            sfusion.updateAcc(Axyz, QMI8658_ODR_MICROS*1e-6);
            optimistic_yield(100);

            Gxyz[0] = (sensor_real_t)((((double)gx - m_Calibration.G_off[0]) * gscale));
            Gxyz[1] = (sensor_real_t)((((double)gy - m_Calibration.G_off[1]) * gscale));
            Gxyz[2] = (sensor_real_t)((((double)gz - m_Calibration.G_off[2]) * gscale));

            // Serial.printf("Gxyz : %f %f %f\n",Gxyz[0],Gxyz[1],Gxyz[2]);
            sfusion.updateGyro(Gxyz, (double)QMI8658_ODR_MICROS * 1.0e-6);
            if(sfusion.getRestDetected()){
                m_Calibration.G_off[0]=gx;
                m_Calibration.G_off[1]=gy;
                m_Calibration.G_off[2]=gz;
            }
        }
        #if !USE_6_AXIS
            imu.getMagneto(&mx,&my,&mz);
            Mxyz[0] = (float)mx;
            Mxyz[1] = (float)my;
            Mxyz[2] = (float)mz;
            #if useFullCalibrationMatrix == true
                float temp[3];
                for (i = 0; i < 3; i++)
                    temp[i] = (Mxyz[i] - m_Calibration.M_B[i]);
                Mxyz[0] = m_Calibration.M_Ainv[0][0] * temp[0] + m_Calibration.M_Ainv[0][1] * temp[1] + m_Calibration.M_Ainv[0][2] * temp[2];
                Mxyz[1] = m_Calibration.M_Ainv[1][0] * temp[0] + m_Calibration.M_Ainv[1][1] * temp[1] + m_Calibration.M_Ainv[1][2] * temp[2];
                Mxyz[2] = m_Calibration.M_Ainv[2][0] * temp[0] + m_Calibration.M_Ainv[2][1] * temp[1] + m_Calibration.M_Ainv[2][2] * temp[2];
            #else
                for (i = 0; i < 3; i++)
                    Mxyz[i] = (Mxyz[i] - m_Calibration.M_B[i]);
            #endif
            sfusion.updateMag(Mxyz, QMI8658_ODR_MICROS * 1.0e-6);
        #endif
    }

    // float grav[3];
    // grav[0] = (q[1] * q[3] - q[0] * q[2]) * 2;
    // grav[1] = (q[0] * q[1] + q[2] * q[3]) * 2;
    // grav[2] = (sq(q[0]) - sq(q[1]) - sq(q[2]) + sq(q[3]));

    fusedRotation = sfusion.getQuaternionQuat()*sensorOffset;
    float lastAcceleration[]{acceleration[0],acceleration[1],acceleration[2]};
    sfusion.getLinearAcc(acceleration);
    if(!OPTIMIZE_UPDATES ||(abs(lastAcceleration[0]-acceleration[0])>0.1 &&
                            abs(lastAcceleration[1]-acceleration[1])>0.1 && 
                            abs(lastAcceleration[2]-acceleration[2])>0.1))
        this->newAcceleration = true;
    if (!OPTIMIZE_UPDATES || !lastFusedRotationSent.equalsWithEpsilon(fusedRotation))
    {
        newFusedRotation = true;
        lastFusedRotationSent = fusedRotation;
    }
}

float QMI8658Sensor::getTemperature()
{
    return (float)imu.getTemperature()/256;
}
void QMI8658Sensor::AutoCalibrateGyro(int16_t gx, int16_t gy, int16_t gz,int16_t ax,int16_t ay,int16_t az){
    if (Cf != Cr){
        // if(m_Calibration.M_B[0]==0.0 && m_Calibration.M_B[1] == 0.0 && m_Calibration.M_B[2] == 0.0)
        //     CalibrateMag(mx,my,mz);
        // else
            CalibrateGyro(gx,gy,gz,ax,ay,az);
    }else{
        Cr += CaliSamples / 4;
        if (Cr > CaliSamples || Cf >= CaliSamples)
        {
            Cf -= CaliSamples;
            Cr -= CaliSamples;
        }
    }
}
void QMI8658Sensor::CalibrateGyro(int16_t gx, int16_t gy, int16_t gz, int16_t acx, int16_t acy, int16_t acz, uint8_t save)
{
    if (abs(prevM[0] - acx) < AccTolerance*10 && abs(prevM[1] - acy) < AccTolerance*10 && abs(prevM[2] - acz) < AccTolerance*10){
        Cx[Cf] = gx;
        Cy[Cf] = gy;
        Cz[Cf] = gz;
        Cf++;
    }
    prevM[0] = acx;
    prevM[1] = acy;
    prevM[2] = acz;
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
            m_Calibration.temperature = getTemperature();
        }
        if (abs(ax) < (0.001 * CaliSamples/gscale) && abs(ay) <  0.001 * CaliSamples/gscale && abs(az) < 0.001 * CaliSamples/gscale
            && vx < sq(GyroTolerance) * (CaliSamples - 1) && vy < sq(GyroTolerance) * (CaliSamples - 1) && vz < sq(GyroTolerance) * (CaliSamples - 1)){
                SlimeVR::Configuration::CalibrationConfig calibration;
                calibration.type = SlimeVR::Configuration::CalibrationConfigType::QMI8658;
                calibration.data.qmi8658 = m_Calibration;
                SlimeVR::Configuration::QMI8658CalibrationConfig o_Calibration = configuration.getCalibration(sensorId).data.qmi8658;
                configuration.setCalibration(sensorId, calibration);
                if(save)
                {
                    configuration.save(sensorId);
                }
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
    abs(gx-m_Calibration.G_off[0]) < 200 && abs(gy-m_Calibration.G_off[1]) < 200 && abs(gz-m_Calibration.G_off[2]) < 200)
    {
        ledManager.off();
        if(accelDupCnt<=CaliSamples/12){
            accelDupCnt++;
            if(Cf==0 && Cr == CaliSamples-1) Serial.print("Starting Accelero Calibration");
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
        }
        else{
            ledManager.on();
        }
    }
    else if (abs(gx-m_Calibration.G_off[0]) > 800 || abs(gy-m_Calibration.G_off[1]) > 800 || abs(gz-m_Calibration.G_off[2]) > 800){
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
    if (invalidCnt < CaliSamples / 8)
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
    int16_t fifoCnt;
    uint8_t buffer[120];
    while(Cf!=Cr){
        int16_t ax,ay,az,gx,gy,gz;
        fifoCnt = imu.getFIFOCount()-12;
        if(fifoCnt>120) fifoCnt=120;
        imu.getFIFOBytes(buffer,fifoCnt);
            // Serial.printf("Status : %x Cnt : %d\n",fifoStatus, fifoCnt);
        for(uint8_t i=0;i<fifoCnt/12;i++){
            ax = buffer[i*12+0]|buffer[i*12+1]<<8;
            ay = buffer[i*12+2]|buffer[i*12+3]<<8;
            az = buffer[i*12+4]|buffer[i*12+5]<<8;
            gx = buffer[i*12+6]|buffer[i*12+7]<<8;
            gy = buffer[i*12+8]|buffer[i*12+9]<<8;
            gz = buffer[i*12+10]|buffer[i*12+11]<<8;
            CalibrateGyro(gx,gy,gz,ax,ay,az,1);
            if(Cf==Cr) break;
        }
        delay(10);
    }
    m_Logger.debug("Gathering accelerometer data...");
    Cf=0; Cr=CaliSamples-1;
    accelDupCnt = 0;
    while(Cf!=Cr){
        int16_t ax,ay,az,gx,gy,gz;
        fifoCnt = imu.getFIFOCount()-12;
        if(fifoCnt>120) fifoCnt=120;
        imu.getFIFOBytes(buffer,fifoCnt);
            // Serial.printf("Status : %x Cnt : %d\n",fifoStatus, fifoCnt);
        for(uint8_t i=0;i<fifoCnt/12;i++){
            ax = buffer[i*12+0]|buffer[i*12+1]<<8;
            ay = buffer[i*12+2]|buffer[i*12+3]<<8;
            az = buffer[i*12+4]|buffer[i*12+5]<<8;
            gx = buffer[i*12+6]|buffer[i*12+7]<<8;
            gy = buffer[i*12+8]|buffer[i*12+9]<<8;
            gz = buffer[i*12+10]|buffer[i*12+11]<<8;
            CalibrateAcc(ax,ay,az,gx,gy,gz);
            if(Cf==Cr) break;
        }
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
