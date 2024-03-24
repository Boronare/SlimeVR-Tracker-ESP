/**
 ******************************************************************************
 * @file    LSM6DSRSensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Implementation of an LSM6DSR Inertial Measurement Unit (IMU) 6 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "LSM6DSR.h"
#include "I2Cdev.h"


/* Class Implementation ------------------------------------------------------*/

class I2CdevMod : public I2Cdev {
    public:
        static bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
            uint8_t b;
            if (readByte(devAddr, regAddr, &b) != 0) {
                uint8_t mask = ((1 << length) - 1) << bitStart;
                data <<= bitStart; // shift data into correct position
                data &= mask; // zero all non-important bits in data
                b &= ~(mask); // zero all important bits in existing byte
                b |= data; // combine data with existing byte
                return writeByte(devAddr, regAddr, b);
            } else {
                return false;
            }
        }
        static int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout) {
            uint8_t count, b;
            if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
                uint8_t mask = ((1 << length) - 1) << bitStart;
                b &= mask;
                b >>= bitStart;
                *data = b;
            }
            return count;
        }
};

LSM6DSR::LSM6DSR() {};

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
void LSM6DSR::initialize(uint8_t addr,
    lsm6dsr_odr_g_t gyroRate, lsm6dsr_fs_g_t gyroRange,
    lsm6dsr_odr_xl_t accelRate, lsm6dsr_fs_xl_t accelRange)
{
    devAddr = addr;
    setRegister(LSM6DSR_CTRL3_C, 0b00000011); // SW_RESET
    delay(50);
    setGyroRate(gyroRate);
    setAccelRate(accelRate);
    setFullScaleGyroRange(gyroRange);
    setFullScaleAccelRange(accelRange);
    setTimestampEnabled(true);
    
    setRegister(LSM6DSR_CTRL4_C, 0b00000010);//Enable LPF
    // setRegister(LSM6DSR_CTRL6_C, 0b00010010);
    // setRegister(LSM6DSR_CTRL7_G, 0b01010000);//Enable HPF

    delay(50);
}

/** Read a LSM6DSR register directly.
 * @param reg register address
 * @return 8-bit register value
 */
uint8_t LSM6DSR::getRegister(uint8_t reg) {
    I2CdevMod::readByte(devAddr, reg, buffer);
    return buffer[0];
}

/** Write a LSM6DSR register directly.
 * @param reg register address
 * @param data 8-bit register value
 */
void LSM6DSR::setRegister(uint8_t reg, uint8_t data) {
    I2CdevMod::writeByte(devAddr, reg, data);
}


void LSM6DSR::setMagDevice(uint8_t addr,uint8_t regAddr,uint8_t odr) {
    accessSensorhub();
    delay(3);
    setRegister(LSM6DSR_MASTER_CONFIG,0b00000101);
    setRegister(LSM6DSR_SLV0_ADD, addr << 1); // 0 bit of address is reserved and needs to be shifted
    setRegister(LSM6DSR_SLV0_CONFIG, odr<<6|0|0);
    setRegister(LSM6DSR_SLV1_ADD, addr << 1|1); // read mode enabled
    setRegister(LSM6DSR_SLV1_SUBADD, regAddr);
    setRegister(LSM6DSR_SLV1_CONFIG, 8|6);
    delay(3);
    accessDefault();
    delay(3);
}

void LSM6DSR::setMagRegister(uint8_t addr, uint8_t value) {
    accessSensorhub();
    delay(3);
    setRegister(LSM6DSR_SLV0_SUBADD, addr);
    setRegister(LSM6DSR_DATAWRITE_SLV0, value);
    setRegister(LSM6DSR_MASTER_CONFIG,0b01000101);
    delay(3);
    accessDefault();
    delay(3);
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b11010001, 0xD1).
 * @return Device ID (should be 0xD1)
 * @see BMI160_RA_CHIP_ID
 */
uint8_t LSM6DSR::getDeviceID() {
    I2CdevMod::readByte(devAddr, LSM6DSR_WHO_AM_I, buffer);
    return buffer[0];
}

/** Verify the SPI connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool LSM6DSR::testConnection()
{
    uint8_t device_id = getDeviceID();
    return (LSM6DSR_ID == device_id);
}

/** Set gyroscope output data rate.
 * @param rate New output data rate
 * @see getGyroRate()
 * @see BMI160_GYRO_RATE_25HZ
 * @see BMI160_RA_GYRO_CONF
 */
void LSM6DSR::setGyroRate(uint8_t rate) {
    I2CdevMod::writeBits(devAddr, LSM6DSR_CTRL2_G,
                   4,
                   4, rate);
}

/** Set accelerometer output data rate.
 * @param rate New output data rate
 * @see getAccelRate()
 * @see BMI160_RA_ACCEL_CONF
 */
void LSM6DSR::setAccelRate(uint8_t rate) {
    I2CdevMod::writeBits(devAddr, LSM6DSR_CTRL1_XL,
                   4,
                   4, rate);
}

/** Get full-scale gyroscope range.
 * The gyr_range parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 4 = +/-  125 degrees/sec
 * 3 = +/-  250 degrees/sec
 * 2 = +/-  500 degrees/sec
 * 1 = +/- 1000 degrees/sec
 * 0 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see BMI160_RA_GYRO_RANGE
 * @see BMI160GyroRange
 */
uint8_t LSM6DSR::getFullScaleGyroRange() {
    I2CdevMod::readBits(devAddr, LSM6DSR_CTRL2_G,
                         0,
                         4, buffer);
    return buffer[0];
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleGyroRange()
 */
void LSM6DSR::setFullScaleGyroRange(uint8_t range) {
    I2CdevMod::writeBits(devAddr, LSM6DSR_CTRL2_G,
                   0,
                   4, range);
}

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 *  3 = +/- 2g
 *  5 = +/- 4g
 *  8 = +/- 8g
 * 12 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see BMI160_RA_ACCEL_RANGE
 * @see BMI160AccelRange
 */
uint8_t LSM6DSR::getFullScaleAccelRange() {
    I2CdevMod::readBits(devAddr, LSM6DSR_CTRL1_XL,
                         2,
                         2, buffer);
    return buffer[0];
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 * @see BMI160AccelRange
 */
void LSM6DSR::setFullScaleAccelRange(uint8_t range) {
    I2CdevMod::writeBits(devAddr, LSM6DSR_CTRL1_XL,
                   2,
                   2, range);
}

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer.
 *
 * In "headerless" FIFO mode, it is directly proportional to the number of
 * samples available given the set of sensor data bound to be stored in the
 * FIFO. See @ref getFIFOHeaderModeEnabled().
 *
 * @param outCount Current FIFO buffer size
 * @return Bool if value was read successfully
 * @see BMI160_RA_FIFO_LENGTH_0
 */
bool LSM6DSR::getFIFOCount(uint16_t* outCount) {
    bool ok = I2CdevMod::readBytes(devAddr, LSM6DSR_FIFO_STATUS1, 2, buffer) >= 0;
    if (!ok) return false;
    *outCount = (((int16_t)buffer[1]&0b11) << 8) | buffer[0];
    return ok;
}

/** Reset the FIFO.
 * This command clears all data in the FIFO buffer.  It is recommended
 * to invoke this after reconfiguring the FIFO.
 *
 * @see BMI160_RA_CMD
 * @see BMI160_CMD_FIFO_FLUSH
 */
void LSM6DSR::resetFIFO() {
    disableFIFO();
    delay(1);
    enableFIFO();
}

/** Get data frames from FIFO buffer.
 * This register is used to read and write data frames from the FIFO buffer.
 * Data is written to the FIFO in order of DATA register number (from lowest
 * to highest) corresponding to the FIFO data sources enabled (@see
 * getGyroFIFOEnabled() and getAccelFIFOEnabled()).
 *
 * The data frame format depends on the enabled data sources and also on
 * the FIFO header-mode setting (@see getFIFOHeaderModeEnabled()).
 *
 * It is strongly recommended, where possible, to read whole frames from the
 * FIFO.  Partially-read frames will be repeated until fully read out.
 *
 * If the FIFO buffer has filled to the point where subsequent writes may
 * cause data loss, the status bit ffull_int is automatically set to 1. This bit
 * is located in INT_STATUS[1]. When the FIFO buffer has overflowed, the oldest
 * data will be lost and new data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return a magic number
 * (@see BMI160_FIFO_DATA_INVALID) until new data is available. The user should
 * check FIFO_LENGTH to ensure that the FIFO buffer is not read when empty (see
 * @getFIFOCount()).
 *
 * @param data Data frames from FIFO buffer
 * @param length Buffer length
 * @return Bool if value was read successfully
 */
bool LSM6DSR::getFIFOBytes(uint8_t *data) {
    bool ok = I2CdevMod::readBytes(devAddr, LSM6DSR_FIFO_DATA_OUT_TAG, 7, data) >= 0;
    return ok;
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see BMI160_RA_GYRO_X_L
 */
void LSM6DSR::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    I2CdevMod::readBytes(devAddr, LSM6DSR_OUTX_L_G, 12, buffer);
    *gx = (((int16_t)buffer[1])  << 8) | buffer[0];
    *gy = (((int16_t)buffer[3])  << 8) | buffer[2];
    *gz = (((int16_t)buffer[5])  << 8) | buffer[4];
    *ax = (((int16_t)buffer[7])  << 8) | buffer[6];
    *ay = (((int16_t)buffer[9])  << 8) | buffer[8];
    *az = (((int16_t)buffer[11]) << 8) | buffer[10];
}

/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Output Data Rate
 * as configured by @see getAccelRate()
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Output Data Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale configured by
 * @setFullScaleAccelRange. For each full scale setting, the accelerometers'
 * sensitivity per LSB is shown in the table below:
 *
 * <pre>
 * Full Scale Range | LSB Sensitivity
 * -----------------+----------------
 * +/- 2g           | 8192 LSB/mg
 * +/- 4g           | 4096 LSB/mg
 * +/- 8g           | 2048 LSB/mg
 * +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see BMI160_RA_ACCEL_X_L
 */
void LSM6DSR::getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    I2CdevMod::readBytes(devAddr, LSM6DSR_OUTX_L_A, 6, buffer);
    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}

/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_ACCEL_X_L
 */
int16_t LSM6DSR::getAccelerationX() {
    I2CdevMod::readBytes(devAddr, LSM6DSR_OUTX_L_A, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get Y-axis accelerometer reading.
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_ACCEL_Y_L
 */
int16_t LSM6DSR::getAccelerationY() {
    I2CdevMod::readBytes(devAddr, LSM6DSR_OUTY_L_A, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get Z-axis accelerometer reading.
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_ACCEL_Z_L
 */
int16_t LSM6DSR::getAccelerationZ() {
    I2CdevMod::readBytes(devAddr, LSM6DSR_OUTZ_L_A, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get current internal temperature as a signed 16-bit integer.
 *  The resolution is typically 1/2^9 degrees Celcius per LSB, at an
 *  offset of 23 degrees Celcius.  For example:
 *
 * <pre>
 * Value    | Temperature
 * ---------+----------------
 * 0x7FFF   | 87 - 1/2^9 degrees C
 * ...      | ...
 * 0x0000   | 23 degrees C
 * ...      | ...
 * 0x8001   | -41 + 1/2^9 degrees C
 * 0x8000   | Invalid
 *
 * @param out Temperature reading in 16-bit 2's complement format
 * @return Bool if value was read successfully
 * @see BMI160_RA_TEMP_L
 */
bool LSM6DSR::getTemperature(int16_t* out) {
    bool ok = I2CdevMod::readBytes(devAddr, LSM6DSR_OUT_TEMP_L, 2, buffer) >= 0;
    if (!ok) return false;
    *out = (((int16_t)buffer[1]) << 8) | buffer[0];
    return ok;
}

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Output Data Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale configured by
 * @setFullScaleGyroRange(). For each full scale setting, the gyroscopes'
 * sensitivity per LSB is shown in the table below:
 *
 * <pre>
 * Full Scale Range   | LSB Sensitivity
 * -------------------+----------------
 * +/- 125  degrees/s | 262.4 LSB/deg/s
 * +/- 250  degrees/s | 131.2 LSB/deg/s
 * +/- 500  degrees/s | 65.5  LSB/deg/s
 * +/- 1000 degrees/s | 32.8  LSB/deg/s
 * +/- 2000 degrees/s | 16.4  LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see BMI160_RA_GYRO_X_L
 */
void LSM6DSR::getRotation(int16_t* x, int16_t* y, int16_t* z) {
    I2CdevMod::readBytes(devAddr, LSM6DSR_OUTX_L_G, 6, buffer);
    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}

/** Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_GYRO_X_L
 */
int16_t LSM6DSR::getRotationX() {
    I2CdevMod::readBytes(devAddr, LSM6DSR_OUTX_L_G, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_GYRO_Y_L
 */
int16_t LSM6DSR::getRotationY() {
    I2CdevMod::readBytes(devAddr, LSM6DSR_OUTY_L_G, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_GYRO_Z_L
 */
int16_t LSM6DSR::getRotationZ() {
    I2CdevMod::readBytes(devAddr, LSM6DSR_OUTZ_L_G, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

/** Get magnetometer readings
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see BMI160_RA_GYRO_Z_L
 */
void LSM6DSR::getMagnetometer(int16_t* mx, int16_t* my, int16_t* mz) {
    accessSensorhub();
    I2CdevMod::readBytes(devAddr, LSM6DSR_SENSOR_HUB_1, 6, buffer);
    *mx = (((int32_t)buffer[0] << 8) | buffer[1])-32768;
    *my = (((int32_t)buffer[2] << 8) | buffer[3])-32768;
    *mz = (((int32_t)buffer[4] << 8) | buffer[5])-32768;
    accessDefault();
}

void LSM6DSR::getMagnetometerXYZBuffer(uint8_t* data) {
    accessSensorhub();
    I2CdevMod::readBytes(devAddr, LSM6DSR_SENSOR_HUB_1, 6, data);
    accessDefault();
}

bool LSM6DSR::getGyroDrdy() {
    I2CdevMod::readBits(devAddr, LSM6DSR_STATUS_REG, 1, 1, buffer);
    return buffer[0];
}

void LSM6DSR::waitForGyroDrdy() {
    do {
        getGyroDrdy();
        if (!buffer[0]) delayMicroseconds(150);
    } while (!buffer[0]);
}

bool LSM6DSR::getAccelDrdy() {
    I2CdevMod::readBits(devAddr, LSM6DSR_STATUS_REG, 1, 0, buffer);
    return buffer[0];
}

void LSM6DSR::waitForAccelDrdy() {
    do {
        getAccelDrdy();
        if (!buffer[0]) delayMicroseconds(150);
    } while (!buffer[0]);
}

bool LSM6DSR::getFIFOEnabled(){
    uint8_t data = getRegister(LSM6DSR_FIFO_CTRL4);
    return data&0b111;
}
void LSM6DSR::enableFIFO() {
    uint8_t gr = getGyroRate();
    uint8_t ar = getAccelRate();
    setRegister(LSM6DSR_FIFO_CTRL3,gr<<4|ar);
    setRegister(LSM6DSR_FIFO_CTRL4,0b01000110);
}
void LSM6DSR::disableFIFO() {
    setRegister(LSM6DSR_FIFO_CTRL4,0b00000000);
}

void LSM6DSR::setTimestampEnabled(bool enabled){
    I2CdevMod::writeBit(devAddr, LSM6DSR_CTRL10_C,5,enabled);
}
bool LSM6DSR::getTimestampEnabled(){
    uint8_t enabled;
    enabled = I2CdevMod::readBit(devAddr,LSM6DSR_CTRL10_C,5,&enabled);
    return enabled;
}
bool LSM6DSR::getSensorTime(uint32_t *v_sensor_time_u32) {
    bool ok = I2CdevMod::readBytes(devAddr, LSM6DSR_TIMESTAMP0, 4, buffer) >= 0;
    if (!ok) return false;
    *v_sensor_time_u32 = (uint32_t)(
        (((uint32_t)buffer[3]) << 24) |
        (((uint32_t)buffer[2]) << 16) |
        (((uint32_t)buffer[1]) << 8)  |
        ((uint32_t)buffer[0])
    );
    return ok;
}
void LSM6DSR::accessDefault(){
   setRegister(LSM6DSR_FUNC_CFG_ACCESS,0);
}
void LSM6DSR::accessSensorhub(){
   setRegister(LSM6DSR_FUNC_CFG_ACCESS,0b01000000);
   delay(5);
}
void LSM6DSR::accessFunctions(){
   setRegister(LSM6DSR_FUNC_CFG_ACCESS,0b10000000);
}
uint8_t LSM6DSR::getGyroRate(){
    uint8_t data;
    I2CdevMod::readBits(devAddr, LSM6DSR_CTRL2_G,
                   4,
                   4, &data);
    return data;
}
uint8_t LSM6DSR::getAccelRate(){
    uint8_t data;
    I2CdevMod::readBits(devAddr, LSM6DSR_CTRL1_XL,
                   4,
                   4, &data);
    return data;
}