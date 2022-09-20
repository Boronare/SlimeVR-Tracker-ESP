#include "QMI8610.h"
#include "I2Cdev.h"

QMI8610::QMI8610() {};

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up).
 */
void QMI8610::initialize(uint8_t addr)
{
    devAddr = addr;
    /* Issue a soft-reset to bring the device into a clean state */
    I2Cdev::writeByte(devAddr, QMI8610_RA_CTRL1, 0b10001101);
    delay(1);
    /* config default accelerometer */
    I2Cdev::writeByte(devAddr, QMI8610_RA_CTRL2, 0b10101010);
    /* config default gyroscope */
    I2Cdev::writeByte(devAddr, QMI8610_RA_CTRL3, 0b11011010);
    /* config default filter setting */
    I2Cdev::writeByte(devAddr, QMI8610_RA_CTRL5, 0b01111111);
    /* config default AttitudeEngine Settings */
    I2Cdev::writeByte(devAddr, QMI8610_RA_CTRL6, 0b10100101);
    /* config Mag/Gyr/Acc Enabled/Disabled */
    I2Cdev::writeByte(devAddr, QMI8610_RA_CTRL7, 0b10001011);
    delay(100);
}

void QMI8610::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz){
    I2Cdev::readBytes(devAddr, QMI8610_RA_AX_L, 12, buffer);
    *ax = (((int16_t)buffer[1])  << 8) | buffer[0];
    *ay = (((int16_t)buffer[3])  << 8) | buffer[2];
    *az = (((int16_t)buffer[5])  << 8) | buffer[4];
    *gx = (((int16_t)buffer[7])  << 8) | buffer[6];
    *gy = (((int16_t)buffer[9])  << 8) | buffer[8];
    *gz = (((int16_t)buffer[11]) << 8) | buffer[10];
}

void QMI8610::getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz){
    I2Cdev::readBytes(devAddr, QMI8610_RA_AX_L, 18, buffer);
    *ax = (((int16_t)buffer[1])  << 8) | buffer[0];
    *ay = (((int16_t)buffer[3])  << 8) | buffer[2];
    *az = (((int16_t)buffer[5])  << 8) | buffer[4];
    *gx = (((int16_t)buffer[7])  << 8) | buffer[6];
    *gy = (((int16_t)buffer[9])  << 8) | buffer[8];
    *gz = (((int16_t)buffer[11]) << 8) | buffer[10];
    *mx = (((int16_t)buffer[13])  << 8) | buffer[12];
    *my = (((int16_t)buffer[15])  << 8) | buffer[14];
    *mz = (((int16_t)buffer[17]) << 8) | buffer[16];
}

void QMI8610::getAcceleration(int16_t* ax, int16_t* ay, int16_t* az){
    I2Cdev::readBytes(devAddr, QMI8610_RA_AX_L, 12, buffer);
    *ax = (((int16_t)buffer[1])  << 8) | buffer[0];
    *ay = (((int16_t)buffer[3])  << 8) | buffer[2];
    *az = (((int16_t)buffer[5])  << 8) | buffer[4];
}

void QMI8610::getQuatDiff(int16_t* dqw, int16_t* dqx, int16_t* dqy, int16_t* dqz){
    I2Cdev::readBytes(devAddr, QMI8610_RA_dQW_L, 8, buffer);
    *dqw = (((int16_t)buffer[1])  << 8) | buffer[0];
    *dqx = (((int16_t)buffer[3])  << 8) | buffer[2];
    *dqy = (((int16_t)buffer[5])  << 8) | buffer[4];
    *dqz = (((int16_t)buffer[7])  << 8) | buffer[6];
}

void QMI8610::getVeloDiff(int16_t* dvx, int16_t* dvy, int16_t* dvz){
    I2Cdev::readBytes(devAddr, QMI8610_RA_dVX_L, 6, buffer);
    *dvx = (((int16_t)buffer[1])  << 8) | buffer[0];
    *dvy = (((int16_t)buffer[3])  << 8) | buffer[2];
    *dvz = (((int16_t)buffer[5])  << 8) | buffer[4];
}

int8_t QMI8610::getTemperature(){
    I2Cdev::readByte(devAddr, QMI8610_RA_TEMP, buffer);
    return buffer[0];
}

uint8_t QMI8610::getDeviceID(){
    I2Cdev::readByte(devAddr, QMI8610_RA_CHIP_ID, buffer);
    return buffer[0];
}

uint8_t QMI8610::getStatus0(){
    I2Cdev::readByte(devAddr, QMI8610_RA_STATUS0, buffer);
    return buffer[0];
}

uint8_t QMI8610::getStatus1(){
    I2Cdev::readByte(devAddr, QMI8610_RA_STATUS1, buffer);
    return buffer[0];
}

bool QMI8610::testConnection(){
    return (QMI8610_CHIP_ID == getDeviceID());
}