#include "QMI8658.h"
#include "I2Cdev.h"

QMI8658::QMI8658(){};

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up).
 */
void QMI8658::initialize(uint8_t addr, uint8_t maddr)
{
    devAddr = addr;
    magAddr = maddr;
    /* Issue a soft-reset to bring the device into a clean state */
    I2Cdev::writeByte(devAddr, QMI8658_RA_RESET, 0xB0);

    // mag -> setMode(0);
    I2Cdev::writeByte(magAddr, 0x0B, 0x01);

    // mag -> setMode(0x0F);
    I2Cdev::writeByte(magAddr, 0x09, 0x01 | 0x01 | 0x00 | 0x01);

    delay(500);
    I2Cdev::writeByte(devAddr, QMI8658_RA_CTRL1, 0b01100000);
    delay(100);
    /* config default accelerometer */
    I2Cdev::writeByte(devAddr, QMI8658_RA_CTRL2, 0b00100100);
    delay(100);
    /* config default gyroscope */
    I2Cdev::writeByte(devAddr, QMI8658_RA_CTRL3, 0b01010100);
    delay(100);
    // /* config default magnetometer */
    // I2Cdev::writeByte(devAddr, QMI8658_RA_CTRL4, 0b00000000);
    /* config default filter setting */
    I2Cdev::writeByte(devAddr, QMI8658_RA_CTRL5, 0b00010001);
    delay(100);
    /* config default AttitudeEngine Settings */
    // I2Cdev::writeByte(devAddr, QMI8658_RA_CTRL6, 0b00000111);
    I2Cdev::writeByte(devAddr, QMI8658_RA_CTRL6, 0b00100110);
    delay(100);
    /* config Mag/Gyr/Acc Enabled/Disabled */
    I2Cdev::writeByte(devAddr, QMI8658_RA_CTRL7, 0b00000011);
    // I2Cdev::writeByte(devAddr, QMI8658_RA_CTRL7, 0b00001011);
    delay(100);
}

void QMI8658::getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    I2Cdev::readBytes(devAddr, QMI8658_RA_AX_L, 12, buffer);
    *ax = (((int16_t)buffer[1]) << 8) | buffer[0];
    *ay = (((int16_t)buffer[3]) << 8) | buffer[2];
    *az = (((int16_t)buffer[5]) << 8) | buffer[4];
    *gx = (((int16_t)buffer[7]) << 8) | buffer[6];
    *gy = (((int16_t)buffer[9]) << 8) | buffer[8];
    *gz = (((int16_t)buffer[11]) << 8) | buffer[10];
}

void QMI8658::getMotion9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz)
{
    getMotion6(ax, ay, az, gx, gy, gz);
    getMagneto(mx, my, mz);
}

void QMI8658::getMagneto(int16_t *mx, int16_t *my, int16_t *mz)
{
    I2Cdev::readBytes(magAddr, 0x00, 6, buffer);
    *mx = (((int16_t)buffer[1]) << 8) | buffer[0];
    *my = (((int16_t)buffer[3]) << 8) | buffer[2];
    *mz = (((int16_t)buffer[5]) << 8) | buffer[4];
}

void QMI8658::getAcceleration(int16_t *ax, int16_t *ay, int16_t *az)
{
    I2Cdev::readBytes(devAddr, QMI8658_RA_AX_L, 12, buffer);
    *ax = (((int16_t)buffer[1]) << 8) | buffer[0];
    *ay = (((int16_t)buffer[3]) << 8) | buffer[2];
    *az = (((int16_t)buffer[5]) << 8) | buffer[4];
}

void QMI8658::getQuatDiff(int16_t *dqw, int16_t *dqx, int16_t *dqy, int16_t *dqz)
{
    uint8_t tmp, tmp2;
    I2Cdev::readByte(devAddr, QMI8658_RA_AE_REG1, &tmp);
    I2Cdev::readByte(devAddr, QMI8658_RA_AE_REG2, &tmp2);
    I2Cdev::readBytes(devAddr, QMI8658_RA_dQW_L, 8, buffer);
    *dqw = (((int16_t)buffer[1]) << 8) | buffer[0];
    *dqx = (((int16_t)buffer[3]) << 8) | buffer[2];
    *dqy = (((int16_t)buffer[5]) << 8) | buffer[4];
    *dqz = (((int16_t)buffer[7]) << 8) | buffer[6];
    Serial.printf("%02x %02x> ", tmp, tmp2);
}

void QMI8658::getVeloDiff(int16_t *dvx, int16_t *dvy, int16_t *dvz)
{
    I2Cdev::readBytes(devAddr, QMI8658_RA_dVX_L, 6, buffer);
    *dvx = (((int16_t)buffer[1]) << 8) | buffer[0];
    *dvy = (((int16_t)buffer[3]) << 8) | buffer[2];
    *dvz = (((int16_t)buffer[5]) << 8) | buffer[4];
}

int16_t QMI8658::getTemperature()
{
    I2Cdev::readBytes(devAddr, QMI8658_RA_TEMP_L, 2, buffer);
    return ((int16_t)buffer[1]) << 8 | buffer[0];
}

uint8_t QMI8658::getDeviceID()
{
    I2Cdev::readByte(devAddr, QMI8658_RA_CHIP_ID, buffer);
    return buffer[0];
}

uint8_t QMI8658::getStatus0()
{
    I2Cdev::readByte(devAddr, QMI8658_RA_STATUS0, buffer);
    return buffer[0];
}

uint8_t QMI8658::getStatus1()
{
    I2Cdev::readByte(devAddr, QMI8658_RA_STATUS1, buffer);
    return buffer[0];
}

bool QMI8658::testConnection()
{
    return (QMI8658_CHIP_ID == getDeviceID());
}