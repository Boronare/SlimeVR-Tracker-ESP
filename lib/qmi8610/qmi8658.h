#ifndef _QMI8658_H_
#define _QMI8658_H_

#include "Arduino.h"
#include "I2Cdev.h"

#define QMI8658_RA_CHIP_ID      0x00
#define QMI8658_CHIP_ID         0x05

#define QMI8658_RA_REVISION     0x01

#define QMI8658_RA_CTRL1        0x02
#define QMI8658_RA_CTRL2        0x03
#define QMI8658_RA_CTRL3        0x04
#define QMI8658_RA_CTRL4        0x05
#define QMI8658_RA_CTRL5        0x06
#define QMI8658_RA_CTRL6        0x07
#define QMI8658_RA_CTRL7        0x08
#define QMI8658_RA_CTRL8        0x09
#define QMI8658_RA_CTRL9        0x0A

#define QMI8658_RA_CAL1_L       0x0B
#define QMI8658_RA_CAL1_H       0x0C
#define QMI8658_RA_CAL2_L       0x0D
#define QMI8658_RA_CAL2_H       0x0E
#define QMI8658_RA_CAL3_L       0x0F
#define QMI8658_RA_CAL3_H       0x10
#define QMI8658_RA_CAL4_L       0x11
#define QMI8658_RA_CAL4_H       0x12

#define QMI8658_RA_FIFOO_WTM_TH 0x13
#define QMI8658_RA_FIFO_CTRL    0x14
#define QMI8658_FIFO_SMPL_CNT   0x15
#define QMI8658_RA_FIFO_STATUS  0x16
#define QMI8658_RA_FIFO_DATA    0x17

#define QMI8658_RA_I2CM_STATUS  0x2C
#define QMI8658_RA_STATUSINT    0x2D
#define QMI8658_RA_STATUS0      0x2E
#define QMI8658_RA_STATUS1      0x2F

#define QMI8658_RA_TIMESTAMP_LOW 0x30
#define QMI8658_RA_TIMESTAMP_MID 0x31
#define QMI8658_RA_TIMESTAMP_HI 0x32

#define QMI8658_RA_TEMP_L       0x33
#define QMI8658_RA_TEMP_H       0x34

#define QMI8658_RA_AX_L         0x35
#define QMI8658_RA_AX_H         0x36
#define QMI8658_RA_AY_L         0x37
#define QMI8658_RA_AY_H         0x38
#define QMI8658_RA_AZ_L         0x39
#define QMI8658_RA_AZ_H         0x3A

#define QMI8658_RA_GX_L         0x3B
#define QMI8658_RA_GX_H         0x3C
#define QMI8658_RA_GY_L         0x3D
#define QMI8658_RA_GY_H         0x3E
#define QMI8658_RA_GZ_L         0x3F
#define QMI8658_RA_GZ_H         0x40

#define QMI8658_RA_MX_L         0x41
#define QMI8658_RA_MX_H         0x42
#define QMI8658_RA_MY_L         0x43
#define QMI8658_RA_MY_H         0x44
#define QMI8658_RA_MZ_L         0x45
#define QMI8658_RA_MZ_H         0x46

#define QMI8658_RA_dQW_L        0x49
#define QMI8658_RA_dQW_H        0x4A
#define QMI8658_RA_dQX_L        0x4B
#define QMI8658_RA_dQX_H        0x4C
#define QMI8658_RA_dQY_L        0x4D
#define QMI8658_RA_dQY_H        0x4E
#define QMI8658_RA_dQZ_L        0x4F
#define QMI8658_RA_dQZ_H        0x50

#define QMI8658_RA_dVX_L        0x51
#define QMI8658_RA_dVX_H        0x52
#define QMI8658_RA_dVY_L        0x53
#define QMI8658_RA_dVY_H        0x54
#define QMI8658_RA_dVZ_L        0x55
#define QMI8658_RA_dVZ_H        0x56

#define QMI8658_RA_AE_REG1      0x57
#define QMI8658_RA_AE_REG2      0x58

#define QMI8658_RA_RESET        0x60


class QMI8658 {
    public:
        QMI8658();
        void initialize(uint8_t addr);
        bool testConnection();

        void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
        void getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);

        void getAcceleration(int16_t* ax, int16_t* ay, int16_t* az);

        void getQuatDiff(int16_t* dqw, int16_t* dqx, int16_t* dqy, int16_t* dqz);
        void getVeloDiff(int16_t* dvx, int16_t* dvy, int16_t* dvz);

        int16_t getTemperature();

        bool getFIFOHeaderModeEnabled();
        void setFIFOHeaderModeEnabled(bool enabled);
        void resetFIFO();

        uint16_t getFIFOCount();
        void getFIFOBytes(uint8_t *data, uint16_t length);

        uint8_t getDeviceID();

        uint8_t getStatus0();
        uint8_t getStatus1();

    private:
        uint8_t buffer[20];
        uint8_t devAddr;
};

#endif