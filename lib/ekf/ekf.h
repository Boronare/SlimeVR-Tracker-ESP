/*************************************************************************************************************
 * Class for Discrete Extended Kalman Filter
 * 
 * 
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/
#ifndef EKF_H
#define EKF_H

#include "../../src/globals.h"

/* ================================================== The AHRS/IMU variables ================================================== */
/* Gravity vector constant (align with global Z-axis) */
#define IMU_ACC_Z0          (1)
/* Magnetic vector constant (align with local magnetic vector) */
static float IMU_MAG_B0[3] = {1, 0, 0.000000};
/* EKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT      (10.)
#define Q_INIT      (1e-6)
#define R_INIT_ACC  (0.15/100.)
#define R_INIT_MAG  (0.15/100.)

#define SS_X_LEN    4
#define SS_Z_LEN    6
#define SS_U_LEN    3
#define SS_DT_MILIS samplingRateInMillis
#define SS_DT       SS_DT_MILIS/1000   /* Sampling time */

#define float_prec_ZERO     (1e-7)
#define float_prec_ZERO_ECO (1e-5)      /* 'Economical' zero, for noisy calculation where 'somewhat zero' is good enough */

/* P(k=0) variable -------------------------------------------------------------------------------------------------- */
static float EKF_PINIT[SS_X_LEN*SS_X_LEN] = {P_INIT, 0,      0,      0,
                                                0,      P_INIT, 0,      0,
                                                0,      0,      P_INIT, 0,
                                                0,      0,      0,      P_INIT};
/* Q constant ------------------------------------------------------------------------------------------------------- */
static float EKF_QINIT[SS_X_LEN*SS_X_LEN] = {Q_INIT, 0,      0,      0,
                                                0,      Q_INIT, 0,      0,
                                                0,      0,      Q_INIT, 0,
                                                0,      0,      0,      Q_INIT};
/* R constant ------------------------------------------------------------------------------------------------------- */
static float EKF_RINIT[SS_Z_LEN*SS_Z_LEN] = {R_INIT_ACC, 0,          0,          0,          0,          0,
                                                0,          R_INIT_ACC, 0,          0,          0,          0,
                                                0,          0,          R_INIT_ACC, 0,          0,          0,
                                                0,          0,          0,          R_INIT_MAG, 0,          0,
                                                0,          0,          0,          0,          R_INIT_MAG, 0,
                                                0,          0,          0,          0,          0,          R_INIT_MAG};

class EKF
{
public:
    EKF(float* XInit, float* P=EKF_PINIT, float* Q=EKF_QINIT, float* R=EKF_RINIT);
    void vReset(float* XInit, float* P=EKF_PINIT, float* Q=EKF_QINIT, float* R=EKF_RINIT);
    bool bUpdate(float* Y, float* U);
    bool bUpdate(float gyro[3],float acc[3],float mag[3]);
    float* GetX() { return X_Est; }
    float* GetY() { return Y_Est; }
    float* GetP() { return P; }
    float* GetErr() { return Err; }

protected:
    bool bNonlinearUpdateX (float* X_Next, float* X, float* U){
    /* Insert the nonlinear update transformation here
     *          x(k+1) = f[x(k), u(k)]
     *
     * The quaternion update function:
     *  q0_dot = 1/2. * (  0   - p*q1 - q*q2 - r*q3)
     *  q1_dot = 1/2. * ( p*q0 +   0  + r*q2 - q*q3)
     *  q2_dot = 1/2. * ( q*q0 - r*q1 +  0   + p*q3)
     *  q3_dot = 1/2. * ( r*q0 + q*q1 - p*q2 +  0  )
     * 
     * Euler method for integration:
     *  q0 = q0 + q0_dot * dT;
     *  q1 = q1 + q1_dot * dT;
     *  q2 = q2 + q2_dot * dT;
     *  q3 = q3 + q3_dot * dT;
     */
    float q0, q1, q2, q3;
    float p, q, r;
    
    q0 = X[0];
    q1 = X[1];
    q2 = X[2];
    q3 = X[3];
    
    p = U[0];
    q = U[1];
    r = U[2];
    
    X_Next[0] = (0.5 * (+0.00 -p*q1 -q*q2 -r*q3))*SS_DT + q0;
    X_Next[1] = (0.5 * (+p*q0 +0.00 +r*q2 -q*q3))*SS_DT + q1;
    X_Next[2] = (0.5 * (+q*q0 -r*q1 +0.00 +p*q3))*SS_DT + q2;
    X_Next[3] = (0.5 * (+r*q0 +q*q1 -p*q2 +0.00))*SS_DT + q3;
    
    
    /* ======= Additional ad-hoc quaternion normalization to make sure the quaternion is a unit vector (i.e. ||q|| = 1) ======= */
    float XnS = sqrt(sq(X_Next[0])+sq(X_Next[1])+sq(X_Next[2])+sq(X_Next[3]));
    if(XnS==0) return false;
    for(uint8_t i=0;i<4;i++){
        X_Next[i]/=XnS;
    }
    
    return true;
}
    bool bNonlinearUpdateY (float* Y, float* X, float* U){
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     *
     * The measurement output is the gravitational and magnetic projection to the body:
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                    2*(q1*q2+q0*q3),                    2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3), (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                    2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                    2*(q2*q3-q0*q1), (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     * 
     *  G_proj_sens = DCM * [0 0 1]             --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    float q0, q1, q2, q3;
    float q0_2, q1_2, q2_2, q3_2;

    q0 = X[0];
    q1 = X[1];
    q2 = X[2];
    q3 = X[3];

    q0_2 = q0 * q0;
    q1_2 = q1 * q1;
    q2_2 = q2 * q2;
    q3_2 = q3 * q3;
    
    Y[0] =  (2*q1*q3 -2*q0*q2) * IMU_ACC_Z0;

    Y[1] =  (2*q2*q3 +2*q0*q1) * IMU_ACC_Z0;

    Y[2] =  (+(q0_2) -(q1_2) -(q2_2) +(q3_2)) * IMU_ACC_Z0;
    
    Y[3] =  (+(q0_2)+(q1_2)-(q2_2)-(q3_2)) * IMU_MAG_B0[0]
            +(2*(q1*q2+q0*q3)) * IMU_MAG_B0[1]
            +(2*(q1*q3-q0*q2)) * IMU_MAG_B0[2];

    Y[4] =  (2*(q1*q2-q0*q3)) * IMU_MAG_B0[0]
            +(+(q0_2)-(q1_2)+(q2_2)-(q3_2)) * IMU_MAG_B0[1]
            +(2*(q2*q3+q0*q1)) * IMU_MAG_B0[2];

    Y[5] =  (2*(q1*q3+q0*q2)) * IMU_MAG_B0[0]
            +(2*(q2*q3-q0*q1)) * IMU_MAG_B0[1]
            +(+(q0_2)-(q1_2)-(q2_2)+(q3_2)) * IMU_MAG_B0[2];
    
    return true;
}
    bool bCalcJacobianF (float* F, float* X, float* U){
    /* In Main_bUpdateNonlinearX():
     *  q0 = q0 + q0_dot * dT;
     *  q1 = q1 + q1_dot * dT;
     *  q2 = q2 + q2_dot * dT;
     *  q3 = q3 + q3_dot * dT;
     */
    float p, q, r;

    p = U[0];
    q = U[1];
    r = U[2];

    F[0*4+0] =  1.000;
    F[1*4+0] =  0.5*p * SS_DT;
    F[2*4+0] =  0.5*q * SS_DT;
    F[3*4+0] =  0.5*r * SS_DT;

    F[0*4+1] = -0.5*p * SS_DT;
    F[1*4+1] =  1.000;
    F[2*4+1] = -0.5*r * SS_DT;
    F[3*4+1] =  0.5*q * SS_DT;

    F[0*4+2] = -0.5*q * SS_DT;
    F[1*4+2] =  0.5*r * SS_DT;
    F[2*4+2] =  1.000;
    F[3*4+2] = -0.5*p * SS_DT;

    F[0*4+3] = -0.5*r * SS_DT;
    F[1*4+3] = -0.5*q * SS_DT;
    F[2*4+3] =  0.5*p * SS_DT;
    F[3*4+3] =  1.000;
    
    return true;
}
    bool bCalcJacobianH (float* H, float* X, float* U){
    /* In Main_bUpdateNonlinearY():
     * 
     * The measurement output is the gravitational and magnetic projection to the body:
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                    2*(q1*q2+q0*q3),                    2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3), (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                    2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                    2*(q2*q3-q0*q1), (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     * 
     *  G_proj_sens = DCM * [0 0 -g]            --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    float q0, q1, q2, q3;

    q0 = X[0];
    q1 = X[1];
    q2 = X[2];
    q3 = X[3];
    
    H[0*4+0] = -2*q2 * IMU_ACC_Z0;
    H[1*4+0] = +2*q1 * IMU_ACC_Z0;
    H[2*4+0] = +2*q0 * IMU_ACC_Z0;
    H[3*4+0] =  2*q0*IMU_MAG_B0[0] + 2*q3*IMU_MAG_B0[1] - 2*q2*IMU_MAG_B0[2];
    H[4*4+0] = -2*q3*IMU_MAG_B0[0] + 2*q0*IMU_MAG_B0[1] + 2*q1*IMU_MAG_B0[2];
    H[5*4+0] =  2*q2*IMU_MAG_B0[0] - 2*q1*IMU_MAG_B0[1] + 2*q0*IMU_MAG_B0[2];
    
    H[0*4+1] = +2*q3 * IMU_ACC_Z0;
    H[1*4+1] = +2*q0 * IMU_ACC_Z0;
    H[2*4+1] = -2*q1 * IMU_ACC_Z0;
    H[3*4+1] =  2*q1*IMU_MAG_B0[0]+2*q2*IMU_MAG_B0[1] + 2*q3*IMU_MAG_B0[2];
    H[4*4+1] =  2*q2*IMU_MAG_B0[0]-2*q1*IMU_MAG_B0[1] + 2*q0*IMU_MAG_B0[2];
    H[5*4+1] =  2*q3*IMU_MAG_B0[0]-2*q0*IMU_MAG_B0[1] - 2*q1*IMU_MAG_B0[2];
    
    H[0*4+2] = -2*q0 * IMU_ACC_Z0;
    H[1*4+2] = +2*q3 * IMU_ACC_Z0;
    H[2*4+2] = -2*q2 * IMU_ACC_Z0;
    H[3*4+2] = -2*q2*IMU_MAG_B0[0]+2*q1*IMU_MAG_B0[1] - 2*q0*IMU_MAG_B0[2];
    H[4*4+2] =  2*q1*IMU_MAG_B0[0]+2*q2*IMU_MAG_B0[1] + 2*q3*IMU_MAG_B0[2];
    H[5*4+2] =  2*q0*IMU_MAG_B0[0]+2*q3*IMU_MAG_B0[1] - 2*q2*IMU_MAG_B0[2];
    
    H[0*4+3] = +2*q1 * IMU_ACC_Z0;
    H[1*4+3] = +2*q2 * IMU_ACC_Z0;
    H[2*4+3] = +2*q3 * IMU_ACC_Z0;
    H[3*4+3] = -2*q3*IMU_MAG_B0[0]+2*q0*IMU_MAG_B0[1] + 2*q1*IMU_MAG_B0[2];
    H[4*4+3] = -2*q0*IMU_MAG_B0[0]-2*q3*IMU_MAG_B0[1] + 2*q2*IMU_MAG_B0[2];
    H[5*4+3] =  2*q1*IMU_MAG_B0[0]+2*q2*IMU_MAG_B0[1] + 2*q3*IMU_MAG_B0[2];
    
    return true;
}

private:
    float X_Est[SS_X_LEN*1];
    float P[SS_X_LEN*SS_X_LEN];
    float F[SS_X_LEN*SS_X_LEN];
    float H[SS_Z_LEN*SS_X_LEN];
    float Y_Est[SS_Z_LEN*1];
    float Err[SS_Z_LEN*1];
    float Q[SS_X_LEN*SS_X_LEN];
    float R[SS_Z_LEN*SS_Z_LEN];
    float S[SS_Z_LEN*SS_Z_LEN];
    float Gain[SS_X_LEN*SS_Z_LEN];
};

#endif // EKF_H
