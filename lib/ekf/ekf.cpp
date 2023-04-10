/*************************************************************************************************************
 *  Class for Discrete Extended Kalman Filter
 *  The system to be estimated is defined as a discrete nonlinear dynamic dystem:
 *              x(k) = f[x(k-1), u(k-1)] + v(k)     ; x = Nx1,    u = Mx1
 *              y(k) = h[x(k)] + n(k)               ; y = Zx1
 *
 *        Where:
 *          x(k) : State Variable at time-k                          : Nx1
 *          y(k) : Measured output at time-k                         : Zx1
 *          u(k) : System input at time-k                            : Mx1
 *          v(k) : Process noise, AWGN assumed, w/ covariance Qn     : Nx1
 *          n(k) : Measurement noise, AWGN assumed, w/ covariance Rn : Nx1
 *
 *          f(..), h(..) is a nonlinear transformation of the system to be estimated.
 *
 ***************************************************************************************************
 *      Extended Kalman Filter algorithm:
 *          Initialization:
 *              x(k=0|k=0) = Expected value of x at time-0 (i.e. x(k=0)), typically set to zero.
 *              P(k=0|k=0) = Identity matrix * covariant(P(k=0)), typically initialized with some 
 *                            big number.
 *              Q, R       = Covariance matrices of process & measurement. As this implementation 
 *                            the noise as AWGN (and same value for every variable), this is set
 *                            to Q=diag(QInit,...,QInit) and R=diag(RInit,...,RInit).
 * 
 * 
 *          EKF Calculation (every sampling time):
 *              Calculate the Jacobian matrix of f (i.e. F):
 *                  F = d(f(..))/dx |x(k-1|k-1),u(k-1)                               ...{EKF_1}
 * 
 *              Predict x(k) through nonlinear function f:
 *                  x(k|k-1) = f[x(k-1|k-1), u(k-1)]                                 ...{EKF_2}
 * 
 *              Predict P(k) using linearized f (i.e. F):
 *                  P(k|k-1)  = F*P(k-1|k-1)*F' + Q                                  ...{EKF_3}
 * 
 *              Calculate the Jacobian matrix of h (i.e. C):
 *                  C = d(h(..))/dx |x(k|k-1)                                        ...{EKF_4}
 * 
 *              Predict residual covariance S using linearized h (i.e. H):
 *                  S       = C*P(k|k-1)*C' + R                                      ...{EKF_5}
 * 
 *              Calculate the kalman gain:
 *                  K       = P(k|k-1)*C'*(S^-1)                                     ...{EKF_6}
 * 
 *              Correct x(k) using kalman gain:
 *                  x(k|k) = x(k|k-1) + K*[y(k) - h(x(k|k-1))]                       ...{EKF_7}
 * 
 *              Correct P(k) using kalman gain:
 *                  P(k|k)  = (I - K*C)*P(k|k-1)                                     ...{EKF_8}
 * 
 * 
 *        *Additional Information:
 *              - Pada contoh di atas X~(k=0|k=0) = [0]. Untuk mempercepat konvergensi bisa
 *                  digunakan informasi plant-spesific. Misal pada implementasi Kalman Filter
 *                  untuk sensor IMU (Inertial measurement unit) dengan X = [quaternion], dengan
 *                  asumsi IMU awalnya menghadap ke atas tanpa rotasi: X~(k=0|k=0) = [1, 0, 0, 0]'
 * 
 * 
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/
#include "ekf.h"
void matmul(float* result,float* A,float* B,uint8_t Arow,uint8_t AcolBrow, uint8_t Bcol){
    for(uint8_t i=0;i<Arow;i++){
        for(uint8_t j=0;j<Bcol;j++){
            result[i*Bcol+j]=0;
            for(uint8_t k=0;k<AcolBrow;k++){
                result[i*Bcol+j]+=A[i*AcolBrow+k]*B[k*AcolBrow+j];
            }
        }
    }
}
void matmulBT(float* result,float* A,float* B,uint8_t Arow,uint8_t AcolBcol, uint8_t Brow){
    float Bc[36];
    uint8_t n = Brow*AcolBcol;
    for(uint8_t i=0;i<n;i++)
        Bc[i] = B[i];
    for(uint8_t i=0;i<Arow;i++){
        for(uint8_t j=0;j<Brow;j++){
            result[i*Brow+j]=0;
            for(uint8_t k=0;k<AcolBcol;k++){
                result[i*Brow+j]+=A[i*AcolBcol+k]*Bc[k+j*AcolBcol];
            }
        }
    }
}
float determinant(float* mat, uint8_t n) {

   for(uint8_t j = 0; j < n; ++j) {
      bool found = false;
      for(uint8_t i = j; i < n; ++i) {
         if(mat[i*n+j]!=0) {
            if ( i != j )
            {
                for(uint8_t k=0;k<n;k++){
                    float t=mat[j*n+k];     
                    mat[j*n+k]=mat[i*n+k];
                    mat[i+n*k]=t;
                }
            }
            found = true;
            break;
         }
      }

      if(!found) {
         return 0;
      }

      for(uint8_t i = j + 1; i < n; ++i) {
         while(true) {
            float del = mat[i*n+j] / mat[j*n+j];
            for (uint8_t k = j; k < n; ++k) {
               mat[i*n+k] -= del * mat[j*n+k];
            }
            if (mat[i*n+j] == 0)
            {
               break;
            }
            else
            {
                for(uint8_t k=0;k<n;k++){
                    float t=mat[j*n+k];     
                    mat[j*n+k]=mat[i*n+k];
                    mat[i+n*k]=t;
                }
            }
         }
      }
   }
   float res = 1.0;

   for(int i = 0; i < n; ++i) {
      res *= mat[i*n+i];
   }
   return res;
}
void matInv(float* res,float* mat, uint8_t n){
    float tmp[36];
    int8_t i,j;
    for(i=0;i<n*n;i++){
        if(i%(n+1) == 0) res[i] = 1;
        else res[i] = 0;
        tmp[i] = mat[i];
    }
    /* Gauss Elimination... */
    for (j = 0; j < n-1; j++) {
        for (i = j+1; i < n; i++) {

            float tf = tmp[i*n+j] / tmp[j*n+j];

            for (int8_t k = 0; k < n; k++) {
                tmp[i*n+k] -= (tmp[j*n+k] * tf);
                res[i*n+k] -= (res[j*n+k] * tf);
                
                if(abs(tmp[i*n+k])<float_prec_ZERO) tmp[i*n+k] = 0;
                if(abs(res[i*n+k])<float_prec_ZERO) res[i*n+k] = 0;
            }

        }
    }
        
    /* Jordan... */
    for (j = n-1; j > 0; j--) {
        for (i = j-1; i >= 0; i--) {
            if(tmp[j*n+j]==0.0){
                return;
            }
            float tf = tmp[i*n+j] / tmp[j*n+j];
            tmp[i*n+j] -= (tmp[j*n+j] * tf);
            if(abs(tmp[i*n+j])<float_prec_ZERO) tmp[i*n+j] = 0;

            for (int8_t k = n-1; k >= 0; k--) {
                res[i*n+k] -= (res[j*n+k] * tf);
                if(abs(res[i*n+k])<float_prec_ZERO) res[i*n+k] = 0;
            }
        }
    }
    /* Normalization */
    for (i = 0; i < n; i++) {

        float tf = tmp[i*n+i];
        tmp[i*n+i] = 1.0;

        for (j = 0; j < n; j++) {
            res[i*n+j] /= tf;
        }
    }
}

EKF::EKF(float* XInit, float* P, float* Q, float* R)
{
    /* Initialization:
     *  x(k=0|k=0)  = Expected value of x at time-0 (i.e. x(k=0)), typically set to zero.
     *  P (k=0|k=0) = Identity matrix * covariant(P(k=0)), typically initialized with some 
     *                 big number.
     *  Q, R        = Covariance matrices of process & measurement. As this implementation 
     *                 the noise as AWGN (and same value for every variable), this is set
     *                 to Q=diag(QInit,...,QInit) and R=diag(RInit,...,RInit).
     */
    for(uint8_t i=0;i<4;i++){
        this->X_Est[i] = XInit[i];
    }
    for(uint8_t i=0;i<16;i++){
        this->P[i] = P[i];
        this->Q[i] = Q[i];
    }
    for(uint8_t i=0;i<36;i++){
        this->R[i] = R[i];
    }
}

void EKF::vReset(float* XInit, float* P, float* Q, float* R)
{
    for(uint8_t i=0;i<4;i++){
        this->X_Est[i] = XInit[i];
    }
    for(uint8_t i=0;i<16;i++){
        this->P[i] = P[i];
        this->Q[i] = Q[i];
    }
    for(uint8_t i=0;i<36;i++){
        this->R[i] = R[i];
    }
}

bool EKF::bUpdate(float* gyro,float* acc,float* mag){
    float Y[SS_Z_LEN];
    float accSize = sqrt(sq(acc[0])+sq(acc[1])+sq(acc[2]));
    float magSize = sqrt(sq(mag[0])+sq(mag[1])+sq(mag[2]));
    for(uint8_t i=0;i<3;i++){
        Y[i]=acc[i]/accSize;
        Y[i+3]=mag[i]/magSize;
    }
    return bUpdate(Y,gyro);
}
bool EKF::bUpdate(float* Y, float* U)
{
    float T[36];
    /* Run once every sampling time */
    
    /* =============== Calculate the Jacobian matrix of f (i.e. F) =============== */
    /* F = d(f(..))/dx |x(k-1|k-1),u(k-1)                               ...{EKF_1} */

    if (!bCalcJacobianF(F, X_Est, U)) {
        return false;
    }
    
    /* =========================== Prediction of x & P =========================== */
    /* x(k|k-1) = f[x(k-1|k-1), u(k-1)]                                 ...{EKF_2} */
    if (!bNonlinearUpdateX(X_Est, X_Est, U)) {
        return false;
    }
    /* P(k|k-1)  = F*P(k-1|k-1)*F' + Q                                  ...{EKF_3} */
    matmul(T,F,P,4,4,4);
    matmulBT(P,T,F,4,4,4);
    for(uint8_t i=0;i<16;i++){
        P[i]+=Q[i];
    }
    
    /* =============== Calculate the Jacobian matrix of h (i.e. H) =============== */
    /* H = d(h(..))/dx |x(k|k-1)                                        ...{EKF_4} */
    if (!bCalcJacobianH(H, X_Est, U)) {
        return false;
    }
    /* =========================== Correction of x & P =========================== */
    /* S       = H*P(k|k-1)*H' + R                                      ...{EKF_5} */
    matmul(T,H,P,6,4,4);
    matmulBT(S,T,H,6,4,6);
    for(uint8_t i=0;i<36;i++) S[i]+=R[i];
    matmulBT(T,P,H,4,4,6);
    float iS[36];
    matInv(iS,S,6);
    matmul(Gain,T,iS,4,6,6);
    for(uint8_t i=0;i<24;i++) Serial.printf("%.1f|",Gain[i]);
    // float Ssize=determinant(S,6);
    // Serial.printf("Ssize : %f",Ssize);
    // if(Ssize == 0.0f) return false;
    // for(uint8_t i=0;i<24;i++) Gain[i]/=Ssize;
    /* x(k|k) = x(k|k-1) + K*[y(k) - h(x(k|k-1))]                       ...{EKF_7} */
    if (!bNonlinearUpdateY(Y_Est, X_Est, U)) {
        return false;
    }
    // Serial.printf("\nYd : ");
    for(uint8_t i=0;i<6;i++){
        T[i]=Y[i]-Y_Est[i];
        // Serial.printf("%f/",T[i]);
    }
    float Xt[4];
    matmul(Xt,Gain,T,4,6,1);
    // float XeSize=0;
    for(uint8_t i=0;i<4;i++){
        X_Est[i]+=Xt[i];
        // XeSize+=sq(X_Est[i]);
    }
    // XeSize = sqrt(XeSize);
    // for(uint8_t i=0;i<4;i++){
    //     X_Est[i]/=XeSize;
    // }
    /* P(k|k)  = (I - K*H)*P(k|k-1)                                     ...{EKF_8} */
    matmul(T,Gain,H,4,6,4);
    for(uint8_t i=0;i<16;i++) T[i]=-T[i];
    for(uint8_t i=0;i<16;i+=5) T[i]++;
    float T2[16];
    matmul(T2,T,P,4,4,4);
    for(uint8_t i=0;i<16;i++) P[i]=T2[i];
    
    return true;
}
