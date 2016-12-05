////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#ifndef _RTFUSIONKALMAN4_H
#define	_RTFUSIONKALMAN4_H

#include "RTFusion.h"

class RTFusionKalman4 : public RTFusion
{
public:
    RTFusionKalman4();
    ~RTFusionKalman4();

    //  fusionType returns the type code of the fusion algorithm

    virtual int fusionType() { return RTFUSION_TYPE_KALMANSTATE4; }

    //  reset() resets the kalman state but keeps any setting changes (such as enables)

    void reset();

    //  newIMUData() should be called for subsequent updates
    //  deltaTime is in units of seconds

    void newIMUData(RTIMU_DATA& data, const RTIMUSettings *settings);

    //  the following two functions can be called to customize the covariance matrices

    void setQMatrix(RTMatrix4x4 Q) {  m_Q = Q; reset();}
    void setRkMatrix(RTMatrix4x4 Rk) { m_Rk = Rk; reset();}

protected:
    RTVector3 m_gyro;                                       // current gyro sample
    RTVector3 m_accel;                                      // current accel sample
    RTVector3 m_compass;                                    // current compass sample
    RTQuaternion m_rotationDelta;                           // amount by which measured state differs from predicted
    RTQuaternion m_rotationPower;                           // delta raised to the appopriate power
    RTVector3 m_rotationUnitVector;                         // the vector part of the rotation delta

private:
    virtual void gyroBiasInit(float samplerate);
    virtual void handleGyroBias(RTIMU_DATA& imuData, RTIMUSettings *settings);
    RTFLOAT m_gyroLearningAlpha;                            // gyro bias rapid learning rate
    RTFLOAT m_gyroContinuousAlpha;                          // gyro bias continuous (slow) learning rate
    RTFLOAT m_gyroSampleRate;
    int m_gyroSampleCount;                                  // number of gyro samples used
    RTVector3 m_previousAccel;                              // previous step accel for gyro learning

    void predict();
    void update();

    RTFLOAT m_timeDelta;                                    // time between predictions

    RTQuaternion m_stateQ;									// quaternion state vector
    RTQuaternion m_stateQError;                             // difference between stateQ and measuredQ

    RTMatrix4x4 m_Kk;                                       // the Kalman gain matrix
    RTMatrix4x4 m_Pkk_1;                                    // the predicted estimated covariance matrix
    RTMatrix4x4 m_Pkk;                                      // the updated estimated covariance matrix
    RTMatrix4x4 m_PDot;                                     // the derivative of the covariance matrix
    RTMatrix4x4 m_Q;                                        // process noise covariance
    RTMatrix4x4 m_Fk;                                       // the state transition matrix
    RTMatrix4x4 m_FkTranspose;                              // the state transition matrix transposed
    RTMatrix4x4 m_Rk;                                       // the measurement noise covariance

    //  Note: SInce Hk ends up being the identity matrix, these are omitted

//    RTMatrix4x4 m_Hk;                                     // map from state to measurement
//    RTMatrix4x4> m_HkTranspose;                           // transpose of map
};

#endif // _RTFUSIONKALMAN4_H
