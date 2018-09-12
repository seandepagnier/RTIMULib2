////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech
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


#include "RTIMU.h"
#include "RTFusionKalman4.h"
#include "RTFusionRTQF.h"
#include "FusionMadgwick.h"
#include "FusionMahony.h"

#include "RTIMUNull.h"
#include "RTIMUMPU9150.h"
#include "RTIMUMPU925x.h"
#include "RTIMUGD20HM303D.h"
#include "RTIMUGD20M303DLHC.h"
#include "RTIMUGD20HM303DLHC.h"
#include "RTIMULSM9DS0.h"
#include "RTIMULSM9DS1.h"
#include "RTIMUBMX055.h"
#include "RTIMUBNO055.h"
#include "RTIMULSM6DS33LIS3MDL.h"
#include "RTIMUHMC5883LADXL345.h"

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

//  this sets the min range (max - min) of values to trigger runtime mag calibration

#define RTIMU_RUNTIME_MAGCAL_RANGE  30

//  Axis rotation arrays

float RTIMU::m_axisRotation[RTIMU_AXIS_ROTATION_COUNT][9] = {
    {1, 0, 0, 0, 1, 0, 0, 0, 1},                    // RTIMU_XNORTH_YEAST
    {0, -1, 0, 1, 0, 0, 0, 0, 1},                   // RTIMU_XEAST_YSOUTH
    {-1, 0, 0, 0, -1, 0, 0, 0, 1},                  // RTIMU_XSOUTH_YWEST
    {0, 1, 0, -1, 0, 0, 0, 0, 1},                   // RTIMU_XWEST_YNORTH

    {1, 0, 0, 0, -1, 0, 0, 0, -1},                  // RTIMU_XNORTH_YWEST
    {0, 1, 0, 1, 0, 0, 0, 0, -1},                   // RTIMU_XEAST_YNORTH
    {-1, 0, 0, 0, 1, 0, 0, 0, -1},                  // RTIMU_XSOUTH_YEAST
    {0, -1, 0, -1, 0, 0, 0, 0, -1},                 // RTIMU_XWEST_YSOUTH

    {0, 1, 0, 0, 0, -1, -1, 0, 0},                  // RTIMU_XUP_YNORTH
    {0, 0, 1, 0, 1, 0, -1, 0, 0},                   // RTIMU_XUP_YEAST
    {0, -1, 0, 0, 0, 1, -1, 0, 0},                  // RTIMU_XUP_YSOUTH
    {0, 0, -1, 0, -1, 0, -1, 0, 0},                 // RTIMU_XUP_YWEST

    {0, 1, 0, 0, 0, 1, 1, 0, 0},                    // RTIMU_XDOWN_YNORTH
    {0, 0, -1, 0, 1, 0, 1, 0, 0},                   // RTIMU_XDOWN_YEAST
    {0, -1, 0, 0, 0, -1, 1, 0, 0},                  // RTIMU_XDOWN_YSOUTH
    {0, 0, 1, 0, -1, 0, 1, 0, 0},                   // RTIMU_XDOWN_YWEST

    {1, 0, 0, 0, 0, 1, 0, -1, 0},                   // RTIMU_XNORTH_YUP
    {0, 0, -1, 1, 0, 0, 0, -1, 0},                  // RTIMU_XEAST_YUP
    {-1, 0, 0, 0, 0, -1, 0, -1, 0},                 // RTIMU_XSOUTH_YUP
    {0, 0, 1, -1, 0, 0, 0, -1, 0},                  // RTIMU_XWEST_YUP

    {1, 0, 0, 0, 0, -1, 0, 1, 0},                   // RTIMU_XNORTH_YDOWN
    {0, 0, 1, 1, 0, 0, 0, 1, 0},                    // RTIMU_XEAST_YDOWN
    {-1, 0, 0, 0, 0, 1, 0, 1, 0},                   // RTIMU_XSOUTH_YDOWN
    {0, 0, -1, -1, 0, 0, 0, 1, 0}                   // RTIMU_XWEST_YDOWN
};

RTIMU *RTIMU::createIMU(RTIMUSettings *settings)
{
    switch (settings->m_imuType) {
    case RTIMU_TYPE_MPU9150:
        return new RTIMUMPU9150(settings);

    case RTIMU_TYPE_GD20HM303D:
        return new RTIMUGD20HM303D(settings);

    case RTIMU_TYPE_GD20M303DLHC:
        return new RTIMUGD20M303DLHC(settings);

    case RTIMU_TYPE_LSM9DS0:
         return new RTIMULSM9DS0(settings);
    case RTIMU_TYPE_LSM9DS1:
        return new RTIMULSM9DS1(settings);

    case RTIMU_TYPE_MPU925x:
        return new RTIMUMPU925x(settings);

    case RTIMU_TYPE_GD20HM303DLHC:
        return new RTIMUGD20HM303DLHC(settings);

    case RTIMU_TYPE_BMX055:
        return new RTIMUBMX055(settings);

    case RTIMU_TYPE_BNO055:
        return new RTIMUBNO055(settings);

    case RTIMU_TYPE_LSM6DS33LIS3MDL:
        return new RTIMULSM6DS33LIS3MDL(settings);

    case RTIMU_TYPE_HMC5883LADXL345:
	return new RTIMU5883L(settings);


    case RTIMU_TYPE_NULL:
        return new RTIMUNull(settings);

    case RTIMU_TYPE_AUTODISCOVER:
    default:
        if (settings->discoverIMU(settings->m_imuType, settings->m_busIsI2C,
                                  settings->m_I2CSlaveAddress)) {
            settings->saveSettings();
            return RTIMU::createIMU(settings);
        }
        return new RTIMUNull(settings);
    }
}


RTIMU::RTIMU(RTIMUSettings *settings)
{
    m_settings = settings;

    m_compassCalibrationMode = false;
    m_accelCalibrationMode = false;

    m_runtimeMagCalValid = false;

    for (int i = 0; i < 3; i++) {
        m_runtimeMagCalMax[i] = -1000;
        m_runtimeMagCalMin[i] = 1000;
    }

    switch (m_settings->m_fusionType) {
    case RTFUSION_TYPE_KALMANSTATE4:
        m_fusion = new RTFusionKalman4();
        break;

    case RTFUSION_TYPE_RTQF:
        m_fusion = new RTFusionRTQF();
        break;

    case RTFUSION_TYPE_MADGWICK:
        m_fusion = new FusionMadgwick();
        break;

    case RTFUSION_TYPE_MAHONY:
        m_fusion = new FusionMahony();
        break;

    default:
        m_fusion = new RTFusion();
        break;
    }
    HAL_INFO1("Using fusion algorithm %s\n", RTFusion::fusionName(m_settings->m_fusionType));
}

RTIMU::~RTIMU()
{
    delete m_fusion;
    m_fusion = NULL;
}

void RTIMU::setCalibrationData()
{
    float maxDelta = -1;
    float delta;

    if (m_settings->m_compassCalValid) {
        //  find biggest range

        for (int i = 0; i < 3; i++) {
            if ((m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i)) > maxDelta)
                maxDelta = m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i);
        }
        if (maxDelta < 0) {
            HAL_ERROR("Error in compass calibration data\n");
            return;
        }
        maxDelta /= 2.0f;                                       // this is the max +/- range

        for (int i = 0; i < 3; i++) {
            delta = (m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i)) / 2.0f;
            m_compassCalScale[i] = maxDelta / delta;            // makes everything the same range
            m_compassCalOffset[i] = (m_settings->m_compassCalMax.data(i) + m_settings->m_compassCalMin.data(i)) / 2.0f;
        }
    }

    if (m_settings->m_compassCalValid) {
        HAL_INFO("Using min/max compass calibration\n");
    } else {
        HAL_INFO("min/max compass calibration not in use\n");
    }

    if (m_settings->m_compassCalEllipsoidValid) {
        HAL_INFO("Using ellipsoid compass calibration\n");
    } else {
        HAL_INFO("Ellipsoid compass calibration not in use\n");
    }

    if (m_settings->m_accelCalValid) {
        HAL_INFO("Using accel calibration\n");
    } else {
        HAL_INFO("Accel calibration not in use\n");
    }
}


void RTIMU::gyroBiasInit()
{
    m_fusion->gyroBiasInit(m_sampleRate);
}

//  Note - code assumes that this is the first thing called after axis swapping
//  for each specific IMU chip has occurred.

void RTIMU::handleGyroBias()
{
    m_fusion->handleGyroBias(m_imuData, m_settings);
}

void RTIMU::calibrateAverageCompass()
{
#if 0 // this method is unreliable
    //  see if need to do runtime mag calibration (i.e. no stored calibration data)
    if (!m_compassCalibrationMode && !m_settings->m_compassCalValid) {
        // try runtime calibration
        bool changed = false;

        // see if there is a new max or min

        if (m_runtimeMagCalMax[0] < m_imuData.compass.x()) {
            m_runtimeMagCalMax[0] = m_imuData.compass.x();
            changed = true;
        }
        if (m_runtimeMagCalMax[1] < m_imuData.compass.y()) {
            m_runtimeMagCalMax[1] = m_imuData.compass.y();
            changed = true;
        }
        if (m_runtimeMagCalMax[2] < m_imuData.compass.z()) {
            m_runtimeMagCalMax[2] = m_imuData.compass.z();
            changed = true;
        }

        if (m_runtimeMagCalMin[0] > m_imuData.compass.x()) {
            m_runtimeMagCalMin[0] = m_imuData.compass.x();
            changed = true;
        }
        if (m_runtimeMagCalMin[1] > m_imuData.compass.y()) {
            m_runtimeMagCalMin[1] = m_imuData.compass.y();
            changed = true;
        }
        if (m_runtimeMagCalMin[2] > m_imuData.compass.z()) {
            m_runtimeMagCalMin[2] = m_imuData.compass.z();
            changed = true;
        }

        //  now see if ranges are sufficient

        if (changed) {

            float delta;

            if (!m_runtimeMagCalValid) {
                m_runtimeMagCalValid = true;

                for (int i = 0; i < 3; i++)
                {
                    delta = m_runtimeMagCalMax[i] - m_runtimeMagCalMin[i];
                    if ((delta < RTIMU_RUNTIME_MAGCAL_RANGE) || (m_runtimeMagCalMin[i] > 0) || (m_runtimeMagCalMax[i] < 0))
                    {
                        m_runtimeMagCalValid = false;
                        break;
                    }
                }
            }

            //  find biggest range and scale to that

            if (m_runtimeMagCalValid) {
                float magMaxDelta = -1;

                for (int i = 0; i < 3; i++) {
                    if ((m_runtimeMagCalMax[i] - m_runtimeMagCalMin[i]) > magMaxDelta)
                    {
                        magMaxDelta = m_runtimeMagCalMax[i] - m_runtimeMagCalMin[i];
                    }
                }

                // adjust for + and - range

                magMaxDelta /= 2.0;

                for (int i = 0; i < 3; i++)
                {
                    delta = (m_runtimeMagCalMax[i] - m_runtimeMagCalMin[i]) / 2.0;
                    m_compassCalScale[i] = magMaxDelta / delta;
                    m_compassCalOffset[i] = (m_runtimeMagCalMax[i] + m_runtimeMagCalMin[i]) / 2.0;
                }
            }
        }
    }
#endif


#if 0 // this doesn't work, fit ellipsoid to sigma points instead
    //  update running average
    m_compassAverage.setX(m_imuData.compass.x() * COMPASS_ALPHA + m_compassAverage.x() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setY(m_imuData.compass.y() * COMPASS_ALPHA + m_compassAverage.y() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setZ(m_imuData.compass.z() * COMPASS_ALPHA + m_compassAverage.z() * (1.0 - COMPASS_ALPHA));
    m_imuData.compass = m_compassAverage;
#endif
}

void RTIMU::calibrateAccel()
{
    return; // unreliable method
    
    if (!getAccelCalibrationValid())
        return;

    if (m_imuData.accel.x() >= 0)
        m_imuData.accel.setX(m_imuData.accel.x() / m_settings->m_accelCalMax.x());
    else
        m_imuData.accel.setX(m_imuData.accel.x() / -m_settings->m_accelCalMin.x());

    if (m_imuData.accel.y() >= 0)
        m_imuData.accel.setY(m_imuData.accel.y() / m_settings->m_accelCalMax.y());
    else
        m_imuData.accel.setY(m_imuData.accel.y() / -m_settings->m_accelCalMin.y());

    if (m_imuData.accel.z() >= 0)
        m_imuData.accel.setZ(m_imuData.accel.z() / m_settings->m_accelCalMax.z());
    else
        m_imuData.accel.setZ(m_imuData.accel.z() / -m_settings->m_accelCalMin.z());
}

RTVector3 RTIMU::CalibratedAccel()
{
    RTVector3 accel = m_imuData.accel;
    // apply accelerometer calibration
    if (getAccelCalibrationValid()) {
        RTVector3 min = m_settings->m_accelCalMin, max = m_settings->m_accelCalMax;
        float b[3] = {(min.x() + max.x()) / 2, (min.y() + max.y()) / 2, (min.z() + max.z()) / 2};
        float s[3] = {(max.x() - min.x()) / 2, (max.y() - min.y()) / 2, (max.z() - min.z()) / 2};
        RTVector3 a = accel;
        accel.setX((accel.x() - b[0]) / s[0]);
        accel.setY((accel.y() - b[1]) / s[1]);
        accel.setZ((accel.z() - b[2]) / s[2]);
        //printf("applied accel %f %f %f %f %f %f \n", a.x(), a.y(), a.z(), accel.x(), accel.y(), accel.z());
    }
    return accel;
}

void RTIMU::updateFusion()
{
    RTIMU_DATA imuData = m_imuData;

    imuData.accel = CalibratedAccel();
    
        // apply ellipsoid parameters
    if (getCompassCalibrationValid() || getRuntimeCompassCalibrationValid()) {
        imuData.compass.setX((imuData.compass.x() - m_compassCalOffset[0]) * m_compassCalScale[0]);
        imuData.compass.setY((imuData.compass.y() - m_compassCalOffset[1]) * m_compassCalScale[1]);
        imuData.compass.setZ((imuData.compass.z() - m_compassCalOffset[2]) * m_compassCalScale[2]);
    }
    if (m_settings->m_compassCalEllipsoidValid) {
        RTVector3 ev = imuData.compass;
        ev -= m_settings->m_compassCalEllipsoidOffset;

        imuData.compass.setX(ev.x() * m_settings->m_compassCalEllipsoidCorr[0][0] +
                             ev.y() * m_settings->m_compassCalEllipsoidCorr[0][1] +
                             ev.z() * m_settings->m_compassCalEllipsoidCorr[0][2]);

        imuData.compass.setY(ev.x() * m_settings->m_compassCalEllipsoidCorr[1][0] +
                             ev.y() * m_settings->m_compassCalEllipsoidCorr[1][1] +
                             ev.z() * m_settings->m_compassCalEllipsoidCorr[1][2]);

        imuData.compass.setZ(ev.x() * m_settings->m_compassCalEllipsoidCorr[2][0] +
                             ev.y() * m_settings->m_compassCalEllipsoidCorr[2][1] +
                             ev.z() * m_settings->m_compassCalEllipsoidCorr[2][2]);
    }

    if ((m_settings->m_axisRotation > 0) && (m_settings->m_axisRotation < RTIMU_AXIS_ROTATION_COUNT)) {
        // need to do an axis rotation
        float *matrix = m_axisRotation[m_settings->m_axisRotation];
        RTIMU_DATA tempIMU = imuData;

        // do new x value
        if (matrix[0] != 0) {
            imuData.gyro.setX(tempIMU.gyro.x() * matrix[0]);
            imuData.accel.setX(tempIMU.accel.x() * matrix[0]);
            imuData.compass.setX(tempIMU.compass.x() * matrix[0]);
        } else if (matrix[1] != 0) {
            imuData.gyro.setX(tempIMU.gyro.y() * matrix[1]);
            imuData.accel.setX(tempIMU.accel.y() * matrix[1]);
            imuData.compass.setX(tempIMU.compass.y() * matrix[1]);
        } else if (matrix[2] != 0) {
            imuData.gyro.setX(tempIMU.gyro.z() * matrix[2]);
            imuData.accel.setX(tempIMU.accel.z() * matrix[2]);
            imuData.compass.setX(tempIMU.compass.z() * matrix[2]);
        }

        // do new y value
        if (matrix[3] != 0) {
            imuData.gyro.setY(tempIMU.gyro.x() * matrix[3]);
            imuData.accel.setY(tempIMU.accel.x() * matrix[3]);
            imuData.compass.setY(tempIMU.compass.x() * matrix[3]);
        } else if (matrix[4] != 0) {
            imuData.gyro.setY(tempIMU.gyro.y() * matrix[4]);
            imuData.accel.setY(tempIMU.accel.y() * matrix[4]);
            imuData.compass.setY(tempIMU.compass.y() * matrix[4]);
        } else if (matrix[5] != 0) {
            imuData.gyro.setY(tempIMU.gyro.z() * matrix[5]);
            imuData.accel.setY(tempIMU.accel.z() * matrix[5]);
            imuData.compass.setY(tempIMU.compass.z() * matrix[5]);
        }

        // do new z value
        if (matrix[6] != 0) {
            imuData.gyro.setZ(tempIMU.gyro.x() * matrix[6]);
            imuData.accel.setZ(tempIMU.accel.x() * matrix[6]);
            imuData.compass.setZ(tempIMU.compass.x() * matrix[6]);
        } else if (matrix[7] != 0) {
            imuData.gyro.setZ(tempIMU.gyro.y() * matrix[7]);
            imuData.accel.setZ(tempIMU.accel.y() * matrix[7]);
            imuData.compass.setZ(tempIMU.compass.y() * matrix[7]);
        } else if (matrix[8] != 0) {
            imuData.gyro.setZ(tempIMU.gyro.z() * matrix[8]);
            imuData.accel.setZ(tempIMU.accel.z() * matrix[8]);
            imuData.compass.setZ(tempIMU.compass.z() * matrix[8]);
        }
    }
    
    m_fusion->newIMUData(imuData, m_settings);

    m_imuData.fusionPoseValid = imuData.fusionPoseValid;
    m_imuData.fusionQPoseValid = imuData.fusionQPoseValid;
    m_imuData.fusionPose = imuData.fusionPose;
    m_imuData.fusionQPose = imuData.fusionQPose;
}

bool RTIMU::IMUGyroBiasValid()
{
    return m_settings->m_gyroBiasValid;
}

void RTIMU::setExtIMUData(RTFLOAT gx, RTFLOAT gy, RTFLOAT gz, RTFLOAT ax, RTFLOAT ay, RTFLOAT az,
                          RTFLOAT mx, RTFLOAT my, RTFLOAT mz, uint64_t timestamp)
{
     m_imuData.gyro.setX(gx);
     m_imuData.gyro.setY(gy);
     m_imuData.gyro.setZ(gz);
     m_imuData.accel.setX(ax);
     m_imuData.accel.setY(ay);
     m_imuData.accel.setZ(az);
     m_imuData.compass.setX(mx);
     m_imuData.compass.setY(my);
     m_imuData.compass.setZ(mz);
     m_imuData.timestamp = timestamp;
     updateFusion();
}
