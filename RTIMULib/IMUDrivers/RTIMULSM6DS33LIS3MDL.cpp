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

//  The LSM6DS33 + LIS3MDL driver code has been contributed by
//  Torsten Kurbad <github@tk-webart.de>

#include "RTIMULSM6DS33LIS3MDL.h"
#include "RTIMUSettings.h"

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

RTIMULSM6DS33LIS3MDL::RTIMULSM6DS33LIS3MDL(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
}

RTIMULSM6DS33LIS3MDL::~RTIMULSM6DS33LIS3MDL()
{
}

bool RTIMULSM6DS33LIS3MDL::IMUInit()
{
    unsigned char result;

    // set validity flags

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    //  configure IMU

    m_accelGyroSlaveAddr = m_settings->m_I2CSlaveAddress;

    uint8_t id = LIS3MDL_ID;
    // work out mag address
    if (m_settings->HALRead(LIS3MDL_ADDRESS0, LIS3MDL_WHO_AM_I, 1, &result, "")) {
        if (result == id) {
            m_compassSlaveAddr = LIS3MDL_ADDRESS0;
        }
    } else {
        m_compassSlaveAddr = LIS3MDL_ADDRESS1;
    }

    setCalibrationData();

    //  enable the I2C bus

    if (!m_settings->HALOpen())
        return false;


    //  Set up the accel/gyro
    if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM6DS33_WHO_AM_I, 1, &m_imuid, "Failed to read LSM6DS33 accel/gyro id"))
        return false;

    if (m_imuid != LSM6DS33_ID && m_imuid != LSM6DSL_ID && m_imuid != ISM330DHCX_ID) {
        HAL_ERROR1("Incorrect LSM6DS33 accel/gyro id %d\n", result);
        return false;
    }

    // reset device
    m_settings->HALWrite(m_accelGyroSlaveAddr, LSM6DS33_CTRL3_C, 0x01, "Failed to set LSM6DS33 gyro CTRL3_C");

    // block data update msb and lsb and register increment
    m_settings->HALWrite(m_accelGyroSlaveAddr, LSM6DS33_CTRL3_C, 0x44, "Failed to set LSM6DS33 gyro CTRL3_C");

    if (!setAccelCTRL1())
        return false;

    if (!setGyroCTRL2())
            return false;

    if (!setGyroCTRL7())
            return false;

    //  Set up the mag

    if (!m_settings->HALRead(m_compassSlaveAddr, LIS3MDL_WHO_AM_I, 1, &result, "Failed to read LIS3MDL mag id"))
        return false;

    if (result != LIS3MDL_ID) {
        HAL_ERROR1("Incorrect LIS3MDL mag id %d\n", result);
        return false;
    }

    if (!setCompassCTRL1())
        return false;

    if (!setCompassCTRL2())
        return false;

    if (!setCompassCTRL3())
        return false;

    if (!setCompassCTRL4())
        return false;

    if (!setCompassCTRL5())
        return false;

    gyroBiasInit();

    HAL_INFO("LSM6DS33LIS3MDL init complete\n");
    return true;
}

bool RTIMULSM6DS33LIS3MDL::setAccelCTRL1()
{
    unsigned char ctrl1;

    switch (m_settings->m_LSM6DS33LIS3MDLAccelSampleRate) {
    case LSM6DS33_ACCEL_SAMPLERATE_13:
        ctrl1 = 0x10;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_26:
        ctrl1 = 0x20;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_52:
        ctrl1 = 0x30;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_104:
        ctrl1 = 0x40;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_208:
        ctrl1 = 0x50;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_416:
        ctrl1 = 0x60;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_833:
        ctrl1 = 0x70;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_1660:
        ctrl1 = 0x80;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_3330:
        ctrl1 = 0x90;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_6660:
        ctrl1 = 0xa0;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 accel sample rate code %d\n", m_settings->m_LSM6DS33LIS3MDLAccelSampleRate);
        return false;
    }

    switch (m_settings->m_LSM6DS33LIS3MDLAccelFsr) {
    case LSM6DS33_ACCEL_FSR_2:
        m_accelScale = (RTFLOAT)0.000061;
        break;

    case LSM6DS33_ACCEL_FSR_4:
        ctrl1 |= 0x08;
        m_accelScale = (RTFLOAT)0.000122;
        break;

    case LSM6DS33_ACCEL_FSR_8:
        ctrl1 |= 0x0c;
        m_accelScale = (RTFLOAT)0.000244;
        break;

    case LSM6DS33_ACCEL_FSR_16:
        ctrl1 |= 0x04;
        m_accelScale = (RTFLOAT)0.000488;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 accel FSR code %d\n", m_settings->m_LSM6DS33LIS3MDLAccelFsr);
        return false;
    }

    switch (m_settings->m_LSM6DS33LIS3MDLAccelLpf) {
    case LSM6DS33_ACCEL_LPF_400:
        break;

    case LSM6DS33_ACCEL_LPF_200:
        ctrl1 |= 0x01;
        break;

    case LSM6DS33_ACCEL_LPF_100:
        ctrl1 |= 0x02;
        break;

    case LSM6DS33_ACCEL_LPF_50:
        ctrl1 |= 0x03;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 accel low pass filter code %d\n", m_settings->m_LSM6DS33LIS3MDLAccelLpf);
        return false;
    }


    m_settings->HALWrite(m_accelGyroSlaveAddr, LSM6DS33_CTRL8_XL, 0x40, "Failed to set LSM6DS33 accel CTRL8_XL");
    
    ctrl1 &= ~0x1;

    return m_settings->HALWrite(m_accelGyroSlaveAddr, LSM6DS33_CTRL1_XL, ctrl1, "Failed to set LSM6DS33 accel CTRL1_XL");
}

bool RTIMULSM6DS33LIS3MDL::setGyroCTRL2()
{
    unsigned char ctrl2;

    switch (m_settings->m_LSM6DS33LIS3MDLGyroSampleRate) {
    case LSM6DS33_GYRO_SAMPLERATE_13:
        ctrl2 = 0x10;
        m_sampleRate = 13;
        break;

    case LSM6DS33_GYRO_SAMPLERATE_26:
        ctrl2 = 0x20;
        m_sampleRate = 26;
        break;

    case LSM6DS33_GYRO_SAMPLERATE_52:
        ctrl2 = 0x30;
        m_sampleRate = 52;
        break;

    case LSM6DS33_GYRO_SAMPLERATE_104:
        ctrl2 = 0x40;
        m_sampleRate = 104;
        break;

    case LSM6DS33_GYRO_SAMPLERATE_208:
        ctrl2 = 0x50;
        m_sampleRate = 208;
        break;

    case LSM6DS33_GYRO_SAMPLERATE_416:
        ctrl2 = 0x60;
        m_sampleRate = 416;
        break;

    case LSM6DS33_GYRO_SAMPLERATE_833:
        ctrl2 = 0x70;
        m_sampleRate = 833;
        break;

    case LSM6DS33_GYRO_SAMPLERATE_1660:
        ctrl2 = 0x80;
        m_sampleRate = 1660;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 gyro sample rate code %d\n", m_settings->m_LSM6DS33LIS3MDLGyroSampleRate);
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_LSM6DS33LIS3MDLGyroFsr) {
    case LSM6DS33_GYRO_FSR_125:
        ctrl2 |= 0x02;
        m_gyroScale = (RTFLOAT)0.004375 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM6DS33_GYRO_FSR_245:
        m_gyroScale = (RTFLOAT)0.00875 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM6DS33_GYRO_FSR_500:
        ctrl2 |= 0x04;
        m_gyroScale = (RTFLOAT)0.0175 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM6DS33_GYRO_FSR_1000:
        ctrl2 |= 0x08;
        m_gyroScale = (RTFLOAT)0.035 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM6DS33_GYRO_FSR_2000:
        ctrl2 |= 0x0c;
        m_gyroScale = (RTFLOAT)0.07 * RTMATH_DEGREE_TO_RAD;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 gyro FSR code %d\n", m_settings->m_LSM6DS33LIS3MDLGyroFsr);
        return false;
    }
    
    return (m_settings->HALWrite(m_accelGyroSlaveAddr, LSM6DS33_CTRL2_G, ctrl2, "Failed to set LSM6DS33 gyro CTRL2_G"));
}

bool RTIMULSM6DS33LIS3MDL::setGyroCTRL7()
{
    unsigned char ctrl7;


    switch (m_settings->m_LSM6DS33LIS3MDLGyroHpf) {
    case LSM6DS33_GYRO_HPF_0:
        break;

    case LSM6DS33_GYRO_HPF_1:
        ctrl7 |= 0x10;
        break;

    case LSM6DS33_GYRO_HPF_2:
        ctrl7 |= 0x20;
        break;

    case LSM6DS33_GYRO_HPF_3:
        ctrl7 |= 0x30;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 gyro high pass filter code %d\n", m_settings->m_LSM6DS33LIS3MDLGyroHpf);
        return false;
    }

    m_settings->HALWrite(m_accelGyroSlaveAddr, LSM6DS33_CTRL6_C, 0x40, "Failed to set LSM6DS33 accel CTRL6_C");
    // Turn off HPF
    ctrl7 = 0x0;
    return m_settings->HALWrite(m_accelGyroSlaveAddr, LSM6DS33_CTRL7_G, ctrl7, "Failed to set LSM6DS33 gyro CTRL7");
}

bool RTIMULSM6DS33LIS3MDL::setCompassCTRL1()
{
    unsigned char ctrl1;

    switch (m_settings->m_LSM6DS33LIS3MDLCompassSampleRate) {
    case LIS3MDL_SAMPLERATE_0_625:
        ctrl1 = 0x00;
        break;

    case LIS3MDL_SAMPLERATE_1_25:
        ctrl1 = 0x04;
        break;

    case LIS3MDL_SAMPLERATE_2_5:
        ctrl1 = 0x08;
        break;

    case LIS3MDL_SAMPLERATE_5:
        ctrl1 = 0x0c;
        break;

    case LIS3MDL_SAMPLERATE_10:
        ctrl1 = 0x10;
        break;

    case LIS3MDL_SAMPLERATE_20:
        ctrl1 = 0x14;
        break;

    case LIS3MDL_SAMPLERATE_40:
        ctrl1 = 0x18;
        break;

    case LIS3MDL_SAMPLERATE_80:
        ctrl1 = 0x1c;
        break;

    case LIS3MDL_SAMPLERATE_FAST:
        // Enable FAST_ODR
        ctrl1 = 0x02 | (m_settings->m_LSM6DS33LIS3MDLCompassPowerMode << 5);
        break;

    default:
        HAL_ERROR1("Illegal LIS3MDL compass sample rate code %d\n", m_settings->m_LSM6DS33LIS3MDLCompassSampleRate);
        return false;
    }

    return m_settings->HALWrite(m_compassSlaveAddr, LIS3MDL_CTRL_REG1, ctrl1, "Failed to set LIS3MDL CTRL1");
}

bool RTIMULSM6DS33LIS3MDL::setCompassCTRL2()
{
    unsigned char ctrl2;

    // Convert FSR to uT
    switch (m_settings->m_LSM6DS33LIS3MDLCompassFsr) {
    case LIS3MDL_FSR_4:
        ctrl2 = 0x00;
        m_compassScale = (RTFLOAT)0.146;
        break;

    case LIS3MDL_FSR_8:
        ctrl2 = 0x20;
        m_compassScale = (RTFLOAT)0.29;
        break;

    case LIS3MDL_FSR_12:
        ctrl2 = 0x40;
        m_compassScale = (RTFLOAT)0.44;
        break;

    case LIS3MDL_FSR_16:
        ctrl2 = 0x60;
        m_compassScale = (RTFLOAT)0.58;
        break;

    default:
        HAL_ERROR1("Illegal LIS3MDL compass FSR code %d\n", m_settings->m_LSM6DS33LIS3MDLCompassFsr);
        return false;
    }

    return m_settings->HALWrite(m_compassSlaveAddr, LIS3MDL_CTRL_REG2, ctrl2, "Failed to set LIS3MDL CTRL2");
}

bool RTIMULSM6DS33LIS3MDL::setCompassCTRL3()
{
    unsigned char ctrl3;

    // Turn on magnetometer
    if (m_settings->m_LSM6DS33LIS3MDLCompassSampleRate < LIS3MDL_SAMPLERATE_FAST) {
        // Single conversion mode
        ctrl3 = 0x01;
    } else {
        // Continuous-conversion mode
        ctrl3 = 0x00;
    }
    ctrl3=0;

    return m_settings->HALWrite(m_compassSlaveAddr, LIS3MDL_CTRL_REG3, ctrl3, "Failed to set LIS3MDL CTRL3");
}

bool RTIMULSM6DS33LIS3MDL::setCompassCTRL4()
{
    if ((m_settings->m_LSM6DS33LIS3MDLCompassPowerMode < LIS3MDL_POWER_LP) || (m_settings->m_LSM6DS33LIS3MDLCompassPowerMode > LIS3MDL_POWER_UHP)) {
        HAL_ERROR1("Illegal LIS3MDL compass power mode code %d\n", m_settings->m_LSM6DS33LIS3MDLCompassPowerMode);
        return false;
    }

    return m_settings->HALWrite(m_compassSlaveAddr, LIS3MDL_CTRL_REG4, m_settings->m_LSM6DS33LIS3MDLCompassPowerMode<<2, "Failed to set LIS3MDL CTRL4");
}

bool RTIMULSM6DS33LIS3MDL::setCompassCTRL5()
{
    return true;
    return m_settings->HALWrite(m_compassSlaveAddr, LIS3MDL_CTRL_REG5, 0x40, "Failed to set LIS3MDL CTRL5");
}

int RTIMULSM6DS33LIS3MDL::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMULSM6DS33LIS3MDL::IMURead()
{
    unsigned char status;
    unsigned char gyroData[6];
    unsigned char accelData[6];
    unsigned char compassData[6];

    if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM6DS33_STATUS_REG, 1, &status, "Failed to read LSM6DS33 status"))
        return false;

    // Check if new gyro dataset is available
    if ((status & 0x02) == 0)
        return false;

    if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM6DS33_OUTX_L_G, 6, gyroData, "Failed to read LSM6DS33 gyro data"))
        return false;

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

    if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM6DS33_OUTX_L_XL, 6, accelData, "Failed to read LSM6DS33 accel data"))
        return false;


    if (!m_settings->HALRead(m_compassSlaveAddr, LIS3MDL_OUT_X_L, 6, compassData, "Failed to read LIS3MDL compass data"))
        return false;

    RTMath::convertToVector(gyroData, m_imuData.gyro, m_gyroScale, false);
    RTMath::convertToVector(accelData, m_imuData.accel, m_accelScale, false);
    RTMath::convertToVector(compassData, m_imuData.compass, m_compassScale/10, false);

    //  sort out gyro axes and correct for bias

    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;

    m_imuData.accel.setX(-m_imuData.accel.x());

    //  sort out compass axes

    m_imuData.compass.setY(-m_imuData.compass.y());
    m_imuData.compass.setZ(-m_imuData.compass.z());

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    //  now update the filter

    updateFusion();

    return true;
}
