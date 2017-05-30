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

//  The MPU-9255 and SPI driver code is based on code generously supplied by
//  staslock@gmail.com (www.clickdrive.io)

#include "RTIMUMPU9255.h"
#include "RTIMUSettings.h"

RTIMUMPU9255::RTIMUMPU9255(RTIMUSettings *settings) : RTIMU(settings)
{

}

RTIMUMPU9255::~RTIMUMPU9255()
{
}

bool RTIMUMPU9255::setSampleRate(int rate)
{
    if ((rate < MPU9255_SAMPLERATE_MIN) || (rate > MPU9255_SAMPLERATE_MAX)) {
        HAL_ERROR1("Illegal sample rate %d\n", rate);
        return false;
    }

    //  Note: rates interact with the lpf settings

    if ((rate < MPU9255_SAMPLERATE_MAX) && (rate >= 8000))
        rate = 8000;

    if ((rate < 8000) && (rate >= 1000))
        rate = 1000;

    if (rate < 1000) {
        int sampleDiv = (1000 / rate) - 1;
        m_sampleRate = 1000 / (1 + sampleDiv);
    } else {
        m_sampleRate = rate;
    }
    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;
    return true;
}

bool RTIMUMPU9255::setGyroLpf(unsigned char lpf)
{
    switch (lpf) {
    case MPU9255_GYRO_LPF_8800:
    case MPU9255_GYRO_LPF_3600:
    case MPU9255_GYRO_LPF_250:
    case MPU9255_GYRO_LPF_184:
    case MPU9255_GYRO_LPF_92:
    case MPU9255_GYRO_LPF_41:
    case MPU9255_GYRO_LPF_20:
    case MPU9255_GYRO_LPF_10:
    case MPU9255_GYRO_LPF_5:
        m_gyroLpf = lpf;
        return true;

    default:
        HAL_ERROR1("Illegal MPU9255 gyro lpf %d\n", lpf);
        return false;
    }
}

bool RTIMUMPU9255::setAccelLpf(unsigned char lpf)
{
    switch (lpf) {
    case MPU9255_ACCEL_LPF_1130:
    case MPU9255_ACCEL_LPF_460:
    case MPU9255_ACCEL_LPF_184:
    case MPU9255_ACCEL_LPF_92:
    case MPU9255_ACCEL_LPF_41:
    case MPU9255_ACCEL_LPF_20:
    case MPU9255_ACCEL_LPF_10:
    case MPU9255_ACCEL_LPF_5:
        m_accelLpf = lpf;
        return true;

    default:
        HAL_ERROR1("Illegal MPU9255 accel lpf %d\n", lpf);
        return false;
    }
}


bool RTIMUMPU9255::setCompassRate(int rate)
{
    if ((rate < MPU9255_COMPASSRATE_MIN) || (rate > MPU9255_COMPASSRATE_MAX)) {
        HAL_ERROR1("Illegal compass rate %d\n", rate);
        return false;
    }
    m_compassRate = rate;
    return true;
}

bool RTIMUMPU9255::setGyroFsr(unsigned char fsr)
{
    switch (fsr) {
    case MPU9255_GYROFSR_250:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (131.0 * 180.0);
        return true;

    case MPU9255_GYROFSR_500:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (62.5 * 180.0);
        return true;

    case MPU9255_GYROFSR_1000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (32.8 * 180.0);
        return true;

    case MPU9255_GYROFSR_2000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (16.4 * 180.0);
        return true;

    default:
        HAL_ERROR1("Illegal MPU9255 gyro fsr %d\n", fsr);
        return false;
    }
}

bool RTIMUMPU9255::setAccelFsr(unsigned char fsr)
{
    switch (fsr) {
    case MPU9255_ACCELFSR_2:
        m_accelFsr = fsr;
        m_accelScale = 1.0/16384.0;
        return true;

    case MPU9255_ACCELFSR_4:
        m_accelFsr = fsr;
        m_accelScale = 1.0/8192.0;
        return true;

    case MPU9255_ACCELFSR_8:
        m_accelFsr = fsr;
        m_accelScale = 1.0/4096.0;
        return true;

    case MPU9255_ACCELFSR_16:
        m_accelFsr = fsr;
        m_accelScale = 1.0/2048.0;
        return true;

    default:
        HAL_ERROR1("Illegal MPU9255 accel fsr %d\n", fsr);
        return false;
    }
}


bool RTIMUMPU9255::IMUInit()
{
    unsigned char result;

    m_firstTime = true;

#ifdef MPU9255_CACHE_MODE
    m_cacheIn = m_cacheOut = m_cacheCount = 0;
#endif

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

    m_slaveAddr = m_settings->m_I2CSlaveAddress;

    setSampleRate(m_settings->m_MPU9255GyroAccelSampleRate);
    setCompassRate(m_settings->m_MPU9255CompassSampleRate);
    setGyroLpf(m_settings->m_MPU9255GyroLpf);
    setAccelLpf(m_settings->m_MPU9255AccelLpf);
    setGyroFsr(m_settings->m_MPU9255GyroFsr);
    setAccelFsr(m_settings->m_MPU9255AccelFsr);

    setCalibrationData();


    //  enable the bus

    if (!m_settings->HALOpen())
        return false;

    //  reset the MPU9255

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_PWR_MGMT_1, 0x80, "Failed to initiate MPU9255 reset"))
        return false;

    m_settings->delayMs(100);

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_PWR_MGMT_1, 0x00, "Failed to stop MPU9255 reset"))
        return false;

    if (!m_settings->HALRead(m_slaveAddr, MPU9255_WHO_AM_I, 1, &result, "Failed to read MPU9255 id"))
        return false;

    if (result != MPU9255_ID) {
        HAL_ERROR2("Incorrect %s id %d\n", IMUName(), result);
        return false;
    }

    //  now configure the various components

    if (!setGyroConfig())
        return false;

    if (!setAccelConfig())
        return false;

    if (!setSampleRate())
        return false;

    if(!compassSetup()) {
        return false;
    }

    if (!setCompassRate())
        return false;

    //  enable the sensors

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_PWR_MGMT_1, 1, "Failed to set pwr_mgmt_1"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_PWR_MGMT_2, 0, "Failed to set pwr_mgmt_2"))
         return false;

    //  select the data to go into the FIFO and enable

    if (!resetFifo())
        return false;

    gyroBiasInit();

    HAL_INFO1("%s init complete\n", IMUName());
    return true;
}


bool RTIMUMPU9255::resetFifo()
{
    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_INT_ENABLE, 0, "Writing int enable"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_FIFO_EN, 0, "Writing fifo enable"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_USER_CTRL, 0, "Writing user control"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_USER_CTRL, 0x04, "Resetting fifo"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_USER_CTRL, 0x60, "Enabling the fifo"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_INT_ENABLE, 1, "Writing int enable"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_FIFO_EN, 0x79, "Failed to set FIFO enables"))
        return false;

    return true;
}

bool RTIMUMPU9255::setGyroConfig()
{
    unsigned char gyroConfig = m_gyroFsr + ((m_gyroLpf >> 3) & 3);
    unsigned char gyroLpf = m_gyroLpf & 7;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_GYRO_CONFIG, gyroConfig, "Failed to write gyro config"))
         return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_GYRO_LPF, gyroLpf, "Failed to write gyro lpf"))
         return false;
    return true;
}

bool RTIMUMPU9255::setAccelConfig()
{
    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_ACCEL_CONFIG, m_accelFsr, "Failed to write accel config"))
         return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_ACCEL_LPF, m_accelLpf, "Failed to write accel lpf"))
         return false;
    return true;
}

bool RTIMUMPU9255::setSampleRate()
{
    if (m_sampleRate > 1000)
        return true;                                        // SMPRT not used above 1000Hz

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_SMPRT_DIV, (unsigned char) (1000 / m_sampleRate - 1),
            "Failed to set sample rate"))
        return false;

    return true;
}

bool RTIMUMPU9255::compassSetup() {
    unsigned char asa[3];

    if (m_settings->m_busIsI2C) {
        // I2C mode

        bypassOn();

        // get fuse ROM data

        if (!m_settings->HALWrite(AK8963_ADDRESS, AK8963_CNTL, 0, "Failed to set compass in power down mode 1")) {
            bypassOff();
            return false;
        }

        if (!m_settings->HALWrite(AK8963_ADDRESS, AK8963_CNTL, 0x0f, "Failed to set compass in fuse ROM mode")) {
            bypassOff();
            return false;
        }

        if (!m_settings->HALRead(AK8963_ADDRESS, AK8963_ASAX, 3, asa, "Failed to read compass fuse ROM")) {
            bypassOff();
            return false;
        }

        if (!m_settings->HALWrite(AK8963_ADDRESS, AK8963_CNTL, 0, "Failed to set compass in power down mode 2")) {
            bypassOff();
            return false;
        }

        bypassOff();

    } else {
    //  SPI mode

        bypassOff();

        if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_MST_CTRL, 0x40, "Failed to set I2C master mode"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV0_ADDR, 0x80 | AK8963_ADDRESS, "Failed to set slave 0 address"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV0_REG, AK8963_ASAX, "Failed to set slave 0 reg"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV0_CTRL, 0x83, "Failed to set slave 0 ctrl"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV1_ADDR, AK8963_ADDRESS, "Failed to set slave 1 address"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV1_REG, AK8963_CNTL, "Failed to set slave 1 reg"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV1_CTRL, 0x81, "Failed to set slave 1 ctrl"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV1_DO, 0x00, "Failed to set compass in power down mode 2"))
            return false;

        m_settings->delayMs(10);

        if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV1_DO, 0x0f, "Failed to set compass in fuse mode"))
            return false;

        if (!m_settings->HALRead(m_slaveAddr, MPU9255_EXT_SENS_DATA_00, 3, asa, "Failed to read compass rom"))
            return false;

        if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV1_DO, 0x0, "Failed to set compass in power down mode 2"))
            return false;
    }
    //  both interfaces

    //  convert asa to usable scale factor
    //printf("asa %x%x%x\n", asa[0], asa[1], asa[2]);

    m_compassAdjust[0] = ((float)asa[0] - 128.0) / 256.0 + 1.0f;
    m_compassAdjust[1] = ((float)asa[1] - 128.0) / 256.0 + 1.0f;
    m_compassAdjust[2] = ((float)asa[2] - 128.0) / 256.0 + 1.0f;

//    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_MST_CTRL, 0x40, "Failed to set I2C master mode"))
//        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV0_ADDR, 0x80 | AK8963_ADDRESS, "Failed to set slave 0 address"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV0_REG, AK8963_ST1+1, "Failed to set slave 0 reg"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV0_CTRL, 0x86, "Failed to set slave 0 ctrl"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV1_ADDR, AK8963_ADDRESS, "Failed to set slave 1 address"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV1_REG, AK8963_CNTL, "Failed to set slave 1 reg"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV1_CTRL, 0x81, "Failed to set slave 1 ctrl"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV1_DO, 0x6, "Failed to set slave 1 DO"))
        return false;

    // needed to ensure correct data if reading without fifo
    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_MST_DELAY_CTRL, 0x81, "Failed to set mst delay"))
        return false;

    return true;
}

bool RTIMUMPU9255::setCompassRate()
{
    int rate;

    rate = m_sampleRate / m_compassRate - 1;

    if (rate > 31)
        rate = 31;

    rate = 0;
    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_I2C_SLV4_CTRL, rate, "Failed to set slave ctrl 4"))
         return false;
    return true;
}


bool RTIMUMPU9255::bypassOn()
{
    unsigned char userControl;

    if (!m_settings->HALRead(m_slaveAddr, MPU9255_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl &= ~0x20;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_USER_CTRL, 1, &userControl, "Failed to write user_ctrl reg"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_INT_PIN_CFG, 0x82, "Failed to write int_pin_cfg reg"))
        return false;

    m_settings->delayMs(50);
    return true;
}


bool RTIMUMPU9255::bypassOff()
{
    unsigned char userControl;

    if (!m_settings->HALRead(m_slaveAddr, MPU9255_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl |= 0x20;

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_USER_CTRL, 1, &userControl, "Failed to write user_ctrl reg"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, MPU9255_INT_PIN_CFG, 0x80, "Failed to write int_pin_cfg reg"))
         return false;

    m_settings->delayMs(50);
    return true;
}


int RTIMUMPU9255::IMUGetPollInterval()
{
    if (m_sampleRate > 400)
        return 1;
    else
        return (400 / m_sampleRate);
}

bool RTIMUMPU9255::IMURead()
{
    unsigned char fifoCount[2];
    unsigned int count;
    unsigned char fifoData[360];

    if (!m_settings->HALRead(m_slaveAddr, MPU9255_FIFO_COUNT_H, 2, fifoCount, "Failed to read fifo count")) {
        resetFifo();
        return false;
    }

    count = ((unsigned int)fifoCount[0] << 8) + fifoCount[1];

    if (count < MPU9255_FIFO_CHUNK_SIZE)
        return false;
    
    if (count == 512) {
        HAL_INFO("MPU-9255 fifo has overflowed\n");
        HAL_INFO("MPU-9255 trying to reinitialize sensors\n");
        IMUInit();
        return false;
        resetFifo();
        return false;
    }

    if (count >= 512) {
        // I have seen this in cases where the device gets in a bad state...
        // I know to fixed it by rebooting the pi :-/  for now simply report it
        HAL_INFO("MPU-9255 fifo has invalid count\n");
        resetFifo();
        return false;
    }

    if (count > 360) {
        HAL_INFO("MPU-9255 fifo has more than 20 samples!!\n");
        count = 360;
    }

#if 0    
    if (count < MPU9255_FIFO_CHUNK_SIZE)
        return false;

    if (!m_settings->HALRead(m_slaveAddr, MPU9255_FIFO_R_W, MPU9255_FIFO_CHUNK_SIZE, fifoData, "Failed to read fifo data"))
        return false;

    if (!m_settings->HALRead(m_slaveAddr, MPU9255_EXT_SENS_DATA_00, 6, compassData, "Failed to read compass data"))
        return false;

    RTMath::convertToVector(fifoData, m_imuData.accel, m_accelScale, true);
    RTMath::convertToVector(fifoData + 6, m_imuData.gyro, m_gyroScale, true);
    RTMath::convertToVector(compassData + 1, m_imuData.compass, 0.6f, false);
#else

    count /= MPU9255_FIFO_CHUNK_SIZE;

    if (count > 10) {
        if (!m_settings->HALRead(m_slaveAddr, MPU9255_FIFO_R_W, MPU9255_FIFO_CHUNK_SIZE*10, fifoData, "Failed to read fifo data") ||
            !m_settings->HALRead(m_slaveAddr, MPU9255_FIFO_R_W, (count-10)*MPU9255_FIFO_CHUNK_SIZE, fifoData+MPU9255_FIFO_CHUNK_SIZE*10, "Failed to read fifo data"))
            return false;
    } else       
        if (!m_settings->HALRead(m_slaveAddr, MPU9255_FIFO_R_W, count*MPU9255_FIFO_CHUNK_SIZE, fifoData, "Failed to read fifo data"))
            return false;

    RTVector3 accel_t, gyro_t, compass_t;
    unsigned char *p = fifoData;
    //printf("count %d\n", count);
    for(uint8_t i=0; i<count; i++) {
        RTVector3 accel, gyro, compass;
        RTMath::convertToVector(p,    accel, m_accelScale, true);
        RTMath::convertToVector(p+6,  gyro, m_gyroScale, true);
        RTMath::convertToVector(p+12, compass, .6, false);

        /* check for out of sync, if high/low bytes are not shadowed */
        static RTVector3 lastcompass;
        if(fabs(lastcompass.z() - compass.z()) > 64) {
            printf("possible byte issue %f %f",
                   lastcompass.z(), compass.z());
            for(int i=0; i<8; i++)
                printf(" %x", p[i+12]);
            printf("\n");
        }
        lastcompass = compass;

        p += MPU9255_FIFO_CHUNK_SIZE;
    
        accel_t += accel;
        gyro_t += gyro;
        compass_t += compass;
    }

    // average samples
    for(int i=0; i<3; i++) {
        m_imuData.accel.setData(i, accel_t.data(i)/count);
        m_imuData.gyro.setData(i, gyro_t.data(i)/count);
        m_imuData.compass.setData(i, compass_t.data(i)/count);
    }
#endif

    //  sort out gyro axes

    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;

    m_imuData.accel.setX(-m_imuData.accel.x());

    //  use the compass fuse data adjustments

    m_imuData.compass.setX(m_imuData.compass.x() * m_compassAdjust[0]);
    m_imuData.compass.setY(m_imuData.compass.y() * m_compassAdjust[1]);
    m_imuData.compass.setZ(m_imuData.compass.z() * m_compassAdjust[2]);

    //  sort out compass axes

    float temp;

    temp = m_imuData.compass.x();
    m_imuData.compass.setX(m_imuData.compass.y());
    m_imuData.compass.setY(-temp);

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    if (m_firstTime)
        m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
    else
        m_imuData.timestamp += m_sampleInterval * count;

    m_firstTime = false;

    //  now update the filter

    updateFusion();

    return true;
}


