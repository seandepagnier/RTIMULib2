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


#include "RTIMUICM20948.h"
#include "RTIMUSettings.h"
#include <iostream>
#include <bitset>


RTIMUICM20948::RTIMUICM20948(RTIMUSettings *settings) : RTIMU(settings)
{

}

RTIMUICM20948::~RTIMUICM20948()
{
}

bool RTIMUICM20948::setSampleRate(int rate)
{
    if ((rate < ICM20948_SAMPLERATE_MIN) || (rate > ICM20948_SAMPLERATE_MAX)) {
        HAL_ERROR1("Illegal sample rate %d\n", rate);
        return false;
    }

    //  Note: rates interact with the lpf settings

    if ((rate < ICM20948_SAMPLERATE_MAX) && (rate >= 8000))
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

bool RTIMUICM20948::setGyroLpf(unsigned char lpf)
{
    switch (lpf) {
    case ICM20948_GYRO_LPF_BYPASS:
    case ICM20948_GYRO_LPF_196_6:
    case ICM20948_GYRO_LPF_151_8:
    case ICM20948_GYRO_LPF_119_5:
    case ICM20948_GYRO_LPF_51_2:
    case ICM20948_GYRO_LPF_23_9:
    case ICM20948_GYRO_LPF_11_6:
    case ICM20948_GYRO_LPF_5_7:
    case ICM20948_GYRO_LPF_361_4:
        m_gyroLpf = lpf;
        return true;

    default:
        HAL_ERROR1("Illegal ICM20948 gyro lpf %d\n", lpf);
        return false;
    }
}

bool RTIMUICM20948::setAccelLpf(unsigned char lpf)
{
    switch (lpf) {
    case ICM20948_ACCEL_LPF_BYPASS:
    case ICM20948_ACCEL_LPF_246:
    case ICM20948_ACCEL_LPF_111_4:
    case ICM20948_ACCEL_LPF_50_4:
    case ICM20948_ACCEL_LPF_23_9:
    case ICM20948_ACCEL_LPF_11_5:
    case ICM20948_ACCEL_LPF_5_7:
    case ICM20948_ACCEL_LPF_473:
        m_accelLpf = lpf;
        return true;

    default:
        HAL_ERROR1("Illegal ICM20948 accel lpf %d\n", lpf);
        return false;
    }
}


bool RTIMUICM20948::setCompassRate(int rate)
{
    if ((rate < ICM20948_COMPASSRATE_MIN) || (rate > ICM20948_COMPASSRATE_MAX)) {
        HAL_ERROR1("Illegal compass rate %d\n", rate);
        return false;
    }
    if (rate <= 10)
        m_compassRate = 10;
    else if (rate <= 20)
        m_compassRate = 20;
    else if (rate <= 50)
        m_compassRate = 50;
    else
        m_compassRate = 100;
    return true;
}

bool RTIMUICM20948::setGyroFsr(unsigned char fsr)
{
    switch (fsr) {
    case ICM20948_GYROFSR_250:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (131.0 * 180.0);
        return true;

    case ICM20948_GYROFSR_500:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (62.5 * 180.0);
        return true;

    case ICM20948_GYROFSR_1000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (32.8 * 180.0);
        return true;

    case ICM20948_GYROFSR_2000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (16.4 * 180.0);
        return true;

    default:
        HAL_ERROR1("Illegal ICM20948 gyro fsr %d\n", fsr);
        return false;
    }
}

bool RTIMUICM20948::setAccelFsr(unsigned char fsr)
{
    switch (fsr) {
    case ICM20948_ACCELFSR_2:
        m_accelFsr = fsr;
        m_accelScale = 1.0/16384.0;
        return true;

    case ICM20948_ACCELFSR_4:
        m_accelFsr = fsr;
        m_accelScale = 1.0/8192.0;
        return true;

    case ICM20948_ACCELFSR_8:
        m_accelFsr = fsr;
        m_accelScale = 1.0/4096.0;
        return true;

    case ICM20948_ACCELFSR_16:
        m_accelFsr = fsr;
        m_accelScale = 1.0/2048.0;
        return true;

    default:
        HAL_ERROR1("Illegal ICM20948 accel fsr %d\n", fsr);
        return false;
    }
}

bool RTIMUICM20948::SelectRegisterBank(uint8_t reg_bank)
{
    if (reg_bank != m_reg_bank) {
        if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_BANK_SEL, reg_bank, "Failed to set ICM20948 register bank"))
            return false;
        m_reg_bank = reg_bank;
    }
    return true;
}


bool RTIMUICM20948::IMUInit()
{
    m_firstTime = true;

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

    setSampleRate(m_settings->m_ICM20948GyroAccelSampleRate);
    setCompassRate(m_settings->m_ICM20948CompassSampleRate);
    setGyroLpf(m_settings->m_ICM20948GyroLpf);
    setAccelLpf(m_settings->m_ICM20948AccelLpf);
    setGyroFsr(m_settings->m_ICM20948GyroFsr);
    setAccelFsr(m_settings->m_ICM20948AccelFsr);

    setCalibrationData();


    //  enable the bus

    if (!m_settings->HALOpen())
        return false;

    //  reset the ICM20948

    if (!SelectRegisterBank(ICM20948_BANK0))
        return false;
    // DEVICE_RESET
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_PWR_MGMT_1, 0x80, "Failed to initiate ICM20948 reset"))
        return false;

    m_settings->delayMs(100);

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_PWR_MGMT_1, 0x01, "Failed to stop ICM20948 reset"))
        return false;

    uint8_t result;
    if (!m_settings->HALRead(m_slaveAddr, ICM20948_WHO_AM_I, 1, &result, "Failed to read ICM20948 id"))
        return false;

    if (result != ICM20948_ID) {
        HAL_ERROR2("Incorrect %s id %d\n", IMUName(), result);
        return false;
    }
    
    //  now configure the various components

    if (!setGyroConfig())
        return false;

    if (!setAccelConfig())
        return false;

    if(!compassSetup())
        return false;
    
    if (!setSampleRate())
        return false;

    gyroBiasInit();

    if (!resetFifo())
        return false;

    HAL_INFO1("%s init complete\n", IMUName());
    return true;
}


bool RTIMUICM20948::resetFifo()
{
    if (!SelectRegisterBank(ICM20948_BANK0))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_ENABLE, 0, "Writing int enable"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_EN_1, 0, "Writing fifo enable"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_EN_2, 0, "Writing fifo enable"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0, "Writing user control"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_RST, 0x01, "Resetting fifo"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_RST, 0x00, "Resetting fifo"))
        return false;


    m_settings->delayMs(50);
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_EN_1, 0x01, "Failed to set FIFO enables"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_EN_2, 0x1e, "Failed to set FIFO enables"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0x60, "Enabling the fifo"))
        return false;
    return true;
}


bool RTIMUICM20948::setGyroConfig()
{
    // GYRO_CONFIG_1 register is in bank 2
    if (!SelectRegisterBank(ICM20948_BANK2))
        return false;
    
    uint8_t value;
    
    // gyro sample rate
    uint8_t rate = (1100.0 / m_sampleRate) - 1;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_GYRO_SMPLRT_DIV, rate, "Failed to write gyro sample rate"))
        return false;
        
    value = (/*m_gyroLpf*/ 4 & 0x07) << 3;
    value |= m_gyroFsr << 1; // gyro fsr
    value |= 1; // enable lpf
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_GYRO_CONFIG_1, value, "Failed to write gyro config"))
        return false;

    // TODO: do lower settings have higher noise!?!?  Is 5 ok??
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_GYRO_CONFIG_2, 0, "Failed to write gyro config"))
        return false;
    
    return true;
}

bool RTIMUICM20948::setAccelConfig()
{
    // ACCEL_CONFIG register is in bank 2
    if (!SelectRegisterBank(ICM20948_BANK2))
        return false;
    
    unsigned char value;
    
    // // accelerometer sample rate
#if 0 // gyro determines output rate
    uint16_t rate = (1125.0 / m_sampleRate) - 1;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_ACCEL_SMPLRT_DIV_1, (rate >> 8) & 0xFF, "Failed to write accelerometer MSB sample rate"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_ACCEL_SMPLRT_DIV_2, rate & 0xFF, "Failed to write accelerometer LSB sample rate"))
        return false;
#endif
        
    // accelerometer fsr
    value = (m_accelFsr << 1);
    value |= (/*m_accelLpf*/ 4 & 0x07) << 3;
    value |= 1; // enable lpf
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_ACCEL_CONFIG, value, "Failed to write accelerometer config"))
        return false;

    // TODO: play with this
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_ACCEL_CONFIG_2, 0, "Failed to write accelerometer config"))
        return false;
    return true;
}

bool RTIMUICM20948::setSampleRate()
{
    // if (m_sampleRate > 1000)
    //     return true;                                        // SMPRT not used above 1000Hz
#if 0
    if (!SelectRegisterBank(ICM20948_BANK0)) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_LP_CONFIG, 0x40, "Failed to set sample rate"))
        return false;
#endif
    return true;
}

bool RTIMUICM20948::trigger_mag_io()
{
    uint8_t userControl;
    if (!SelectRegisterBank(ICM20948_BANK0)) return false;

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg")) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, userControl | 0x20, "Failed to set user_ctrl reg")) return false;
    m_settings->delayMs(5);
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, userControl, "Failed to set user_ctrl reg")) return false;
    
    return true;
}

bool RTIMUICM20948::mag_write(uint8_t reg, uint8_t value)
{
    if (!SelectRegisterBank(ICM20948_BANK3))
        return false;
    
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR, "Failed to set magnetometer as slave"))
        return false;    
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_REG, reg, "Failed to set magnetometer reg to slave"))
        return false;    
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_DO, value, "Failed to set magnetometer reg value to slave"))
        return false;

    return trigger_mag_io();
}

uint8_t RTIMUICM20948::mag_read(uint8_t reg)
{
    if (!SelectRegisterBank(ICM20948_BANK3))
        return false;
    
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80, "Failed to set magnetometer as slave")) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_REG, reg, "Failed to set magnetometer reg to slave")) return false;
//    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_DO, 0xff, "Failed to set magnetometer reg value to slave")) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_CTRL, 0x80 | 1, "Failed to set magnetometer reg value to slave")) return false;

    trigger_mag_io();

    uint8_t b = 0xff;
    m_settings->HALRead(m_slaveAddr, ICM20948_EXT_SLV_SENS_DATA_00, 1, &b, "Failed to read compass data");
    return b;
}


bool RTIMUICM20948::bypassOn()
{
    // i2cset -y 1 0x68 0x7F 0x00
    if (!SelectRegisterBank(ICM20948_BANK0))
        return false;

    unsigned char userControl;

    // i2cset -y 1 0x68 0x7F 0x00 && i2cget -y 1 0x68 0x03
    if (!m_settings->HALRead(m_slaveAddr, ICM20948_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl &= ~0x20;

    // i2cset -y 1 0x68 0x7F 0x00 && i2cset -y 1 0x68 0x03 
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 1, &userControl, "Failed to write user_ctrl reg"))
        return false;

    m_settings->delayMs(50);

    // i2cset -y 1 0x68 0x7F 0x00 && i2cset -y 1 0x68 0x0F 0x82
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_PIN_CFG, 0x82, "Failed to write int_pin_cfg reg"))
        return false;

    m_settings->delayMs(50);
    return true;
}


bool RTIMUICM20948::bypassOff()
{
    if (!SelectRegisterBank(ICM20948_BANK0))
        return false;

    unsigned char userControl;

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl |= 0x20;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 1, &userControl, "Failed to write user_ctrl reg"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_PIN_CFG, 0x80, "Failed to write int_pin_cfg reg"))
         return false;

    m_settings->delayMs(50);
    return true;
}


bool RTIMUICM20948::compassSetup()
{
    if (!SelectRegisterBank(ICM20948_BANK3)) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_CTRL, 0x08, "Failed to set I2C master mode")) return false;

    // soft reset, reading writing with mag_read and mag_write is not completely
    // reliable...  a few retries may be needed to ensure these work but they are only
    // needed during setup so not a big deal, it may be better to use bypass
    int retries = 0;
    for(;;) {
        if (!SelectRegisterBank(ICM20948_BANK0)) return false;
        if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0x2, "Failed to set user_ctrl reg")) return false;
        mag_write(AK09916_CNTL3, 0x01);

        uint8_t id = mag_read(AK09916_WHO_AM_I);
        if(id == 9)
            break;

        if(retries++ > 10) {
            printf("ICM20948 has AK09916 wrong ID! %x\n", id);
            return false;
        }
 
        // reset i2c bus and try again...
     }

    // ensure rate is set
    uint8_t rate = 0;
    for(;;) {
        rate = mag_read(AK09916_CNTL2);
        if(rate == 8)
            break;
        mag_write(AK09916_CNTL2, 0x8);
        m_settings->delayMs(100);

        if(retries++ > 10) {
            printf("AK09916 failed to set rate!!!!!\n");
            break;
        }
    }
    
    if (!SelectRegisterBank(ICM20948_BANK3)) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80/*read*/, "Failed to set slave 0 address")) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_REG, AK09916_ST1+1, "Failed to set slave 0 reg")) return false;

    // read 8 bytes reads the 3 axes, 0 register and status register
    // maybe... could use SLV1 for status and a single byte, and possibly not store it in the fifo using REG_DIS?  this is simpler, and plenty of fifo space, but slightly more data: 20 bytes per sample instead of 18 or 19
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_CTRL, 0x88, "Failed to set slave 0 ctrl")) return false;

    // needed to ensure correct data if reading without fifo
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_DELAY_CTRL, 0x81, "Failed to set mst delay"))
        return false;
    
    m_settings->delayMs(5);
    if (!SelectRegisterBank(ICM20948_BANK0)) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0x22, "Failed to set user_ctrl reg")) return false;
    m_settings->delayMs(5);

    return true;
}

int RTIMUICM20948::IMUGetPollInterval()
{
    if (m_sampleRate > 400)
        return 1;
    else
        return (400 / m_sampleRate);
}

bool RTIMUICM20948::IMURead()
{
    unsigned char fifoCount[2];
    unsigned char fifoData[400];
    unsigned int count;

    if (!SelectRegisterBank(ICM20948_BANK0)) return false;
    if (!m_settings->HALRead(m_slaveAddr, ICM20948_FIFO_COUNTH, 2, fifoCount, "Failed to read fifo count")) {
        return false;
    }
    
    count = ((unsigned int)fifoCount[0] << 8) + fifoCount[1];
    if (count < ICM20948_FIFO_CHUNK_SIZE)
        return false;

    if (count > 512) {
        HAL_INFO("ICM20948 fifo has overflowed\n");
        HAL_INFO("ICM20948 trying to reinitialize sensors\n");
        IMUInit();
        return false;
    }

    if (count >= 512) {
        // is this even possible?
        HAL_INFO("ICM20948 fifo count invalid!!\n");
        resetFifo();
        return false;
    }

    if (count > 400) {
        HAL_INFO("ICM20948 fifo has more than 20 samples!!\n");
        count = 400;
    }
    
    count /= ICM20948_FIFO_CHUNK_SIZE;
    // read fifo data in two reads if more than 10 samples
    int roffset = 0;
    int fcount = count;
    if (count > 10) {
        if (!m_settings->HALRead(m_slaveAddr, ICM20948_FIFO_R_W, ICM20948_FIFO_CHUNK_SIZE*10, fifoData, "Failed to read fifo data"))
            return false;
        fcount = count - 10;
        roffset = ICM20948_FIFO_CHUNK_SIZE*10;
    }
    if (!m_settings->HALRead(m_slaveAddr, ICM20948_FIFO_R_W, fcount*ICM20948_FIFO_CHUNK_SIZE, fifoData+roffset, "Failed to read fifo data"))
        return false;

    RTVector3 accel_t, gyro_t, compass_t;
    unsigned char *p = fifoData;

    int compass_count = 0;
//    printf("count %d %x %x\n", count, fifoCount[0], fifoCount[1]);
    for(uint8_t i=0; i<count; i++) {
        RTVector3 accel, gyro, compass;
        RTMath::convertToVector(p,    accel, m_accelScale, true);
        RTMath::convertToVector(p+6,  gyro, m_gyroScale, true);
        RTMath::convertToVector(p+12, compass, .6/4, false);

        if(fabs(gyro.x()) > 3 || fabs(gyro.y()) > 3 || fabs(gyro.z()) > 3)
            printf("AAAHAHA %f %f %f %d %d\n", gyro.x(), gyro.y(), gyro.z(), i, count);

        accel_t += accel;
        gyro_t += gyro;

        if(!(p[19] & 0x08))
        { // compass data valid?
            compass_t += compass;
            compass_count++;
        }
        p += ICM20948_FIFO_CHUNK_SIZE;
    }

    if(compass_count == 0)
        return false;

    // average samples
    for(int i=0; i<3; i++) {
        m_imuData.accel.setData(i, accel_t.data(i)/count);
        m_imuData.gyro.setData(i, gyro_t.data(i)/count);
        m_imuData.compass.setData(i, compass_t.data(i)/compass_count);
    }
    //  sort out gyro axes

    // x fwd y right z down
    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;

    // x back y left z up
    m_imuData.accel.setX(-m_imuData.accel.x());

    //  sort out compass axes

    // x fwd y right z down

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    // if (m_firstTime)
    //     m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
    // else
    //     m_imuData.timestamp += m_sampleInterval;

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

    m_firstTime = false;

    //  now update the filter

    updateFusion();
    
    return true;
}


