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
    m_compassRate = rate;
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
        m_settings->delayMs(10);
        m_reg_bank = reg_bank;
    }
    return true;
}


bool RTIMUICM20948::IMUInit()
{
    uint8_t result;

    m_firstTime = true;

#ifdef ICM20948_CACHE_MODE
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
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_PWR_MGMT_1, 0x80, "Failed to initiate ICM20948 reset"))
        return false;
    m_settings->delayMs(100);
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_PWR_MGMT_1, 0x01, "Failed to stop ICM20948 reset"))
        return false;


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

    
    // //  enable the sensors
    if (!SelectRegisterBank(ICM20948_BANK0))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_PWR_MGMT_1, 0x01, "Failed to stop ICM20948 reset"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_PWR_MGMT_2, 0x00, "Failed to stop ICM20948 reset"))
        return false;
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_PIN_CFG, 0x30, "Failed to set ICM20948_INT_PIN_CFG"))
        // return false;
    
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_EN_1, 0x0, "Writing fifo enable"))
    //     return false;
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_EN_2, 0b00011110, "Writing fifo enable"))
    //     return false;
    // // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_RST, 0x00, "Resetting fifo"))
    //     // return false;
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0b01000000, "Enabling the fifo"))
    //     return false;

    // if (!SelectRegisterBank(ICM20948_BANK3))
    //     return false;
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_CTRL, 0x4D, "Failed to set ICM20948_I2C_MST_CTRL"))
    //      return false;
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_DELAY_CTRL, 0x01, "Failed to set ICM20948_I2C_MST_DELAY_CTRL"))
    //      return false;

    // Reset the magnetometer
    // mag_write(AK09916_CNTL3, 0x01)
    // while self.mag_read(AK09916_CNTL3) == 0x01:
    //     time.sleep(0.0001)
    
    //  select the data to go into the FIFO and enable

    // if (!resetFifo())
        // return false;
    
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_PIN_CFG, 0x22, "Writing int enable"))
    //     return false;    
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_ENABLE_1, 0x01, "Writing int enable"))
    //     return false;
    
    gyroBiasInit();

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

    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0x04, "Resetting fifo"))
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_RST, 0x00, "Resetting fifo"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0x60, "Enabling the fifo"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_PIN_CFG, 0x22, "Writing int enable"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_ENABLE_1, 0x01, "Writing int enable"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_EN_1, 0x00, "Failed to set FIFO enables"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_FIFO_EN_2, 0b00011110, "Failed to set FIFO enables"))
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
        
    // gyro fsr
    if (!m_settings->HALRead(m_slaveAddr, ICM20948_GYRO_CONFIG_1, 1, &value, "Failed to read gyro config"))
        return false;
    value &= 0b11111001;
    value |= m_gyroFsr << 1;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_GYRO_CONFIG_1, value, "Failed to write gyro config"))
        return false;
    
    // gyro lpf
    if (!m_settings->HALRead(m_slaveAddr, ICM20948_GYRO_CONFIG_1, 1, &value, "Failed to read gyro config"))
        return false;
    // value &= 0b10001110; // TODO: report bug in pimoroni
    value &= 0b11000110; // TODO: report bug in pimoroni
    if (m_gyroLpf != ICM20948_GYRO_LPF_BYPASS) {
        value |= 0b1; // enable lpf
    }
    value |= (m_gyroLpf & 0x07) << 3; // TODO: report bug in pimoroni
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_GYRO_CONFIG_1, value, "Failed to write gyro config"))
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
    // // TODO: handle reserved bits in MSB
    uint16_t rate = (1125.0 / m_sampleRate) - 1;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_ACCEL_SMPLRT_DIV_1, (rate >> 8) & 0xFF, "Failed to write accelerometer MSB sample rate"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_ACCEL_SMPLRT_DIV_2, rate & 0xFF, "Failed to write accelerometer LSB sample rate"))
        return false;
    // m_settings->delayMs(100);
        
    // accelerometer fsr
    if (!m_settings->HALRead(m_slaveAddr, ICM20948_ACCEL_CONFIG, 1, &value, "Failed to read accelerometer config"))
        return false;
    std::cout << "read ACCEL_CONFIG as " << std::bitset<8>(value) << std::endl;
    value &= 0b11111001;
    value |= m_accelFsr << 1;
    std::cout << "setting ACCEL_CONFIG to " << std::bitset<8>(value) << std::endl;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_ACCEL_CONFIG, value, "Failed to write accelerometer config"))
        return false;
    // m_settings->delayMs(1);

    // accelerometer lpf
    if (!m_settings->HALRead(m_slaveAddr, ICM20948_ACCEL_CONFIG, 1, &value, "Failed to read accelerometer config"))
        return false;
    std::cout << "read ACCEL_CONFIG as " << std::bitset<8>(value) << std::endl;
    // value &= 0b10001110; // TODO: report bug in pimoroni
    value &= 0b11000110; // TODO: report bug in pimoroni
    // if (m_accelLpf != ICM20948_ACCEL_LPF_BYPASS) {
        value |= 0b1; // enable lpf
    // }
    value |= (m_accelLpf & 0x07) << 3; // TODO: report bug in pimoroni
    std::cout << "setting ACCEL_CONFIG to " << std::bitset<8>(value) << std::endl;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_ACCEL_CONFIG, value, "Failed to write accelerometer config"))
        return false;
    // m_settings->delayMs(1);

    return true;
}

bool RTIMUICM20948::setSampleRate()
{
    // if (m_sampleRate > 1000)
    //     return true;                                        // SMPRT not used above 1000Hz

    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_SMPRT_DIV, (unsigned char) (1000 / m_sampleRate - 1),
    //         "Failed to set sample rate"))
    //     return false;

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
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_DO, 0xff, "Failed to set magnetometer reg value to slave")) return false;
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


bool RTIMUICM20948::compassSetup() {
    std::cout << "COMPASS_SETUP" << std::endl;
    // if (!SelectRegisterBank(ICM20948_BANK0)) return false;
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0x20, "Failed to write user_ctrl reg")) return false;

    if (!SelectRegisterBank(ICM20948_BANK0)) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_INT_PIN_CFG, 0x30, "Failed to set I2C master mode")) return false;
    

    if (!SelectRegisterBank(ICM20948_BANK3)) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_CTRL, 0x4D, "Failed to set I2C master mode")) return false;
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_DELAY_CTRL, 0x83, "Failed to set I2C master mode"))
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_DELAY_CTRL, 0x01, "Failed to set I2C master mode")) return false;


    std::cout << "mag_read(AK09916_WHO_AM_I) " << int(mag_read(AK09916_WHO_AM_I)) << std::endl;

    mag_write(AK09916_CNTL3, 0x01);
    while (mag_read(AK09916_CNTL3) == 0x01) {
        usleep(1000);
    }
    std::cout << "COMPASS RESET!" << std::endl;
    // bypassOn();
    // if (!SelectRegisterBank(ICM20948_BANK3))
    //     return false;
    // if (!m_settings->HALWrite(AK09916_ADDRESS, AK09916_CNTL3, 0x01, "Failed to set reset compass")) {
    // // if (!m_settings->HALWrite(AK09916_ADDRESS, AK09916_CNTL2, 0x02, "Failed to set compass in power down mode")) {
    //     bypassOff();
    //     return false;
    // }
    // bypassOff();
    m_settings->delayMs(100);
    
    mag_write(AK09916_CNTL2, 0x01);
    m_settings->delayMs(100);
    
    std::cout << "mag_read(AK09916_WHO_AM_I) " << int(mag_read(AK09916_WHO_AM_I)) << std::endl;
    std::cout << "mag_read(AK09916_CNTL2) " << int(mag_read(AK09916_CNTL2)) << std::endl;
    std::cout << "mag_read(AK09916_CNTL3) " << int(mag_read(AK09916_CNTL3)) << std::endl;
    std::cout << "mag_read(AK09916_ST1) " << int(mag_read(AK09916_ST1)) << std::endl;
    std::cout << "mag_read(AK09916_ST2) " << int(mag_read(AK09916_ST2)) << std::endl;
    std::cout << "mag_read(AK09916_HXL) " << int(mag_read(AK09916_HXL)) << std::endl;


    if (!SelectRegisterBank(ICM20948_BANK3)) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80/*read*/, "Failed to set slave 0 address")) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_REG, AK09916_ST1, "Failed to set slave 0 reg")) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV0_CTRL, 0x88, "Failed to set slave 0 ctrl")) return false;
    
    
    if (!SelectRegisterBank(ICM20948_BANK3)) return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV1_ADDR, AK09916_I2C_ADDR /*write*/, "Failed to set slave 1 address"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV1_REG, AK09916_CNTL2, "Failed to set slave 1 reg"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV1_CTRL, 0x81, "Failed to set slave 1 ctrl"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV1_DO, 0x01, "Failed to set slave 1 DO"))
        return false;
    

    // if (!SelectRegisterBank(ICM20948_BANK0))
    //     return false;
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0x20, "Failed to write user_ctrl reg"))
    //     return false;

    // if (!SelectRegisterBank(ICM20948_BANK3))
    //     return false;
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_CTRL, 0x4D, "Failed to set I2C master mode"))
    //     return false;
    // // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_DELAY_CTRL, 0x83, "Failed to set I2C master mode"))
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_DELAY_CTRL, 0x01, "Failed to set I2C master mode"))
    //     return false;
    // if (!SelectRegisterBank(ICM20948_BANK0))
    //     return false;
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_USER_CTRL, 0x20, "Failed to write user_ctrl reg"))
    //     return false;
    
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_MST_DELAY_CTRL, 0x3, "Failed to set mst delay"))
        // return false;

    
    
    // m_settings->delayMs(10);
    // if (!mag_write(AK09916_CNTL3, 0x01))
    //     return false;
    // m_settings->delayMs(10);

    return true;
}

bool RTIMUICM20948::setCompassRate()
{
    if (!SelectRegisterBank(ICM20948_BANK3))
        return false;

    int rate;

    rate = m_sampleRate / m_compassRate - 1;

    if (rate > 31)
        rate = 31;
    // if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV4_CTRL, rate, "Failed to set slave ctrl 4"))
        //  return false;
    // TODO: put this in compass setup
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
    unsigned char fifoData[12];
    unsigned char compassData[8];

    if (!SelectRegisterBank(ICM20948_BANK3))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_I2C_SLV1_DO, 0x01, "Failed to set slave 1 DO"))
        return false;
        
    if (!SelectRegisterBank(ICM20948_BANK0)) return false;
    if (!m_settings->HALRead(m_slaveAddr, ICM20948_ACCEL_XOUT_H, ICM20948_FIFO_CHUNK_SIZE, fifoData, "Failed to read fifo data"))
        return false;
    trigger_mag_io();
    if (!SelectRegisterBank(ICM20948_BANK0)) return false;
    if (!m_settings->HALRead(m_slaveAddr, ICM20948_EXT_SLV_SENS_DATA_00, 8, compassData, "Failed to read compass data"))
        return false;
    

    if (!SelectRegisterBank(ICM20948_BANK2))
        return false;
    uint8_t a_scale;
    if (!m_settings->HALRead(m_slaveAddr, ICM20948_ACCEL_CONFIG, 1, &a_scale, "Failed to read fifo data"))
        return false;
    a_scale = (a_scale & 0x06) >> 1;
    // RTMath::convertToVector(fifoData, m_imuData.accel, ACCELFSR_MAP.at(a_scale), true);
    
    RTMath::convertToVector(fifoData, m_imuData.accel, m_accelScale, true);
    RTMath::convertToVector(fifoData + 6, m_imuData.gyro, m_gyroScale, true);
    RTMath::convertToVector(compassData + 1, m_imuData.compass, 0.15f, false);

    // std::cout << "GYRO: " << m_imuData.gyro.x() << " " << m_imuData.gyro.y() << " " << m_imuData.gyro.z() << " " <<  std::endl;
    // std::cout << "ACCEL: " << int(a_scale) << "  " << m_imuData.accel.x() << " " << m_imuData.accel.y() << " " << m_imuData.accel.z() << " " <<  std::endl;
    // std::cout << "MAG: " << int(compassData[3]) <<  std::endl;
    // std::cout << "MAG: " << "  " << m_imuData.compass.x() << " " << m_imuData.compass.y() << " " << m_imuData.compass.z() << " " <<  std::endl;
    
    //  sort out gyro axes

    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;

    m_imuData.accel.setX(-m_imuData.accel.x());

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
        m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
        // m_imuData.timestamp += m_sampleInterval;

    m_firstTime = false;

    //  now update the filter

    updateFusion();

    return true;
}


