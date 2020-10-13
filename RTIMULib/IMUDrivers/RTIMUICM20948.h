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


#ifndef _RTIMUICM20948_H
#define	_RTIMUICM20948_H

#include "RTIMU.h"

//  Define this symbol to use cache mode

//  FIFO transfer size

#define ICM20948_FIFO_CHUNK_SIZE     20 // gyro and accel are 12, compass 8

class RTIMUICM20948 : public RTIMU
{
public:
    RTIMUICM20948(RTIMUSettings *settings);
    ~RTIMUICM20948();

    bool setGyroLpf(unsigned char lpf);
    bool setAccelLpf(unsigned char lpf);
    bool setSampleRate(int rate);
    bool setCompassRate(int rate);
    bool setGyroFsr(unsigned char fsr);
    bool setAccelFsr(unsigned char fsr);

    bool SelectRegisterBank(uint8_t reg_bank);

    bool trigger_mag_io();
    bool mag_write(uint8_t reg, uint8_t value);
    bool mag_read_bytes(unsigned char* data, uint8_t length=1);
    uint8_t mag_read(uint8_t reg);

    virtual const char *IMUName() { return "ICM-20948"; }
    virtual int IMUType() { return RTIMU_TYPE_ICM20948; }
    virtual bool IMUInit();
    virtual bool IMURead();
    virtual int IMUGetPollInterval();

protected:

    RTFLOAT m_compassAdjust[3];                             // the compass fuse ROM values converted for use

private:
    bool setGyroConfig();
    bool setAccelConfig();
    bool setSampleRate();
    bool compassSetup();
    bool setCompassRate();
    bool resetFifo();
    bool bypassOn();
    bool bypassOff();

    bool m_firstTime;                                       // if first sample

    unsigned char m_slaveAddr;                              // I2C address of ICM20948

    uint8_t m_reg_bank = 255;                                     // last selected register bank

    unsigned char m_gyroLpf;                                // gyro low pass filter setting
    unsigned char m_accelLpf;                               // accel low pass filter setting
    int m_compassRate;                                      // compass sample rate in Hz
    unsigned char m_gyroFsr;
    unsigned char m_accelFsr;

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;


#ifdef ICM20948_CACHE_MODE

    ICM20948_CACHE_BLOCK m_cache[ICM20948_CACHE_BLOCK_COUNT]; // the cache itself
    int m_cacheIn;                                          // the in index
    int m_cacheOut;                                         // the out index
    int m_cacheCount;                                       // number of used cache blocks

#endif

};

#endif // _RTIMUICM20948_H
