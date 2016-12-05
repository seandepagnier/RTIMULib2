////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2016 Sean D'Epagnier
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
//  PARTICULAR PURPOpSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#ifndef _RTFUSIONMAHONY_H
#define	_RTFUSIONMAHONY_H

#include "RTFusion.h"

class FusionMahony : public RTFusion
{
public:
    FusionMahony();
    ~FusionMahony();

    //  fusionType returns the type code of the fusion algorithm

    virtual int fusionType() { return RTFUSION_TYPE_MAHONY; }

    //  reset() resets the state but keeps any setting changes (such as enables)

    void reset();

    //  newIMUData() should be called for subsequent updates
    //  deltaTime is in units of seconds

    void newIMUData(RTIMU_DATA& data, const RTIMUSettings *settings);

private:

    void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
    void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    float integralFBx,  integralFBy, integralFBz;	// integral error terms scaled by Ki
    float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
};

#endif // _RTFUSIONMAHONY_H
