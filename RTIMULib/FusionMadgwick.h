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
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#ifndef _RTFUSIONMADGWICK_H
#define	_RTFUSIONMADGWICK_H

#include "RTFusion.h"

class FusionMadgwick : public RTFusion
{
public:
    FusionMadgwick();
    ~FusionMadgwick();

    //  fusionType returns the type code of the fusion algorithm

    virtual int fusionType() { return RTFUSION_TYPE_MADGWICK; }

    //  reset() resets the state but keeps any setting changes (such as enables)

    void reset();

    //  newIMUData() should be called for subsequent updates
    //  deltaTime is in units of seconds

    void newIMUData(RTIMU_DATA& data, const RTIMUSettings *settings);

private:

    void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
    void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
};

#endif // _RTFUSIONMADGWICK_H
