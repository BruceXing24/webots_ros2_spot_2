/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef GAITWAVE_H
#define GAITWAVE_H

#include "common/mathTypes.h"
#include "common/enumClass.h"
#include <unistd.h>



/*generate linear wave, [0, 1]*/
class WaveGenerator{
public:
    WaveGenerator(double period, double stancePhaseRatio, Vec4 bias);
    ~WaveGenerator();
    void calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult, WaveStatus status);
    
    void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status);
    void getPassTime();
    float getTstance();//
    float getTswing();   // 腾空时间
    float getT();
    double _passT; 
private:
    double _period;
    double _stRatio;

    Vec4 _bias;
    Vec4 _normalT;                   // [0, 1)
    Vec4 _phase, _phasePast;
    VecInt4 _contact, _contactPast;
    VecInt4 _switchStatus;          // 1: switching, 0: do not switch
    WaveStatus _statusPast;

                     // unit: second
    long long _startT;    // unit: us


};

#endif  // WAVEGENERATOR_H