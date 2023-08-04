/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H

#include "common/mathTypes.h"
#include "common/enumClass.h"


/*cycloid gait*/
class GaitGenerator{
public:
    GaitGenerator(float Tstance,float Tswing, Vec34 feetPosBody);
    ~GaitGenerator();
    void setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight);
    void run(Vec34 &feetPos, Vec34 &feetVel,Vec34 _feetPos,Vec3 _bodyVelGlobal,float _yaw,float _dYaw,Vec3 _footPos);
    Vec3 getFootPos(int i);
    Vec3 getFootVel(int i);
    void restart();

    Vec4 *_phase, _phasePast;
    VecInt4 *_contact;
    Vec3 calFootPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase,Vec3 _bodyVelGlobal,float _yaw,float _dYaw,Vec3 _footPos);

private:
    float cycloidXYPosition(float startXY, float endXY, float phase);
    float cycloidXYVelocity(float startXY, float endXY, float phase);
    float cycloidZPosition(float startZ, float height, float phase);
    float cycloidZVelocity(float height, float phase);


    float _gaitHeight;
    Vec2 _vxyGoal;
    float _dYawGoal;
    // Vec4 *_phase, _phasePast;
    // VecInt4 *_contact;
    Vec34 _startP, _endP, _idealP, _pastP, _feetPosBody;
    bool _firstRun;


    Vec3 _nextStep, _footPos;                                                                                                                                                                   
    Vec3 _bodyVelGlobal;        // linear velocity
    Vec3 _bodyAccGlobal;        // linear accelerator
    Vec3 _bodyWGlobal;          // angular velocity

    Vec4 _feetRadius, _feetInitAngle;
    float _yaw, _dYaw, _nextYaw;

    float _Tstance, _Tswing;
    float _kx, _ky, _kyaw;







};

#endif  // GAITGENERATOR_H