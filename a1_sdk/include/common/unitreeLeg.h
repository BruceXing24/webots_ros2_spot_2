/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREELEG_H
#define UNITREELEG_H

#include "common/mathTypes.h"
#include "common/enumClass.h"

class QuadrupedLeg{
public:
    QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength, 
                 float kneeLinkLength, Vec3 pHip2B);
    ~QuadrupedLeg(){}
    Vec3 calcPEe2H(Vec3 q); //foot to hip
    Vec3 calcPEe2B(Vec3 q); //end to body
    Vec3 calcVEe(Vec3 q, Vec3 qd);  // velocity  foot to end 
    Vec3 calcQ(Vec3 pEe, FrameType frame);  // inverse kinematic frame type = HIP  frame type = Body  which coor is used 
    Vec3 calcQd(Vec3 q, Vec3 vEe);   // acoording to angle q and foot velocity to calculate angular speed 
    Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame); //under frame xxx postion pEe and velocity vEe, to calculate three joint angular spped 
    Vec3 calcTau(Vec3 q, Vec3 force);
    Mat3 calcJaco(Vec3 q);
    Vec3 getHip2B(){return _pHip2B;}
protected:
    float q1_ik(float py, float pz, float b2y);
    float q3_ik(float b3z, float b4z, float b);
    float q2_ik(float q1, float q3, float px, 
                float py, float pz, float b3z, float b4z);
    float _sideSign;
    const float _abadLinkLength, _hipLinkLength, _kneeLinkLength;
    const Vec3 _pHip2B;
};

class A1Leg : public QuadrupedLeg{
public:
    A1Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.0838, 0.2, 0.2, pHip2B){}
    ~A1Leg(){}
};

class Go1Leg : public QuadrupedLeg{
public:
    Go1Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.08, 0.213, 0.213, pHip2B){}
    ~Go1Leg(){}
};

#endif  // UNITREELEG_H