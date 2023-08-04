/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/GaitGenerator.h"

GaitGenerator::GaitGenerator(float Tstance,float Tswing, Vec34 feetPosBody):
_Tstance(Tstance),_Tswing(Tswing),_feetPosBody(feetPosBody)
{
    _firstRun = true;

    // _Tstance  = ctrlComp->waveGen->getTstance();
    // _Tswing   = ctrlComp->waveGen->getTswing();
    // Vec34 feetPosBody = _robModel->getFeetPosIdeal();


    _kx = 0.005;
    _ky = 0.005;
    _kyaw = 0.005;

    for(int i(0); i<4; ++i){
        _feetRadius(i)    = sqrt( pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2) );
        _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i));
    }
    _contact = new VecInt4;
    (*_contact) << 1, 1, 1, 1;
}

GaitGenerator::~GaitGenerator(){
}

void GaitGenerator::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight){
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator::restart(){
    _firstRun = true;
    _vxyGoal.setZero();
}

void GaitGenerator::run(Vec34 &feetPos, Vec34 &feetVel,Vec34 _feetPos,Vec3 _bodyVelGlobal,float _yaw,float _dYaw,Vec3 _footPos){
    if(_firstRun){
        // _startP = _est->getFeetPos();
        _startP = _feetPos;
        _firstRun = false;
    }
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 1){
            if((*_phase)(i) < 0.5){
                _startP.col(i) = _feetPos.col(i);
            }
            feetPos.col(i) = _startP.col(i);
            feetVel.col(i).setZero();
        }
        else{
            _endP.col(i) = calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i),_bodyVelGlobal,_yaw,_dYaw,_footPos);
            feetPos.col(i) = getFootPos(i);
            feetVel.col(i) = getFootVel(i);
            // std::cout<<"swing calc=="<<i<<std::endl;
        }

    // std::cout<<"feetPos==="<<feetPos<<std::endl;
    // std::cout<<"feetVel==="<<feetVel<<std::endl;
    // std::cout<<"Tswing time ==="<<_waveG->getTswing()<<std::endl;

    }
    _pastP = feetPos;
    _phasePast = *_phase;
    // 运行一次phase 增加一次
}

Vec3 GaitGenerator::getFootPos(int i){
    Vec3 footPos;

    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    
    return footPos;
}

Vec3 GaitGenerator::getFootVel(int i){
    Vec3 footVel;

    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i));

    return footVel;
}

float GaitGenerator::cycloidXYPosition(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start;
}

float GaitGenerator::cycloidXYVelocity(float start, float end, float phase){
    // std::cout<<"_Tswing=="<<_Tswing<<std::endl;
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _Tswing;
}

float GaitGenerator::cycloidZPosition(float start, float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*(1 - cos(phasePI))/2 + start;
}

float GaitGenerator::cycloidZVelocity(float h, float phase){
    // std::cout<<"_Tswing=="<<_Tswing<<std::endl;
    float phasePI = 2 * M_PI * phase;
    return h*M_PI * sin(phasePI) / _Tswing;
}



Vec3 GaitGenerator::calFootPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase,Vec3 _bodyVelGlobal,float _yaw,float _dYaw,Vec3 _footPos){
    // _bodyVelGlobal = _est->getVelocity();
    // _bodyWGlobal = _lowState->getGyroGlobal();

    _nextStep(0) = _bodyVelGlobal(0)*(1-phase)*_Tswing + _bodyVelGlobal(0)*_Tstance/2 + _kx*(_bodyVelGlobal(0) - vxyGoalGlobal(0));
    _nextStep(1) = _bodyVelGlobal(1)*(1-phase)*_Tswing + _bodyVelGlobal(1)*_Tstance/2 + _ky*(_bodyVelGlobal(1) - vxyGoalGlobal(1));
    _nextStep(2) = 0;

    // _yaw = _lowState->getYaw();
    // _dYaw = _lowState->getDYaw();
    _nextYaw = _dYaw*(1-phase)*_Tswing + _dYaw*_Tstance/2 + _kyaw*(dYawGoal - _dYaw);

    _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw);
    _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw);
    
    _footPos = _footPos+_nextStep;
    // _footPos = _est->getPosition() + _nextStep;  _est->getPosition() return robot center coordinate
    _footPos(2) = 0.0;
    return _footPos;

}