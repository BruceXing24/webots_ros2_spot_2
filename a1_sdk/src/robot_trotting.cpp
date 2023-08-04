#include "rclcpp/rclcpp.hpp"
#include "iostream"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "common/unitreeRobot.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "control/BalanceCtrl.h"
#include "Gait/GaitWave.h"
#include "Gait/GaitGenerator.h"
#include "a1_msg/msg/joint_angles.hpp"
#include "a1_msg/msg/robot_states.hpp"
#include "a1_msg/msg/joint_state.hpp"
#include "a1_msg/msg/contact_detection.hpp"
#include "a1_msg/msg/feet_state.hpp"

class SendJointPosition : public rclcpp::Node
{

public:
    SendJointPosition() : Node("state_balance")
    {

        joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&SendJointPosition::joyCallback, this, std::placeholders::_1));
        estimator_subscription = this->create_subscription<a1_msg::msg::RobotStates>("/estimator/robot_state", 1, std::bind(&SendJointPosition::estiCallback, this, std::placeholders::_1));
        jointState_subscription = this->create_subscription<a1_msg::msg::JointState>("/joint_state", 1, std::bind(&SendJointPosition::jointState_Callback, this, std::placeholders::_1));
        contacts_subscription = this->create_subscription<a1_msg::msg::ContactDetection>("/estimator/contacts",1,std::bind(&SendJointPosition::contact_Callback, this, std::placeholders::_1));
        feetState_subscription = this->create_subscription<a1_msg::msg::FeetState>("/estimator/feet_state",1,std::bind(&SendJointPosition::feetState_Callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&SendJointPosition::publishJointAngles, this));
        joint_state_publisher = this->create_publisher<a1_msg::msg::JointState>("/cmd_joints",1);       

        pose.resize(4);
        pose = {0, 0, 0, 0};

  


        _pcdInit << -0.0, -0.0046, 0.327;
        _pcd = _pcdInit;
        _wvBody = Vec3(0.0, 0.0, 0.0);
        _RdInit = rpyToRotMat(0, 0, 0);

        _Kpp = Vec3(150, 150, 150).asDiagonal(); // 8.17 Kp
        _Kdp = Vec3(25, 25, 25).asDiagonal();    // 8.17 Kd

        _kpw = 200;                           // 8.22 kp
        _Kdw = Vec3(30, 30, 30).asDiagonal(); // 8.22 kD
        Limit<<-30,30;
        _KpSwing = Vec3(400, 400, 400).asDiagonal();
        _KdSwing = Vec3(10, 10, 10).asDiagonal();
        _gaitHeight = 0.08;

        Lowstate = new LowlevelState();
        Lowcmd = new LowlevelCmd();
        robotModel = new A1Robot();
        balCtrl = new BalanceCtrl(robotModel);
        gaitWave = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0));
        gaitGenerator = new GaitGenerator(gaitWave->getTstance(),gaitWave->getTswing(),robotModel->getFeetPosIdeal());

        for (int i = 0; i < 4; i++)
        {
            Lowcmd->setSimStanceGain(i);
            Lowcmd->setZeroDq(i);
            Lowcmd->setZeroTau(i);
        }

        for (int i = 0; i < 4; i++)
        {
            Lowstate->motorState[i * 3].q = 0.044;
            Lowstate->motorState[i * 3 + 1].q = 0.670;
            Lowstate->motorState[i * 3 + 2].q = -1.31;
        }
    }

    ~SendJointPosition()
    {
        delete Lowstate;
        delete Lowcmd;
        delete robotModel;
        delete balCtrl;
        delete gaitWave;
        delete gaitGenerator;
    }


    LowlevelState *Lowstate;
    LowlevelCmd *Lowcmd;
    A1Robot *robotModel;
    BalanceCtrl *balCtrl;
    WaveGenerator * gaitWave;
    GaitGenerator *gaitGenerator;
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        pose[0] += joy_msg->axes[0] / 100;
        pose[1] += joy_msg->axes[1] / 100;
        pose[2] += joy_msg->axes[6] / 100;
        pose[3] += joy_msg->axes[7] / 100;
        // std::cout << "pose==" << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3] << std::endl;
        std::cout << "pose==" << pose[0]  << std::endl;

        _vCmdGlobal << 0.1, 0 , 0;
        _wCmdGlobal << 0, 0, 0;



    }
    void estiCallback(const a1_msg::msg::RobotStates::SharedPtr robot_state)
    {
        // assume here is estminator,
        _posBody << robot_state->robot_pos_s[0], robot_state->robot_pos_s[1], robot_state->robot_pos_s[2];
        _velBody << robot_state->robot_vel_s[0], robot_state->robot_vel_s[1], robot_state->robot_vel_s[2];
        _wvBody << robot_state->robot_w_s[0], robot_state->robot_w_s[1], robot_state->robot_w_s[2];
        _B2G_RotMat = rpyToRotMat(robot_state->robot_ori_s[0], robot_state->robot_ori_s[1], robot_state->robot_ori_s[2]);
        _G2B_RotMat = _B2G_RotMat.transpose();
        _rpyBody << robot_state->robot_ori_s[0], robot_state->robot_ori_s[1], robot_state->robot_ori_s[2];

    }
    
    void feetState_Callback(const a1_msg::msg::FeetState feet_state)
    {   
        // assume here is estminator
        _posFeetGlobal << feet_state.feet_pos[0],feet_state.feet_pos[3],feet_state.feet_pos[6],feet_state.feet_pos[9],
                          feet_state.feet_pos[1],feet_state.feet_pos[4],feet_state.feet_pos[7],feet_state.feet_pos[10],
                          feet_state.feet_pos[2],feet_state.feet_pos[5],feet_state.feet_pos[8],feet_state.feet_pos[11];

        _velFeetGlobal << feet_state.feet_vel[0],feet_state.feet_vel[3],feet_state.feet_vel[6],feet_state.feet_vel[9],
                          feet_state.feet_vel[1],feet_state.feet_vel[4],feet_state.feet_vel[7],feet_state.feet_vel[10],
                          feet_state.feet_vel[2],feet_state.feet_vel[5],feet_state.feet_vel[8],feet_state.feet_vel[11];
        
        // std::cout<<"_posFeetGlobal=="<<_posFeetGlobal<<std::endl;
        // std::cout<<"_velFeetGlobal=="<<_velFeetGlobal<<std::endl;

    }

    void jointState_Callback(const a1_msg::msg::JointState joint_state)
    {   
        for (int i = 0; i < 12; i++)
        {
            Lowstate->motorState[i].q = joint_state.joints_angle[i];

            // std::cout<<" Lowstate->motorState[i].q=="<< Lowstate->motorState[i].q<<std::endl;
            // std::cout<<" joint_state.joints_angle[i]=="<< joint_state.joints_angle[i]<<std::endl;

            Lowstate->motorState[i].dq = joint_state.joints_velocity[i];
        }
    }

    void contact_Callback(const a1_msg::msg::ContactDetection contacts){

        for (int i=0;i <4;i++){
            _contact << contacts.contacts[0],contacts.contacts[1],contacts.contacts[2],contacts.contacts[3];
        }
    }

    void publishJointAngles()
    {
        _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * 0.02, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
        _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * 0.02, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));
        _pcd(2) = _pcdInit(2);
        float yaw = 0;
        _Rd = rpyToRotMat(0, 0, yaw)*_RdInit; //期望的Rd
        _yaw = _rpyBody[2];
        _dYaw =_wvBody[2];

        _posFeet2BGlobal = robotModel->getFeet2BPositions(*Lowstate,FrameType::GLOBAL);
        // std::cout<<"_posFeet2BGlobal=="<<_posFeet2BGlobal<<std::endl;  ✓
        // std::cout<<"_posFeetGlobal=="<<_posFeetGlobal<<std::endl;      ✓
        // std::cout<<"_velFeetGlobal=="<<_velFeetGlobal<<std::endl;      ✓
        // std::cout<<"_B2G_RotMat=="<<_B2G_RotMat<<std::endl;            ✓
        // std::cout<<"_yaw=="<<_yaw<<std::endl;                          ✓
        // std::cout<<"_dYaw=="<<_dYaw<<std::endl;                        ✓


        gaitWave->calcContactPhase(phase_result,contact_result,WaveStatus::WAVE_ALL);
        gaitGenerator->_phase = &phase_result;
        gaitGenerator->_contact = &contact_result;
        // std::cout<<"contact_result=="<<contact_result<<std::endl;
        // _contact = contact_result;

        gaitGenerator->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);  //计算得到足端轨迹
        gaitGenerator->run(_posFeetGlobalGoal, _velFeetGlobalGoal,_posFeetGlobal,_velBody,_yaw,_dYaw,_posBody);
        
        
        // std::cout<<"_posFeetGlobalGoal=="<<_posFeetGlobalGoal<<std::endl;
        // std::cout<<"_velFeetGlobalGoal=="<<_velFeetGlobalGoal<<std::endl;




        calcTau();
        calcQQd(); 

    //  startTime = getSystemTime();
    // _ctrlComp->sendRecv();
    // _ctrlComp->runWaveGen();
    // _ctrlComp->estimator->run();
    // if(!checkSafty()){
    //     _ctrlComp->ioInter->setPassive();
    // }
    // //repeat current condition 
    // if(_mode == FSMMode::NORMAL){
    //     _currentState->run();

    }



    void calcTau()
    {   

        _ddPcd = _Kpp * (_pcd - _posBody) + _Kdp * (_vCmdGlobal - _velBody);
        _dWbd = _kpw * rotMatToExp(_Rd * _G2B_RotMat) + _Kdw * (Vec3(0, 0, 0) - _wvBody);
        // std::cout<<"_ddPcd=="<<_ddPcd<<std::endl;
        // std::cout<<"_dWbd=="<<_dWbd<<std::endl;

        _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
        _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
        _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

        _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
        _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
        _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));
        // std::cout<<"_contact=="<<_contact<<std::endl;

        _forceFeetGlobal = - balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, contact_result);  //阻断对外界的作用力

        _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
        // std::cout<<"_forceFeetBody=="<<_forceFeetBody<<std::endl;
        // std::cout<<"_B2G_RotMat=="<<_B2G_RotMat<<std::endl;


        for(int i(0); i<4; ++i){
            if((_contact)(i) == 0){
                _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
            }
        }
        
        _q = vec34ToVec12(Lowstate->getQ());
        // std::cout<<"_q"<<_q<<std::endl;

        _tau = robotModel->getTau(_q, _forceFeetBody);

        // std::cout<<"_tau"<<_tau<<std::endl;

    }

    void calcQQd(){

        Vec34 _posFeet2B;
        _posFeet2B = robotModel->getFeet2BPositions(*Lowstate,FrameType::BODY);
        // std::cout<<"_posFeet2B=="<<_posFeet2B<<std::endl;

        for(int i(0); i<4; ++i){
            _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
            _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
            // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
        }

        _qGoal = robotModel->getQ(_posFeet2BGoal, FrameType::BODY);      // 各个关节的目标角度
        _qdGoal = robotModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY); // 各个关节的目标角速度        
        _dq = vec34ToVec12(Lowstate->getQd());

        // std::cout<<"_qGoal"<<_qGoal<<std::endl;
        // std::cout<<"_qdGoal"<<_qdGoal<<std::endl;
        // std::cout<<"_dq"<<_dq<<std::endl;

        // std::cout<<"_velFeet2BGoal=="<<_velFeet2BGoal<<std::endl;


        for (int i = 0; i<12; i++){
                cmd_joints.joints_torque[i] = _tau[i];
                cmd_joints.joints_angle[i] = _qGoal[i];
                cmd_joints.joints_velocity[i] = _qdGoal[i];
        }

        joint_state_publisher->publish(cmd_joints);
    }




private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
    rclcpp::Subscription<a1_msg::msg::RobotStates>::SharedPtr estimator_subscription;
    rclcpp::Subscription<a1_msg::msg::JointState>::SharedPtr jointState_subscription;
    rclcpp::Subscription<a1_msg::msg::ContactDetection>::SharedPtr contacts_subscription;
    rclcpp::Subscription<a1_msg::msg::FeetState>::SharedPtr feetState_subscription;

    rclcpp::Publisher<a1_msg::msg::JointState>::SharedPtr joint_state_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<float> pose;

    VecInt4 _contact,contact_result;

    Vec4 phase_result;
    a1_msg::msg::JointState cmd_joints;
    RotMat _Rd, _RdInit;
    Vec3 _pcd, _pcdInit;
    double _kpw;
    Mat3 _Kpp, _Kdp, _Kdw;
    Vec3 _ddPcd, _dWbd;

    Vec12 _q,_dq, _tau;
    Vec3 _posBody, _velBody, _wvBody, _rpyBody;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec34 _posFeet2BGlobal;
    Vec34 _forceFeetGlobal, _forceFeetBody;
    Vec34 _posFeetGlobal,_velFeetGlobal;
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34 _posFeet2BGoal,_velFeet2BGoal;
    Vec12 _qGoal,_qdGoal;
    Vec2 Limit;
    double _yaw,_dYaw, _gaitHeight;
    Vec3 _vCmdGlobal,_wCmdGlobal;
    Mat3 _KpSwing, _KdSwing;

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SendJointPosition>();
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



// *_contact==0
// 1
// 1
// 0
// phase====0.032
// 0.032
// 0.032
// 0.032
// phase====0.561
// 0.561
// 0.561
// 0.561
// *_contact==0
// 1
// 1
// 0


// phase====0.967
// 0.967
// 0.967
// 0.967
// *_contact==0
// 1
// 1
// 0


// phase====0.432
// 0.432
// 0.432
// 0.432
// *_contact==1
// 0
// 0
// 1
