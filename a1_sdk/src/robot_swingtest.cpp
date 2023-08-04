#include "rclcpp/rclcpp.hpp"
#include "iostream"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "common/unitreeRobot.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "control/BalanceCtrl.h"
#include "a1_msg/msg/robot_states.hpp"
#include "a1_msg/msg/joint_state.hpp"
#include "a1_msg/msg/contact_detection.hpp"
// #include "gait/GaitWave.h"




class SendJointPosition : public rclcpp::Node
{
public:
    SendJointPosition() : Node("state_balance")
    {

        joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&SendJointPosition::joyCallback, this, std::placeholders::_1));
        estimator_subscription = this->create_subscription<a1_msg::msg::RobotStates>("/estimator/robot_state", 10, std::bind(&SendJointPosition::estiCallback, this, std::placeholders::_1));
        jointState_subscription = this->create_subscription<a1_msg::msg::JointState>("/joint_state", 10, std::bind(&SendJointPosition::jointState_Callback, this, std::placeholders::_1));
        contacts_subscription = this->create_subscription<a1_msg::msg::ContactDetection>("/estimator/contacts",10,std::bind(&SendJointPosition::contact_Callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&SendJointPosition::publishJointAngles, this));
        joint_state_publisher = this->create_publisher<a1_msg::msg::JointState>("/cmd_joints",10);       


        _xMin = -0.15;
        _xMax =  0.10;
        _yMin = -0.15;
        _yMax =  0.15;
        _zMin = -0.05;
        _zMax =  0.20;

        Lowstate = new LowlevelState();
        Lowcmd = new LowlevelCmd();
        robotModel = new A1Robot();
        balCtrl = new BalanceCtrl(robotModel);
        // gait  = new FeetEndCal(robotModel);

        _Kp = Vec3(20, 20, 50).asDiagonal();
        _Kd = Vec3( 5,  5, 20).asDiagonal();

        pose.resize(3);
        Limit<< -40,40;
        _tau.setZero();

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

            Lowstate->motorState[i * 3].dq = 0.;
            Lowstate->motorState[i * 3 + 1].dq = 0.;
            Lowstate->motorState[i * 3 + 2].dq = 0.;
        }

        _initFeetPos = robotModel->getFeet2BPositions(*Lowstate, FrameType::HIP);
        _feetPos = _initFeetPos;
        _initPos = _initFeetPos.col(0);
        _posGoal = _initPos;

    }

    ~SendJointPosition()
    {
        delete Lowstate;
        delete Lowcmd;
        delete robotModel;
        delete balCtrl;
    }


    LowlevelState *Lowstate;
    LowlevelCmd *Lowcmd;
    A1Robot *robotModel;
    BalanceCtrl *balCtrl;
    // FeetEndCal *gait;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        pose[0] += joy_msg->axes[0] / 100;
        pose[1] += joy_msg->axes[1] / 100;
        pose[2] += joy_msg->axes[6] / 100;
        std::cout << "pose==" << pose[0] << " " << pose[1] << " " << pose[2] << " " << std::endl;

    if(pose[0] > 0){
        _posGoal(0) = invNormalize(pose[0], _initPos(0), _initPos(0)+_xMax, 0, 1);
    }else{
        _posGoal(0) = invNormalize(pose[0], _initPos(0)+_xMin, _initPos(0), -1, 0);
    }
    
    if(pose[1] > 0){
        _posGoal(1) = invNormalize(pose[1], _initPos(1, 0), _initPos(1)+_yMax, 0, 1);
    }else{
        _posGoal(1) = invNormalize(pose[1], _initPos(1)+_yMin, _initPos(1), -1, 0);
    }

    if(pose[2] > 0){
        _posGoal(2) = invNormalize(pose[2], _initPos(2), _initPos(2)+_zMax, 0, 1);
    }else{
        _posGoal(2) = invNormalize(pose[2], _initPos(2)+_zMin, _initPos(2), -1, 0);
    }

    }
    void estiCallback(const a1_msg::msg::RobotStates::SharedPtr robot_state)
    {
        // _posBody << robot_state->robot_pos_s[0], robot_state->robot_pos_s[1], robot_state->robot_pos_s[2];
        // _velBody << robot_state->robot_vel_s[0], robot_state->robot_vel_s[1], robot_state->robot_vel_s[2];
        // _wvBody << robot_state->robot_w_s[0], robot_state->robot_w_s[1], robot_state->robot_w_s[2];
        // _B2G_RotMat = rpyToRotMat(robot_state->robot_ori_s[0], robot_state->robot_ori_s[1], robot_state->robot_ori_s[2]);
        // _G2B_RotMat = _B2G_RotMat.transpose();
    }

    void jointState_Callback(const a1_msg::msg::JointState joint_state)
    {   
        for (int i = 0; i < 12; i++)
        {
            Lowstate->motorState[i].q = joint_state.joints_angle[i];
            Lowstate->motorState[i].dq = joint_state.joints_velocity[i];
        }
        // _posFeet2BGlobal = robotModel->getFeet2BPositions(*Lowstate,FrameType::GLOBAL);
        // _velFeet2BGlobal = robotModel->getFeet2BVelocities(*Lowstate,FrameType::GLOBAL);
    }


    void contact_Callback(const a1_msg::msg::ContactDetection contacts){
        // for (int i=0;i <4;i++){
        //     _contact << contacts.contacts[0],contacts.contacts[1],contacts.contacts[2],contacts.contacts[3];
        // }
    }

    void publishJointAngles()
    {   
        _positionCtrl();
    }


    void _positionCtrl(){
    _feetPos.col(0) = _posGoal;
    _targetPos = robotModel->getQ(_feetPos, FrameType::HIP);
    // Lowcmd->setQ(_targetPos);
    std::cout<<"_targetPos="<<_targetPos<<std::endl;
    for (int i = 0; i < 3; i++)
    {
    std::cout<<"Lowstate="<<Lowstate->motorState[i].q<<std::endl;    
    }





    Vec3 pos0 = robotModel->getFootPosition(*Lowstate, 0, FrameType::HIP);
    Vec3 vel0 = robotModel->getFootVelocity(*Lowstate, 0);
    // std::cout<<"vel0="<<vel0<<std::endl;

    Vec3 force0 = _Kp*(_posGoal - pos0) + _Kd*(-vel0);

    std::cout<<"_posGoal="<<_posGoal<<std::endl;
    std::cout<<"pos0="<<pos0<<std::endl;

    // std::cout<<"force0="<<force0<<std::endl;

    Vec12 torque;
    Mat3 jaco0 = robotModel->getJaco(*Lowstate, 0);
    // std::cout<<"jaco0="<<jaco0<<std::endl;

    torque.segment(0, 3) = jaco0.transpose() * force0;

    std::cout<<"torque="<<torque.segment(0, 3)<<std::endl;


    // for (int i = 0; i<3; i++){
    //     _tau[i] = motor_model(torque[i],_targetPos[i],Lowstate->motorState[i].q, 0.0,Lowstate->motorState[i].dq );
    // };
    std::cout<<"_tau"<<_tau<<std::endl;
    for (int i = 0; i<12; i++){
            cmd_joints.joints_torque[i] = torque[i];
            cmd_joints.joints_angle[i] = _targetPos[i];
            cmd_joints.joints_velocity[i] = 0.;

    }
    calcTau();
}



    void calcTau()
    {

        joint_state_publisher->publish(cmd_joints);
    }


    double motor_model(float _tauff,float _qGoal,float _qCur,float _qdGoal,float _qdCur){
        // std::cout<<"_qGoal="<<_qGoal<<std::endl;
        // std::cout<<"_qCur="<<_qCur<<std::endl;

        float Kp =10.;
        float Kd =2.;
        float _tau_result;
        _tau_result = _tauff + Kp*(_qGoal-_qCur)+ Kd*(_qdGoal-_qdCur);
        return _tau_result;
    }



private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
    rclcpp::Subscription<a1_msg::msg::RobotStates>::SharedPtr estimator_subscription;
    rclcpp::Subscription<a1_msg::msg::JointState>::SharedPtr jointState_subscription;
    rclcpp::Subscription<a1_msg::msg::ContactDetection>::SharedPtr contacts_subscription;
    rclcpp::Publisher<a1_msg::msg::JointState>::SharedPtr joint_state_publisher;

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<float> pose;
    Vec34 _initFeetPos, _feetPos;
    Vec3  _initPos, _posGoal;
    Vec12 _targetPos;
    float _xMin, _xMax;
    float _yMin, _yMax;
    float _zMin, _zMax;
    Mat3 _Kp, _Kd;

    Vec12 _q, _tau;
    Vec2 Limit;
    a1_msg::msg::JointState cmd_joints;

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