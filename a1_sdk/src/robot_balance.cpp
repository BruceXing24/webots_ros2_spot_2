#include "rclcpp/rclcpp.hpp"
#include "iostream"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "common/unitreeRobot.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "control/BalanceCtrl.h"
#include "a1_msg/msg/joint_angles.hpp"
#include "a1_msg/msg/robot_states.hpp"
#include "a1_msg/msg/joint_state.hpp"
#include "a1_msg/msg/contact_detection.hpp"
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

        pose.resize(4);
        pose = {0, 0, 0, 0};

        _xMax = 0.05;
        _xMin = -_xMax;
        _yMax = 0.05;
        _yMin = -_yMax;
        _zMax = 0.04;
        _zMin = -_zMax;
        _yawMax = 20 * M_PI / 180;
        _yawMin = -_yawMax;
        _contact_test<<1,1,1,1;

// - -0.14561888575553894
// - -0.0046030739322304726
// - 0.32700657844543457


        _pcdInit << -0.0, -0.0046, 0.327;
        _pcd = _pcdInit;
        _wvBody = Vec3(0.0, 0.0, 0.0);
        _RdInit = rpyToRotMat(0, 0, 0);

        _Kpp = Vec3(150, 150, 150).asDiagonal(); // 8.17 Kp
        _Kdp = Vec3(25, 25, 25).asDiagonal();    // 8.17 Kd

        _kpw = 200;                           // 8.22 kp
        _Kdw = Vec3(30, 30, 30).asDiagonal(); // 8.22 kD
        Limit<<-30,30;


        Lowstate = new LowlevelState();
        Lowcmd = new LowlevelCmd();
        robotModel = new A1Robot();
        balCtrl = new BalanceCtrl(robotModel);


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
    }


    LowlevelState *Lowstate;
    LowlevelCmd *Lowcmd;
    A1Robot *robotModel;

    BalanceCtrl *balCtrl;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        // my home joysticker
        // joint_angles[0] += joy_msg->axes[0];
        // joint_angles[1] += joy_msg->axes[1];
        // joint_angles[2] = 0.0;
        // joint_angles[3] = 0.0;

        pose[0] += joy_msg->axes[0] / 100;
        pose[1] += joy_msg->axes[1] / 100;
        pose[2] += joy_msg->axes[6] / 100;
        pose[3] += joy_msg->axes[7] / 100;
        std::cout << "pose==" << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3] << std::endl;
    }
    void estiCallback(const a1_msg::msg::RobotStates::SharedPtr robot_state)
    {
        _posBody << robot_state->robot_pos_s[0], robot_state->robot_pos_s[1], robot_state->robot_pos_s[2];
        _velBody << robot_state->robot_vel_s[0], robot_state->robot_vel_s[1], robot_state->robot_vel_s[2];
        _wvBody << robot_state->robot_w_s[0], robot_state->robot_w_s[1], robot_state->robot_w_s[2];
        _B2G_RotMat = rpyToRotMat(robot_state->robot_ori_s[0], robot_state->robot_ori_s[1], robot_state->robot_ori_s[2]);
        _G2B_RotMat = _B2G_RotMat.transpose();
        // _G2B_RotMat = rpyToRotMat(robot_state->robot_ori_s[0], robot_state->robot_ori_s[1], robot_state->robot_ori_s[2]);
        // _B2G_RotMat= _G2B_RotMat.transpose();


    }

    void jointState_Callback(const a1_msg::msg::JointState joint_state)
    {   
        for (int i = 0; i < 12; i++)
        {
            Lowstate->motorState[i].q = joint_state.joints_angle[i];
        }
    }

    void contact_Callback(const a1_msg::msg::ContactDetection contacts){

        for (int i=0;i <4;i++){
            _contact << contacts.contacts[0],contacts.contacts[1],contacts.contacts[2],contacts.contacts[3];
        }
    }

    void publishJointAngles()
    {
        // auto message = std::make_shared<std_msgs::msg::Float32MultiArray>();
        a1_msg::msg::JointAngles message;
        _pcd(0) = _pcdInit(0) + invNormalize(pose[0], _xMin, _xMax);
        _pcd(1) = _pcdInit(1) - invNormalize(pose[1], _yMin, _yMax);
        _pcd(2) = _pcdInit(2) + invNormalize(pose[2], _zMin, _zMax);
        float yaw = invNormalize(pose[3], _yawMin, _yawMax);
        _Rd = rpyToRotMat(0, 0, yaw)*_RdInit; //期望的Rd
        calcTau();
    }



    void calcTau()
    {
        _ddPcd = _Kpp * (_pcd - _posBody) + _Kdp * (Vec3(0, 0, 0) - _velBody);
        _dWbd = _kpw * rotMatToExp(_Rd * _G2B_RotMat) + _Kdw * (Vec3(0, 0, 0) - _wvBody);
        _posFeet2BGlobal = robotModel->getFeet2BPositions(*Lowstate,FrameType::GLOBAL);



        _forceFeetGlobal = - balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, _contact);  //阻断对外界的作用力
        _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;


        _q = vec34ToVec12(Lowstate->getQ());
        _tau = robotModel->getTau(_q, _forceFeetBody);

        for (int i = 0; i<12; i++){
            _tau[i] = saturation(_tau(i),Limit);
            cmd_joints.joints_torque[i] = _tau[i];
        }
        // std::cout<< "_pcd==" << _pcd<<std::endl;
        // std::cout<< "_posBody==" << _posBody<<std::endl;   
        // std::cout<< "_velBody==" << _velBody<<std::endl;

        // std::cout<< "_Rd==" << _Rd<<std::endl;
        // std::cout<< "_ddPcd==" << _ddPcd <<std::endl;
        // std::cout<< "_dWbd==" << _dWbd <<std::endl;

        // std::cout<<"_B2G_RotMat"<<_B2G_RotMat<<std::endl;        
        // std::cout<< "_posFeet2BGlobal=="<<_posFeet2BGlobal <<std::endl;
        // std::cout<<"contact"<<_contact<<std::endl;
        std::cout<< "_forceFeetGlobal=="<<_forceFeetBody <<std::endl;
        // std::cout<<"_tau==="<<_tau<<std::endl;
 
        joint_state_publisher->publish(cmd_joints);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
    rclcpp::Subscription<a1_msg::msg::RobotStates>::SharedPtr estimator_subscription;
    rclcpp::Subscription<a1_msg::msg::JointState>::SharedPtr jointState_subscription;
    rclcpp::Subscription<a1_msg::msg::ContactDetection>::SharedPtr contacts_subscription;
    rclcpp::Publisher<a1_msg::msg::JointState>::SharedPtr joint_state_publisher;

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<float> pose;

    VecInt4 _contact;
    VecInt4 _contact_test;
    a1_msg::msg::JointState cmd_joints;
    RotMat _Rd, _RdInit;
    Vec3 _pcd, _pcdInit;
    double _kpw;
    Mat3 _Kpp, _Kdp, _Kdw;
    Vec3 _ddPcd, _dWbd;

    Vec12 _q, _tau;
    Vec3 _posBody, _velBody, _wvBody;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec34 _posFeet2BGlobal;
    Vec34 _forceFeetGlobal, _forceFeetBody;

    float _xMax, _xMin;
    float _yMax, _yMin;
    float _zMax, _zMin;
    float _yawMax, _yawMin;
    Vec2 Limit;
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