#include "rclcpp/rclcpp.hpp"
#include "iostream"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "common/unitreeRobot.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "a1_msg/msg/joint_angles.hpp"

class SendJointPosition: public rclcpp::Node
{

    public:
    SendJointPosition():Node("send_joints_angle"){
        // joint_angles_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("/robot_joints",10);

        joint_angles_publisher = this->create_publisher<a1_msg::msg::JointAngles>("/robot_joints",10);       

        joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>("joy",10,std::bind (&SendJointPosition::joyCallback,
        this,std::placeholders::_1) );
        pose.resize(4); 
        pose = {0,0,0,0};
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&SendJointPosition::publishJointAngles,this));

        _rowMax = 20 * M_PI / 180;
        _rowMin = -_rowMax;
        _pitchMax = 15 * M_PI / 180;
        _pitchMin = -_pitchMax;
        _yawMax = 20 * M_PI / 180;
        _yawMin = -_yawMax;
        _heightMax = 0.04;
        _heightMin = -_heightMax;

        Lowstate = new LowlevelState();
        Lowcmd = new LowlevelCmd();
        robotModel=new A1Robot();
        for(int i=0; i<4; i++){
            Lowcmd->setSimStanceGain(i);
            Lowcmd->setZeroDq(i);
            Lowcmd->setZeroTau(i);
        }
        for(int i=0; i<4; i++){
            Lowstate->motorState[i*3].q = 0.044;
            Lowstate->motorState[i*3+1].q = 0.670;
            Lowstate->motorState[i*3+2].q = -1.31;
        }
        _initVecOX = robotModel->getX(*Lowstate);
        _initVecXP = robotModel->getVecXP(*Lowstate);


        //_initVecOX = robotModel->getX(（LowlevelState::LowlevelState*）&Lowstate);
        //_initVecXP = robotModel->getVecXP(（LowlevelState::LowlevelState*)&Lowstate);
        // initVecOX[0] = 0.175;
        // initVecOX[1] = -0.126;
        // initVecOX[2] = -0.338;

        // initVecXP << 0, 0,     -0.358,   -0.358,
        //              0, 0.252,  0,       0.252,
        //              0, 0,      0.002,   0.002;



    }
    ~SendJointPosition(){
        delete Lowstate;
        delete Lowcmd;
        delete robotModel;
            }


    LowlevelState *Lowstate;
    LowlevelCmd *Lowcmd;
    A1Robot *robotModel;


        


 void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
    // my home joysticker
    // joint_angles[0] += joy_msg->axes[0];
    // joint_angles[1] += joy_msg->axes[1];
    // joint_angles[2] = 0.0;
    // joint_angles[3] = 0.0;

        pose[0] +=joy_msg->axes[0]/40;
        pose[1] +=joy_msg->axes[1]/40;
        pose[2] +=joy_msg->axes[6]/100;
        pose[3] +=joy_msg->axes[7]/40;
        std::cout<<"pose==" << pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" "<<pose[3] <<std::endl;
    }


 void publishJointAngles() {
    // auto message = std::make_shared<std_msgs::msg::Float32MultiArray>();
    a1_msg::msg::JointAngles message;
    Vec34 vecOP = Pose_control(pose[0],pose[1],pose[2],pose[3]);
    Vec12 q = robotModel->getQ(vecOP,FrameType::BODY);

    for (int i(0); i<12 ;++i){
        message.joint_angles[i] = q[i];
    }
    joint_angles_publisher->publish(message);

    //debug
    // std::cout<<"p_b0"<<_initVecOX<<std::endl;
    // std::cout<<"p_si"<<_initVecXP<<std::endl;
    // std::cout<<"vectorOP=="<<vecOP<<std::endl;
    // std::cout<<"q is "<<q<<std::endl;

    }


Vec34 Pose_control(float row, float pitch, float yaw, float height){
    Vec34 vecOP;
    row    = invNormalize(row, _rowMin, _rowMax);
    pitch  = invNormalize(pitch, _pitchMin, _pitchMax);
    yaw    =-invNormalize(yaw, _yawMin, _yawMax);
    height = invNormalize(height, _heightMin, _heightMax);

    Vec3 vecXO = -_initVecOX;
    vecXO(2) += height;

    RotMat rotM = rpyToRotMat(row, pitch, yaw);

    HomoMat Tsb = homoMatrix(vecXO, rotM);
    HomoMat Tbs = homoMatrixInverse(Tsb);

    Vec4 tempVec4;

    //求出最终结果pbi 的四列， 然后传给calccmd运动学逆解
    for(int i(0); i<4; ++i){
        tempVec4 = Tbs * homoVec(_initVecXP.col(i));
        vecOP.col(i) = noHomoVec(tempVec4);
    }
    return vecOP;
  }


    // Vec12 calcCmd(Vec34 vecOP){
    //     Vec12 q = robotModel->getQ(vecOP, FrameType::BODY);
    //     return q;
    // }

    private:
    rclcpp::Publisher<a1_msg::msg::JointAngles>::SharedPtr joint_angles_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<float> pose;
    Vec3 _initVecOX;
    Vec34 _initVecXP;
    float _rowMax, _rowMin;
    float _pitchMax, _pitchMin;
    float _yawMax, _yawMin;
    float _heightMax, _heightMin;
    Vec3 q;
    Vec3 q_r;
    
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