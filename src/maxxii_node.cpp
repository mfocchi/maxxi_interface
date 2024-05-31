#include <stdio.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "maxxii_interface/Constants.h"
#include "maxxii_interface/ErrorCodes.h"
#include "maxxii_interface/driver_interface.h"
#include "maxxii_interface/encoder.h"
#include "maxxii_interface/motor.h"
#include "maxxii_interface/differential_drive_model.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#define MILLI 1000.0
#define VELOCITY_CTRL_MODE 0
#define TORQUE_CTRL_MODE 1

class TrackedRobotNode : public rclcpp::Node 
{
private:
    rclcpp::TimerBase::SharedPtr timer_fbk;
    rclcpp::TimerBase::SharedPtr timer_sys;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sys_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr enc_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr io_pub;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub;

	std::shared_ptr<DifferentialDriveModel> Model;
    std::shared_ptr<DriverInterface> Device;
    std::shared_ptr<Encoder> EncoderLeft;
    std::shared_ptr<Encoder> EncoderRight;
    std::shared_ptr<Motor> MotorLeft;
    std::shared_ptr<Motor> MotorRight;
    bool enable_encoder_topic;
    bool enable_system_topic;
    bool enable_io_topic;
    int control_mode;

    double getTimeSec()
    {
        return this->get_clock()->now().seconds();
    }

public:
    TrackedRobotNode() : rclcpp::Node("maxxii")
    {
        double ppr, max_rpm, max_tau, max_amp;
        std::string port;
        int rate_feedback, rate_system;
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("motor_max_rpm", 1500.0);
        this->declare_parameter("motor_max_torque_Nm", 1.0);
        this->declare_parameter("motor_max_current_amp", 60.0);
        this->declare_parameter("enable_system_topic", true);
        this->declare_parameter("enable_encoder_topic", true);
        this->declare_parameter("enable_io_topic", false);
        this->declare_parameter("pub_rate_system_Hz", 1);
        this->declare_parameter("pub_rate_feedback_Hz", 200);
        this->declare_parameter("pulse_per_revolution", 1024.0);
		this->declare_parameter("control_mode", 0);

        this->get_parameter("port", port);
        this->get_parameter("motor_max_rpm", max_rpm);
        this->get_parameter("motor_max_current_amp", max_amp);
        this->get_parameter("motor_max_torque_Nm", max_tau);
        this->get_parameter("enable_system_topic", enable_system_topic);
        this->get_parameter("enable_encoder_topic", enable_encoder_topic);
        this->get_parameter("enable_io_topic", enable_io_topic);
        this->get_parameter("pub_rate_system_Hz", rate_system);
        this->get_parameter("pub_rate_feedback_Hz", rate_feedback);
        this->get_parameter("pulse_per_revolution", ppr);
		this->get_parameter("control_mode", control_mode);

        try{
            Device.reset(new RoboteqDriverInterface(max_rpm, max_amp, max_tau, port));
            EncoderLeft.reset(new Encoder(1, ppr, Device));
            EncoderRight.reset(new Encoder(2, ppr, Device));
            MotorLeft.reset(new Motor(1, Device));
            MotorRight.reset(new Motor(2, Device));
        }
        catch(int code){
            printErrorCode(code);
        }

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
        
        sys_pub = this->create_publisher<std_msgs::msg::String>("doretta/sys", 1);
        enc_pub = this->create_publisher<sensor_msgs::msg::JointState>("doretta/wheels", 1);
        io_pub = this->create_publisher<std_msgs::msg::String>("doretta/io", 1);

        if(control_mode == VELOCITY_CTRL_MODE)
        {
            cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                "/command", qos, 
                std::bind(&TrackedRobotNode::velocityCmdCallback, this, std::placeholders::_1)
                );
        }
        if(control_mode == TORQUE_CTRL_MODE)
        {
            cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                "/command", qos, 
                std::bind(&TrackedRobotNode::toruqeCmdCallback, this, std::placeholders::_1)
                );
        }
        
        timer_fbk = this->create_wall_timer(
            std::chrono::milliseconds(int(MILLI / rate_feedback)), 
            std::bind(&TrackedRobotNode::feedbackCallback, this)
            );
        timer_sys = this->create_wall_timer(
            std::chrono::milliseconds(int(MILLI / rate_system)), 
            std::bind(&TrackedRobotNode::systemCallback, this)
            );
        Device->printFirmware();
    }
    ~TrackedRobotNode(){}

    void velocityCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        try{
            if(this->control_mode == VELOCITY_CTRL_MODE){
                setMotorSpeeds(msg);
            } else{
                throw DIFFERENT_MOTOR_MODES;
            }
        }
        catch(int code){
            printErrorCode(code);
        }
    }

    void toruqeCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        try{
            if(this->control_mode == TORQUE_CTRL_MODE){
                setMotorTorques(msg);
            } else{
                throw DIFFERENT_MOTOR_MODES;
            }
        }
        catch(int code){
            printErrorCode(code);
        }
    }

    void setMotorSpeeds(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        if (joint_state_msg->velocity.size() != 2)
            throw WRONG_SIZE_JOINTSTATE_CMD;
        double motor_left_speed = joint_state_msg->velocity[0];
        double motor_right_speed = joint_state_msg->velocity[1];
        MotorLeft->moveRADPS(motor_left_speed);
        MotorRight->moveRADPS(motor_right_speed);
    }
    void setMotorTorques(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        if (joint_state_msg->effort.size() != 2)
            throw WRONG_SIZE_JOINTSTATE_CMD;
        double motor_left_tau = joint_state_msg->effort[0];
        double motor_right_tau = joint_state_msg->effort[1];
        MotorLeft->applyTorque(motor_left_tau);
        MotorRight->applyTorque(motor_right_tau);
    }

    void systemCallback() 
    {
        try{
            if(enable_system_topic)
            {
                sendSystemStateMessage();
            }
            if(enable_io_topic)
            {
                sendIOMessage();
            }
        }
        catch(int code){
            printErrorCode(code);
        }
    }

    void feedbackCallback() 
    {
        try{
            if(enable_encoder_topic)
            {
                sendEncoderMessage();
            }
        }
        catch(int code){
            printErrorCode(code);
        }
    }

    void sendSystemStateMessage()
    {
        double temp_left = MotorLeft->getTemperature();
        double temp_right = MotorRight->getTemperature();
        double volt_left = MotorLeft->getVoltage();
        double volt_right = MotorRight->getVoltage();
        double ampere_left = MotorLeft->getCurrent();
        double ampere_right = MotorRight->getCurrent();

        std::stringstream ss;
        ss  <<"t="<< getTimeSec()
            <<",Temp=["<<temp_left<<","<<temp_right
            <<"],volt=["<<volt_left<<","<<volt_right
            <<"],amp=["<<ampere_left<<","<<ampere_right<<"]";

        std_msgs::msg::String msg_sys;
        msg_sys.data = ss.str();
        sys_pub->publish(msg_sys);
    }

    void sendEncoderMessage()
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->get_clock()->now();
        msg.name = {"left", "right"};
        // for some reason left and right are swapped (right is left)
        msg.position = std::vector<double>{
            EncoderRight->getRadiants(),
            EncoderLeft->getRadiants()};
            
        msg.velocity = std::vector<double>{
            EncoderRight->getSpeedRads(),
            EncoderLeft->getSpeedRads()};
      
        msg.effort = std::vector<double>{
            MotorRight->getCurrent(),
            MotorLeft->getCurrent()};
          

        enc_pub->publish(msg);
    }

    void sendIOMessage()
    {
        std::string aio = "";
        Device->getInOutA(&aio);
        std::string dio = "";
         Device->getInOutD(&dio);

        std::stringstream ss;
        ss  <<"t="<< getTimeSec() 
            <<",aio=["<<aio
            <<"],dio=["<<dio<<"]";

        std_msgs::msg::String msg_io;
        msg_io.data = ss.str();
        io_pub->publish(msg_io);
    }
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackedRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
