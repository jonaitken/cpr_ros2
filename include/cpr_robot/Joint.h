#pragma once

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace cpr_robot
{
	//! \class Joint Joint.h <cpr_robot.h>
	//! \brief Represents a single joint of a robot. 
	//!
	//! This class contains information about the current state of the joint (position, speed) as well as the parameters for the joint (gear ratio, ticks per motor rotation). Conversion between encoder ticks and the actual position of the joint (measured in degrees) is done by this class.
    //! It is also used to get/set digital I/O states of the controller board associated with the joint and can be used for this alone also, i.e. with a pure DIO-module.
    class Joint 
    {
    public:
        //! Status bit indicating that the joint has been referenced.
        static constexpr uint8_t DATABIT_REFERENCED=0x80;
    private:
        //! Flag indicating whether the joint is reported as referenced by the firmware.
        bool m_bReferenced;
        //cpr_robot::Robot m_myRobot;
        //! A handle to the current ROS node.
        //ros::NodeHandle m_Node;
        rclcpp::Node::SharedPtr m_node;
        //! Pointer to an instance of the MotorModule class which represents the module and firmware driving the motor for the joint.
        MotorModule* m_pModule;
        //! The current position of the joint computed from the encoder position and joint parameters in degrees.
        double m_CurrentPosition;
        //! The current angular velocity of the joint computed from the change in encoder position and joint parameters in degrees per second.
        double m_CurrentVelocity;
        //! Reserved for future use with closed loop joints. Currently always zero.
        double m_CurrentEffort;
        //! The total gear ratio of the joint.
        double m_GearRatio;
        //! The desired angular velocity of the joint in radians per second.
        double m_DesiredVelocity;
        //! The maximum angular velocity of the joint in radians per second.
        double m_MaxVelocity;
        //! The error flags reported by the firmware of the module that is controlling the joint.
        uint8_t m_ErrorFlags;
        //! The number of encoder ticks representing exactly one rotation of the motor that is driving this joint.
        int32_t m_TicksPerMotorRotation;
        //! The name used for this joint when communicating over ROS topics and services.
        std::string m_JointName;
        //! The timestamp of the last status message received from the firmware in the module that is controlling the joint.
        uint8_t m_LastTimeStamp;
        //! The time at which the last status message has been received from the firmware in the module that is controlling the joint.
        std::chrono::high_resolution_clock::time_point m_LastUpdate;
        //! The ROS subscriber for the /JointJog topic which will receive jog messages.
        rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr m_JointJogSubscriber;
        double ReadPosition(uint8_t& timeStamp, std::chrono::high_resolution_clock::time_point& receptionTime,uint8_t& errorFlags, uint8_t& dataBits);
        void JointJogCallback(const control_msgs::msg::JointJog::SharedPtr msg);
    protected:
        virtual void OnInit();
        virtual void OnRead();
        virtual void OnWrite(double override);
        int32_t PositionToTicks(const double position) const;
        double TicksToPosition(const int32_t ticks) const;
    public:
        void Init();
        void Read();
        void Write(double override);
        void PublishState(sensor_msgs::msg::JointState& msg);
        const std::string& get_JointName() const;
        void set_JointName(const std::string& jointName);
        int32_t get_TicksPerMotorRotation() const;
        void set_TicksPerMotorRotation(const int32_t ticks);
        double get_GearRatio() const;
        void set_GearRatio(const double ratio);
        bool get_DigitalInput(const uint8_t channel) const;
        void set_DigitalOutput(const uint8_t channel, const bool state);
        bool get_DigitalOutput(const uint8_t channel) const;
        double get_CurrentPosition() const;
        double get_CurrentVelocity() const;
        double get_CurrentEffort() const;
        double get_DesiredVelocity() const;
        double get_MaxVeclocity() const;
        void set_MaxVelocity(const double velocity);
        double get_MinPosition() const;
        void set_MinPosition(const double position);
        double get_MaxPosition() const;
        void set_MaxPosition(const double position);
        uint8_t get_ErrorFlags() const;
        bool get_IsReferenced() const;
        int32_t get_MotorOffset() const;
        void set_MotorOffset(const int32_t ticks);
        Joint(Bus& canBus, const uint32_t moduleId, rclcpp::Node::SharedPtr node);
        virtual ~Joint();
        void EnableMotor();
        void DisableMotor();
        void StartReferencing();
        void SetZero();
        void Start();
        void Stop();
    };
}
