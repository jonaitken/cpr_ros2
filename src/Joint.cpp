#include <cpr_robot.h>
#include <iostream>
#include <sstream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


using std::placeholders::_1;

namespace cpr_robot
{
    //! \brief Constructs an instance of the Joint class.
    //! \param canBus An instance of the Bus class which will be used for communication with the firmware in the module that is controlling the motor of the joint.
    //! \param moduleId The Id of the module that is controlling the motor of this joint.
    Joint::Joint(Bus& canBus, const uint32_t moduleId, rclcpp::Node::SharedPtr node) :
        m_CurrentEffort(0.0),
        m_CurrentPosition(0.0),
        m_CurrentVelocity(0.0),
        m_MaxVelocity(M_PI*45.0/180.0),
        m_DesiredVelocity(0.0),
        m_GearRatio(1.0),
        m_bReferenced(false),
        m_TicksPerMotorRotation(2000),
        m_ErrorFlags(0x00)
    {
		
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "In Joint Constructor...");
        m_pModule=new MotorModule(canBus,(unsigned int)moduleId);
        std::stringstream sstrm;
        sstrm << "joint" << (moduleId+1);
        m_JointName=sstrm.str();
        m_node=node;
        //m_myRobot=myRobot;
    }

    //! \brief Allows to determine if the joint has been referenced.
    //! \return If the joint is reported as referenced by the firmware returns true, elsewise false.
    bool Joint::get_IsReferenced() const
    {
        return m_bReferenced;
    }

    //! \brief The destructor of the Joint class.
    Joint::~Joint()
    {
        delete m_pModule;
    }

    //! \brief Gets the name of the joint used when communicating over ROS topics and services.
    //! \return The name of the joint as used in message over /JointJog and /joint_states ROS topics.
    const std::string& Joint::get_JointName() const
    {
        return m_JointName;
    }

    //! \brief Sets the name of the joint used when communicating over ROS topics and services.
    //! \param jointName The name of the joint as used in message over /JointJog and /joint_states ROS topics.
    void Joint::set_JointName(const std::string& jointName)
    {
        m_JointName=jointName;
    }

    //! \brief Gets the current position of the joint.
    //! \return The current position in degrees.
    double Joint::get_CurrentPosition() const
    {
        return m_CurrentPosition;
    }

    //! \brief Gets the current angular velocity of the joint.
    //! \return The current angular velocity in degrees per second.
    double Joint::get_CurrentVelocity() const
    {
        return m_CurrentVelocity;
    }

    //! \brief Gets the effort that is put into moving the joing. Currently not implemented.
    //! \return Always returns zero.
    double Joint::get_CurrentEffort() const
    {
        return m_CurrentEffort;
    }
     
    //! \brief Initialization. Should only be called after the parameters (gear ratio, ticks per motor rotation) have been set.
    //!
    //! This virtual function is intended to be overridable by derived classes, but should then be called from the override.
    void Joint::OnInit()
    {
        uint8_t dataBits;
        m_CurrentPosition=ReadPosition(m_LastTimeStamp, m_LastUpdate,m_ErrorFlags, dataBits);
        m_bReferenced=(dataBits&DATABIT_REFERENCED)!=0x00;
        m_CurrentEffort=0.0;
        m_CurrentVelocity=0.0;
        m_DesiredVelocity=0.0;
        //m_JointJogSubscriber=m_Node.subscribe("/JointJog",10,&Joint::JointJogCallback, this);
        //node_ptr_=shared_from_this();
        //node=node_ptr_->get_raw_node();
        //m_JointJogSubscriber = this->create_subscription<control_msgs::msg::JointJog>("/JointJog", 10, std::bind(&Joint::JointJogCallback, this, _1));
        m_JointJogSubscriber = m_node->create_subscription<control_msgs::msg::JointJog>("/JointJog", 10, std::bind(&Joint::JointJogCallback, this, _1));
        //m_JointJogSubscriber = this->create_subscription<control_msgs::msg::JointJog>("/JointJog", 10, std::bind(&Joint::JointJogCallback, this, _1));
    }
    
    //! \brief Callback for received messages on the /JointJog ROS topic.
    //! \param msg The received message.
    void Joint::JointJogCallback(const control_msgs::msg::JointJog::SharedPtr msg)
    {
        for(size_t i=0;i<msg->joint_names.size();i++)
        {
            if(msg->joint_names[i]==m_JointName)
            {
                if(msg->velocities.size()>i)
                {
                    m_DesiredVelocity=msg->velocities[i]*m_MaxVelocity;
                }
            }
        }
    }

    //! \brief Gets the desired angular velocity of the joint.
    //! \return The desired angular velocity in degrees per second.
    double Joint::get_DesiredVelocity() const
    {
        return m_DesiredVelocity;
    }
    
    //! \brief Gets the maximum allowed angular velocity of the joint.
    //! \return The currently allowed angular velocity in degrees per second.
    double Joint::get_MaxVeclocity() const
    {
        return 180.0*m_MaxVelocity/M_PI;
    }
    
    //! \brief Sets the maximum allowed angular velocity of the joint.
    //! \param velocity The new allowed angular velocity in degrees per second.
    void Joint::set_MaxVelocity(const double velocity)
    {
        m_MaxVelocity=M_PI*velocity/180.0;
    }
    
    //! \brief Gets the lower position bound of the joint.
    //! \return The currrent lower bound for the position of the joint in degrees.
    double Joint::get_MinPosition() const
    {
        return 180.0*TicksToPosition(m_pModule->get_MinPosition())/M_PI;
    }
    
    //! \brief Sets the lower position bound of the joint.
    //! \param position The new lower bound for the position of the joint in degrees.
    void Joint::set_MinPosition(const double position)
    {
        m_pModule->set_MinPosition(PositionToTicks(M_PI*position/180.0));
    }
    
    //! \brief Gets the upper position bound of the joint.
    //! \return The currrent upper bound for the position of the joint in degrees.
    double Joint::get_MaxPosition() const
    {
        return 180.0*TicksToPosition(m_pModule->get_MaxPosition())/M_PI;
    }
    
    //! \brief Sets the upper position bound of the joint.
    //! \param position The new upper bound for the position of the joint in degrees.
    void Joint::set_MaxPosition(const double position)
    {
        m_pModule->set_MaxPosition(M_PI*PositionToTicks(position)/180.0);
    }
    
    //! \brief Gets the zero position of the motor.
    //! \return Current offset of the the zero position in encoder ticks.
    int32_t Joint::get_MotorOffset() const
    {
        return m_pModule->get_Offset();
    }
         
    //! \brief Sets the zero position of the motor.
    //! \param ticks New offset of the the zero position in encoder ticks.
    void Joint::set_MotorOffset(const int32_t ticks)
    {
        m_pModule->set_Offset(ticks);
    }

    //! \brief Reads the current state of the joint from the firmware.
    //!
    //! This virtual function is intended to be overridable by derived classes, but should then be called from the override.
    void Joint::OnRead()
    {
        uint8_t timeStamp;
        uint8_t errorFlags;
        std::chrono::high_resolution_clock::time_point newUpdate;
        uint8_t dataBits;
        double position=ReadPosition(timeStamp, newUpdate, errorFlags, dataBits);
        if(timeStamp!=m_LastTimeStamp)
        {
            m_bReferenced=(dataBits&DATABIT_REFERENCED)!=0x00;
            double seconds=0.001*(double)std::chrono::duration_cast<std::chrono::milliseconds>(newUpdate - m_LastUpdate).count();
            m_LastUpdate=newUpdate;
            m_CurrentVelocity=(position-m_CurrentPosition)/seconds;
            m_CurrentPosition=position;
            m_CurrentEffort=0.0;
            m_ErrorFlags=errorFlags;
            m_LastTimeStamp=timeStamp;
        }
    }

    //! \brief Gets the error flags that are currently reported by the firmware in the module that is controlling the motor of the joint.
    //! \return Bitfield with the error flags.
    uint8_t Joint::get_ErrorFlags() const
    {
        return m_ErrorFlags;
    }

    //! \brief Converts a joint position to a motor position.
    //! \param position Joint position in radians.
    //! \return Corresponding motor position in encoder ticks.
    int32_t Joint::PositionToTicks(const double position) const
    {
        double motorRotations=position*m_GearRatio/(2.0*M_PI);
        int32_t ticks=(int32_t)(motorRotations*((double)m_TicksPerMotorRotation));
        return ticks;
    }
    
    //! \brief Converts a motor position to a joint position.
    //! \param ticks Motor position in encoder ticks.
    //! \return Corresponding joint position in radians.
    double Joint::TicksToPosition(const int32_t ticks) const
    {
        double motorRotations=((double)ticks)/((double)m_TicksPerMotorRotation);
        double position=2.0*M_PI*motorRotations/m_GearRatio;
        return position;
    }

    //! \brief Sends the current motion commands to the firmware in the module that is controlling the motor of the joint.
    //! This virtual function is intended to be overridable by derived classes, but should then be called from the override.
    //! \param override The override factor applied to the currently desired velocity. Should be a value between 0 and 1.
    void Joint::OnWrite(double override)
    {
        if(m_ErrorFlags==0x00)
        {
            double seconds=m_pModule->get_UpdateInterval();
            double desiredPositionIncrement=m_DesiredVelocity*seconds*override;
            int32_t desiredTicks=PositionToTicks(desiredPositionIncrement);
            m_pModule->set_Increment(desiredTicks);
        }
    }

    //! \brief Initializes the joint. 
    //! 
    //! Should be called after all joint parameters (gear ratio, ticks per motor rotation) have been set. Will call the virtual OnInit method.
    void Joint::Init()
    {
        OnInit();
    }
    
    //! \brief Reads the current state of the joint from the firmware in the module that is controlling the motor of the joint.
    //!
    //! Will call the virtual OnRead method.
    void Joint::Read()
    {
        OnRead();
    }
    
    //! \brief Sends the current motion commands to the firmware in the module that is controlling the motor of the joint.
    //!
    //! Will call the virtual OnWrite method.
    //! \param override The override factor applied to the currently desired velocity. Should be a value between 0 and 1.
    void Joint::Write(double override)
    {
        OnWrite(override);
    }

    //! \brief Used when publishing joint states.
    //!
    //! Will add the current state of the joint to a JointState message that can then be published by the caller, typically on
    //! the /joint_states ROS topic.
    //! \param msg The message to which the state should be added.
    void Joint::PublishState(sensor_msgs::msg::JointState& msg)
    {
    
        msg.position.push_back(m_CurrentPosition);
        msg.velocity.push_back(m_CurrentVelocity);
        msg.effort.push_back(0.0);
        msg.name.push_back(m_JointName);
    }

    //! \brief Returns the position of the joint according to the last update received from the firmware that is controlling the motor of the joint.
    //! \param timeStamp The timeStamp of the update as received from the firmware.
    //! \param receptionTime Time time at which the update was received from the firmware.
    //! \param errorFlags The error flags reported by the firmware.
    //! \param dataBits The status bits received from the firmware.
    //! \return The position of the joint in radians.
    double Joint::ReadPosition(uint8_t& timeStamp, std::chrono::high_resolution_clock::time_point& receptionTime,uint8_t& errorFlags, uint8_t& dataBits)
    {
        int32_t ticks=m_pModule->get_CurrentPosition(timeStamp, receptionTime, errorFlags, dataBits);
        return TicksToPosition(ticks);
    }
    
    //! \brief Gets the current number of ticks per motor rotation used during conversion from encoder position to joint position.
    //! \return Current number of encoder ticks representing exactly one motor rotation.
    int32_t Joint::get_TicksPerMotorRotation() const
    {
        return m_TicksPerMotorRotation;
    }

    //! \brief Sets the current number of ticks per motor rotation used during conversion from encoder position to joint position.
    //! \param ticks New number of encoder ticks representing exactly one motor rotation.
    void Joint::set_TicksPerMotorRotation(const int32_t ticks)
    {
        m_TicksPerMotorRotation=ticks;
    }

    //! \brief Gets the current gear ratio that is used during conversion from encoder position to joint position.
    //! \return Current gear ratio.
    double Joint::get_GearRatio() const
    {
        return m_GearRatio;
    }
        
    //! \brief Sets the current gear ratio that is used during conversion from encoder position to joint position.
    //! \param ratio New gear ratio.
    void Joint::set_GearRatio(const double ratio)
    {
        m_GearRatio=ratio;
    }

    //! \brief Will send a command to enable motor motion to the firmware of the module that is controlling the motor of the joint.
    void Joint::EnableMotor()
    {
        m_pModule->Enable();
    }

    //! \brief Will send a command to disable motor motion to the firmware of the module that is controlling the motor of the joint.
    void Joint::DisableMotor()
    {
        m_pModule->Disable();
    }

    //! \brief Will start the communication with the firmware of the module that is controlling the motor of the joint.
    void Joint::Start()
    {
        m_pModule->Start();
    }
    
    //! \brief Will stop the communication with the firmware of the module that is controlling the motor of the joint.
    void Joint::Stop()
    {
        m_pModule->Stop();
    }

    //! \brief Will send a command to begin the referencing procedure for the joint to the firmware of the module that is controlling the motor of the joint.
    void Joint::StartReferencing()
    {
        m_pModule->StartReferencing();
    }

    //! \brief Will send a command to reset the encoder position for the joint to the firmware of the module that is controlling the motor of the joint.
    void Joint::SetZero()
    {
        m_pModule->SetZero();
    }

    //! \brief Retrieves the state of a digital input channel.
    //! \param channel The index of the cannel. A maximum of 8 channels is supported.
    //! \return The current state of the channel.
    bool Joint::get_DigitalInput(const uint8_t channel) const
    {
        return (m_pModule->get_DigitalInputs()&(1<<channel))!=0;
    }

    //! \brief Sets the state of a digital output channel.
    //! \param channel The index of the cannel. A maximum of 8 channels is supported.
    //! \param state The desired state of the channel.
    void Joint::set_DigitalOutput(const uint8_t channel, const bool state)
    {
        const uint8_t mask=1<<channel;
        uint8_t bits=m_pModule->get_DigitalOutputs();
        if(state)
            bits|=mask;
        else
            bits&=~mask;
        m_pModule->set_DigitalOutputs(bits);
    }
    
    //! \brief Retrieves the state of a digital output channel.
    //! \param channel The index of the cannel. A maximum of 8 channels is supported.
    //! \return The current state of the channel.
    bool Joint::get_DigitalOutput(const uint8_t channel) const
    {
        return (m_pModule->get_DigitalOutputs()&(1<<channel))!=0;
    }
 
}
