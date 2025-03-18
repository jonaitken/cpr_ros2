#include <cpr_robot.h>
#include "qt_includes.h"
#include "JointControl.h"
#include <sstream>
#include <iomanip>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <rviz_common/display_context.hpp>
#include "TaggedButton.h"
#include "RobotPanel.h"
using std::placeholders::_1;

namespace cpr_ros2
{
    //! \brief Constructs a JointControl widget.
    //! \param jointId The ID of the joint that will be controlled by the widget.
    //! \param parent The parent widget
    JointControl::JointControl(const uint32_t jointId,QWidget* parent, rclcpp::Node::SharedPtr node ) :
        m_GroupBox(this),
        m_JointId(jointId),
        m_PositionLabel(&m_GroupBox),
        m_VelocityLabel(&m_GroupBox),
        m_VelocitySlider(Qt::Horizontal,&m_GroupBox),
        m_StopButton(&m_GroupBox),
        m_bIsReferenced(false)
    {
		m_node=node;
        InitializeROS();
        InitializeState();
        InitializeUI();
    }
    
    //! \brief Initializes all UI elements.
    void JointControl::InitializeUI()
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::InitializeUI()...");
        m_GroupLayout.addRow(tr("Position:"),&m_PositionLabel);
        m_GroupLayout.addRow(tr("Velocity:"),&m_VelocityLabel);
        m_GroupLayout.addRow(&m_VelocitySlider);
        m_GroupLayout.addRow(&m_StopButton);
        m_StopButton.setText(tr("STOP"));
        m_VelocitySlider.setMinimum(-100);
        m_VelocitySlider.setMaximum(100);
        m_VelocitySlider.setTickInterval(100);
        m_VelocitySlider.setTickPosition(QSlider::TicksBelow);
        m_GroupBox.setLayout(&m_GroupLayout);
        m_GroupBox.setTitle((m_JointName+":").c_str());
        m_MainLayout.addWidget(&m_GroupBox);
        setLayout(&m_MainLayout);
        OnPositionChanged();
        OnVelocityChanged();
        Disable();
        connect(&m_VelocitySlider,&QAbstractSlider::valueChanged,this,&JointControl::OnVelocitySliderValueChanged);
        connect(&m_StopButton,&QAbstractButton::clicked,this,&JointControl::OnStopButtonClicked);
    }
    
    //! \brief Sets the referencing state.
    //! \param bReferenced Flag indicating whether the joint has been referenced. 
    void JointControl::set_IsReferenced(const bool bReferenced)
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::set_IsReferenced()...");
        if(m_bIsReferenced!=bReferenced)
        {
            m_bIsReferenced=bReferenced;
            OnPositionChanged();
        }
    }
        
    //! \brief Gets the referencing state.
    //! \return Flag indicating whether the joint has been referenced. 
    bool JointControl::get_Referenced() const
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::get_Referenced()...");
        return m_bIsReferenced;
    }

    //! \brief Enables all relevant UI elements.
    void JointControl::Enable()
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::Enable()...");
        m_VelocitySlider.setValue(0);
        m_VelocitySlider.setEnabled(true);
        m_StopButton.setEnabled(true);
    }

    //! \brief Disables all relevant UI elements.
    void JointControl::Disable()
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::Disable()...");
        m_VelocitySlider.setValue(0);
        m_VelocitySlider.setEnabled(false);
        m_StopButton.setEnabled(false);
    }

    //! \brief Callback slot that is called when the STOP button is clicked.
    void JointControl::OnStopButtonClicked(bool checked)
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::OnStopButtonClicked()...");
        m_VelocitySlider.setValue(0);
    }

    //! \brief Updates the UI elements associated with velocity.
    void JointControl::OnVelocityChanged()
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::OnVelocityChanged()...");
        std::stringstream sstr;
        sstr << ::std::fixed << std::setprecision(1) << (180.0*m_Velocity/M_PI) << " °/s";
        m_VelocityLabel.setText(sstr.str().c_str());
    }

    //! \brief Updates the UI elements associated with position.
    void JointControl::OnPositionChanged()
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::OnPositionChanged()...");
        std::stringstream sstr;
        sstr << ::std::fixed << std::setprecision(1) << (180.0*m_Position/M_PI) << " °";
        if(!m_bIsReferenced)
            sstr << " (unreferenced)";
        m_PositionLabel.setText(sstr.str().c_str());
    }

    //! \brief Initializes all relevant members.
    void JointControl::InitializeState()
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::InitializeState()...");
        //cpr_ros2::srv::GetJointInfo_Response jointInfo=GetJointInfo();
        m_JointName="joint"+std::to_string(m_JointId+1);
        m_JointType=0;
        m_Position=0.0;
        m_Velocity=0.0;
    }
    
    //! \brief Sets up communication with ROS.
    void JointControl::InitializeROS()
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::InitializeROS()...");
    	//cpr_ros2::RobotPanel* panel = dynamic_cast<cpr_ros2::RobotPanel*>(parent());
    	//rviz_common::DisplayContext* context = panel->getContext();

    	//node_ptr_=context->getRosNodeAbstraction().lock();
    	//node_ptr_=getDisplayContext()->getRosNodeAbstraction().lock();
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "node_ptr_...");
    	//node_ptr = node_ptr_->get_raw_node();
    	//rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::Raw Nodes...");
        //std::shared_ptr<rclcpp::Node> node1 = rclcpp::Node::make_shared("getjointcontrol_server");
        //m_GetJointInfoClient=m_Node.serviceClient<cpr_ros2::srv::GetJointInfo>("/GetJointInfo");
        m_GetJointInfoClient = m_node->create_client<cpr_ros2::srv::GetJointInfo>("/GetJointInfo");
        
        m_JointJogPublisher=m_node->create_publisher<control_msgs::msg::JointJog>("/JointJog", 50);
        
        //m_JointStateSubscriber=m_Node.subscribe("/joint_states",10,&JointControl::JointStateCallback, this);
        m_JointStateSubscriber = m_node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&JointControl::JointStateCallback, this, _1));
        
        
    }

    //! \brief Callback that handles messages received over the /JointState ROS topic.
    //! \param msg The received message.
    void JointControl::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::JointStateCallback()...");
        for(size_t i=0;i<msg->name.size();i++)
        {
            if(msg->name[i]==m_JointName)
            {
                double velocity=msg->velocity[i];
                if(velocity!=m_Velocity)
                {
                    m_Velocity=velocity;
                    OnVelocityChanged();
                }
                double position=msg->position[i];
                if(position!=m_Position)
                {
                    m_Position=position;
                    OnPositionChanged();
                }
            }
        }
    }

    //! \brief Callback slot that handles changes to the velocity control slider.
    void JointControl::OnVelocitySliderValueChanged(int value)
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JointControl::OnVelocitySliderValueChanged()...");
    	rclcpp::Clock clock;
    	
    	
        control_msgs::msg::JointJog msg;
        msg.header.stamp= clock.now();
        msg.joint_names.push_back(m_JointName);
        double velocity=0.01*(double)value;
        msg.velocities.push_back(velocity);
        m_JointJogPublisher->publish(msg);
    }

    //! \brief Queries information about the joint from the /GetJointInfo ROS service.
    cpr_ros2::srv::GetJointInfo_Response JointControl::GetJointInfo()
    {
        //cpr_ros2::srv::GetJointInfo srv;
    	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Called GetJointInfo...");
        auto request = std::make_shared<cpr_ros2::srv::GetJointInfo::Request>();
        request->sender="cpr_ros2::JointControl";
        request->jointid=m_JointId;
        cpr_ros2::srv::GetJointInfo::Response result = *m_GetJointInfoClient->async_send_request(request).get();
        //if (m_GetJointInfoClient->call(srv))
        //{
            //ROS_INFO("GetJointInfo: %s, type %ui.",srv.response.jointname.c_str(),srv.response.jointtype);
        //}
        //else
        //{
            //ROS_ERROR("Failed to call service GetJointInfo.");
            //srv.Response.jointname="unknown_joint";
            //srv.Response.jointtype=0;
        //}
        return result;
    }
}

#include "moc_JointControl.cpp"
