#pragma once
#include "rclcpp/rclcpp.hpp"


#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>


namespace cpr_ros2
{
	//! \class JointControl JointControl.h <JointControl.h>
	//! \brief Widget that provides motion control for a single robot joint. 
    class JointControl: public QWidget
    {
    	Q_OBJECT
    private:
        //! A handle to the current ROS node.
        //ros::NodeHandle m_Node;
        rclcpp::Node::SharedPtr m_node;
        //! Client used to query information about the joint via the /GetJointInfo ROS service.
        //ros::ServiceClient m_GetJointInfoClient;
        rclcpp::Client<cpr_ros2::srv::GetJointInfo>::SharedPtr m_GetJointInfoClient;
        //! Publisher used to publish jog commands over the /JointJog ROS topic.
        //ros::Publisher m_JointJogPublisher;
        rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr m_JointJogPublisher;
        //! Subscriber that will listen for joint states on the /joint_states ROS topic.
        //ros::Subscriber m_JointStateSubscriber;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_JointStateSubscriber;
        //! The ID of the joint.
        uint32_t m_JointId;
        //! The name of the joint as used in ROS topics and services.
        std::string m_JointName;
        //! The position of the joint in radians.
        double m_Position;
        //! The angular velocity of the joint in radians per second.
        double m_Velocity;
        //! Flag indicating whether the joint has been referenced.
        bool m_bIsReferenced;
        //! The joint type. Currently this is not used
        uint32_t m_JointType;
        //! The top-level layout of the widget.
        QHBoxLayout m_MainLayout;
        //! A group box widget that will contain the controls.
        QGroupBox m_GroupBox;
        //! The layout arranging the controls in the group box.
        QFormLayout m_GroupLayout;
        //! A label that will show the current position.
        QLabel m_PositionLabel;
        //! A label that will show the current velocity.
        QLabel m_VelocityLabel;
        //! A slider that allows to control the desired velocity.
        QSlider m_VelocitySlider;
        //! A button that will set the desired velocity to zero when pressed.
        QPushButton m_StopButton;

        void OnVelocityChanged();
        void OnPositionChanged();

        void InitializeUI();
        void InitializeROS();
        void InitializeState();
        void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        
    protected slots:
        void OnVelocitySliderValueChanged(int value);
        void OnStopButtonClicked(bool checked = false);
    protected:
        cpr_ros2::srv::GetJointInfo_Response GetJointInfo();
        std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
    public:
        void set_IsReferenced(const bool bReferenced);
        bool get_Referenced() const;
        void Enable();
        void Disable();
 	    JointControl(const uint32_t jointId, QWidget* parent = nullptr, rclcpp::Node::SharedPtr m_node  = nullptr );
    };
}
