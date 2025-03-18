#pragma once

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>


namespace cpr_ros2
{
	//! \class RobotPanel RobotPanel.h <RobotPanel.h>
	//! \brief Plugin for RViz that allows to move a robot remotely over ROS.
    class RobotPanel: public rviz_common::Panel
    {
    	Q_OBJECT
    private:
        //! A handle to the current ROS node.
        //ros::NodeHandle m_Node;
        //! Client used to query information about the robot via the /GetRobotInfo ROS service.
        //ros::ServiceClient m_GetRobotInfoClient;
        rclcpp::Client<cpr_ros2::srv::GetRobotInfo>::SharedPtr m_GetRobotInfoClient;
        //! Client used to send commands to the robot via the /RobotCommand ROS service.
        //ros::ServiceClient m_RobotCommandClient;
        rclcpp::Client<cpr_ros2::srv::RobotCommand>::SharedPtr m_RobotCommandClient;
        //! Subscriber listening to the /RobotState ROS topic.
        //ros::Subscriber m_RobotStateSubscriber;
        rclcpp::Subscription<cpr_ros2::msg::RobotState>::SharedPtr m_RobotStateSubscriber;
        //! The model designation reported by the robot.
        std::string m_ModelName;
        //! The number of joints reported by the robot.
        uint32_t m_CountJoints;
        //! The digital inputs reported by the robot.
        std::vector<std::string> m_Inputs;
        //! The digital outputs reported by the robot.
        std::vector<std::string> m_Outputs;
        //! Layout aranging the control buttons horizontally.
        QHBoxLayout m_ControlButtonsLayout;
        //! Button allowing to connect/disconnect the robot remotely.
        QPushButton m_ConnectButton;
        //! Button allowing to enable/disable motor motion on the robot remotely.
        QPushButton m_EnableButton;
        //! Button allowing to start the referencing procedure on the robot remotely.
        QPushButton m_ReferenceButton;
        //! Button allowing to set the encoder positions on the robot remotely to zero.
        QPushButton m_ZeroButton;
        //! Label used to show the current override value of the robot.
        QLabel m_OverrideLabel;
        //! Slider used to change the desired override value of the robot.
        QSlider m_OverrideSlider;
        //! Layout arranging the controls of the individual joints.
        QGridLayout m_JointsLayout;
        //! A group box widget that will contain the input channels.
        QGroupBox m_InputGroupBox;
        //! Layout arranging the controls of the digital inputs.
        QGridLayout m_InputLayout;
         //! A group box widget that will contain the output channels.
        QGroupBox m_OutputGroupBox;
        //! Layout arranging the controls of the digital outputs.
        QGridLayout m_OutputLayout;
        //! The top-level layout of this widget.
        QFormLayout m_MainLayout;
        //! Label used to show the model deisgnation of the robot.
        QLabel m_ModelNameLabel;
         //! The status flags reported by the robot.
        uint32_t m_StatusFlags;
        //! The override value reported by the robot.
        double m_Override;
        //! Flag indicating whether the position of the override slider has been properly initialized.
        bool m_OverrideSliderValid;
        //! Array containing the control widgets for the individual joints.
        JointControl** m_pJointControls;
        //! Array containing the checboxes for the digital inputs.
        QCheckBox** m_pInputs;
        //! Array containing the buttons for the digital outputs.
        TaggedButton** m_pOutputs;
        //! Subscriber that will listen for joint states on the /InputChannels ROS topic.
        //ros::Subscriber m_InputChannelsSubscriber;
        rclcpp::Subscription<cpr_ros2::msg::ChannelStates>::SharedPtr m_InputChannelsSubscriber;
        //! Subscriber that will listen for joint states on the /OutputChannels ROS topic.
        //ros::Subscriber m_OutputChannelsSubscriber;
        rclcpp::Subscription<cpr_ros2::msg::ChannelStates>::SharedPtr m_OutputChannelsSubscriber;
        rclcpp::Node::SharedPtr node_ptr;
    
        void InitializeUI();
        void InitializeROS();
        void InitializeState();
        
        //void RobotStateCallback(const cpr_robot::RobotState::ConstPtr& msg);
        void RobotStateCallback(const cpr_ros2::msg::RobotState::SharedPtr msg);
                
        //void InputChannelsCallback(const cpr_robot::ChannelStates::ConstPtr& msg);
        void InputChannelsCallback(const cpr_ros2::msg::ChannelStates::SharedPtr msg);
                
        //void OutputChannelsCallback(const cpr_robot::ChannelStates::ConstPtr& msg);
        void OutputChannelsCallback(const cpr_ros2::msg::ChannelStates::SharedPtr msg);

        void OnStatusFlagsChanged();
        void OnOverrideChanged();
    protected:
        cpr_ros2::srv::GetRobotInfo_Response GetRobotInfo();
        void handle_GetRobotInfo_response(rclcpp::Client<cpr_ros2::srv::GetRobotInfo>::SharedFuture future);
        cpr_ros2::srv::RobotCommand_Response RobotCommand(const uint32_t commandId, const double payloadFloat, const int64_t payloadInt);
        std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
    protected slots:
        void OnOverrideSliderValueChanged(int value);
        void OnConnectButtonClicked(bool bChecked = true);
        void OnOutputButtonClicked(int tag, bool bChecked = true);
        void OnEnableButtonClicked(bool bChecked = true);
        void OnReferenceButtonClicked(bool bChecked = true);
        void OnZeroButtonClicked(bool bChecked = true);
    public:
	    RobotPanel( QWidget* parent = nullptr );
	    rviz_common::DisplayContext* getContext();
	    void onInitialize() override;
        virtual ~RobotPanel();
    };

}

//! \namespace cpr_rviz Provides classes defining a plugin for RViz.
