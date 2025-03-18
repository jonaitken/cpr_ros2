#include "cpr_robot.h"
#include "qt_includes.h"
#include "TaggedButton.h"
#include "JointControl.h"
#include <rviz_common/panel.hpp>
#include "RobotPanel.h"
#include <sstream>
#include <memory>
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <rviz_common/display_context.hpp>
using std::placeholders::_1;
using namespace std::chrono_literals;
namespace cpr_ros2
{
    //! \brief Constructor of the RobotPanel class.
    //! \param parent The parent widget.
    RobotPanel::RobotPanel( QWidget* parent ) :
        rviz_common::Panel( parent ),
        m_ConnectButton(this),
        m_EnableButton(this),
        m_InputGroupBox(this),
        m_OutputGroupBox(this),
        m_ReferenceButton(this),
        m_ZeroButton(this),
        m_OverrideSlider(Qt::Horizontal,this),
        m_OverrideLabel(this),
        m_OverrideSliderValid(false)
    {
    	
        
    }

    //! \brief Destructor of the RobotPanel class.
    RobotPanel::~RobotPanel()
    {
    	int y=0;
        m_ControlButtonsLayout.setParent(nullptr);
        m_JointsLayout.setParent(nullptr);
        for(size_t i=0;i<m_Inputs.size();i++)
        {
            delete m_pInputs[i];
        }
        for(size_t i=0;i<m_Outputs.size();i++)
        {
            delete m_pOutputs[i];
        }
        delete[] m_pOutputs;
        for(uint32_t i=0;i<m_CountJoints;i++)
        {
            delete m_pJointControls[i];
        }
        delete[] m_pJointControls;
    }
    
    rviz_common::DisplayContext* RobotPanel::getContext() {
        return getDisplayContext();  // This works since it's in a derived class
    }
    
     void RobotPanel::onInitialize() {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "You rang...");
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Entering InitializeROS...");
        InitializeROS();
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Exiting InitializeROS...");
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Entering InitializeState...");
        InitializeState();
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Exiting InitializeState...");
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Entering InitializeUI...");
        InitializeUI();
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Exiting InitializeUI...");
     }

    //! \brief Initializes all relevant UI elements.
    void RobotPanel::InitializeUI()
    {
        m_ModelNameLabel.setText(m_ModelName.c_str());
        m_MainLayout.addRow(tr("Model:"), &m_ModelNameLabel);
        m_ControlButtonsLayout.addWidget(&m_ConnectButton);
        m_ControlButtonsLayout.addWidget(&m_EnableButton);
        m_ControlButtonsLayout.addWidget(&m_ReferenceButton);
        m_ControlButtonsLayout.addWidget(&m_ZeroButton);
        m_ReferenceButton.setText("Reference");
        m_ZeroButton.setText("Set Zero");
        double k=0.5*(double)m_CountJoints;
        int rows=(int)round(k+0.5);
        m_MainLayout.addRow(&m_ControlButtonsLayout);
        m_MainLayout.addRow(tr("Override:"), &m_OverrideLabel);
        m_OverrideSlider.setMinimum(0);
        m_OverrideSlider.setMaximum(100);
        m_OverrideSlider.setTickInterval(10);
        m_OverrideSlider.setTickPosition(QSlider::TicksBelow);
        m_MainLayout.addRow(&m_OverrideSlider);
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "m_Inputs.size() %d...",m_Inputs.size());
        
        m_pInputs=new QCheckBox*[m_Inputs.size()];
        for(size_t i=0;i<m_Inputs.size();i++)
        {
            m_pInputs[i]=new QCheckBox(&m_InputGroupBox);
            m_pInputs[i]->setText(m_Inputs[i].c_str());
            m_InputLayout.addWidget(m_pInputs[i],i/4,i%4);
        }
        m_InputGroupBox.setLayout(&m_InputLayout);
        m_InputGroupBox.setTitle("Digital inputs:");
        m_MainLayout.addRow(&m_InputGroupBox);
        m_pOutputs=new TaggedButton*[m_Outputs.size()];
        for(size_t i=0;i<m_Outputs.size();i++)
        {
            m_pOutputs[i]=new TaggedButton((int)i,&m_OutputGroupBox);
            m_pOutputs[i]->setText(m_Outputs[i].c_str());
            m_OutputLayout.addWidget(m_pOutputs[i],i/4,i%4);
            m_pOutputs[i]->setCheckable(true);
            connect(m_pOutputs[i],&TaggedButton::buttonClicked,this,&RobotPanel::OnOutputButtonClicked);
        }
        m_OutputGroupBox.setLayout(&m_OutputLayout);
        m_OutputGroupBox.setTitle("Digital outputs:");
        m_MainLayout.addRow(&m_OutputGroupBox);
        m_pJointControls=new JointControl*[m_CountJoints];
        for(uint32_t i=0;i<m_CountJoints;i++)
        {
            m_pJointControls[i]=new JointControl(i,this,node_ptr);
            m_JointsLayout.addWidget(m_pJointControls[i],i/2,i%2);
        }
        m_MainLayout.addRow(&m_JointsLayout);
        setLayout(&m_MainLayout);
        OnStatusFlagsChanged();
        connect(&m_ConnectButton, &QAbstractButton::clicked, this, &RobotPanel::OnConnectButtonClicked);
        connect(&m_EnableButton, &QAbstractButton::clicked, this, &RobotPanel::OnEnableButtonClicked);
        connect(&m_ReferenceButton, &QAbstractButton::clicked, this, &RobotPanel::OnReferenceButtonClicked);
        connect(&m_ZeroButton, &QAbstractButton::clicked, this, &RobotPanel::OnZeroButtonClicked);
        connect(&m_OverrideSlider,&QAbstractSlider::valueChanged,this,&RobotPanel::OnOverrideSliderValueChanged);
    }

    //! \brief Callback slot handling clicks to the digital outputs buttons.
    //! \param tag Channel number associated with the clicked button.
    void RobotPanel::OnOutputButtonClicked(int tag, bool bChecked)
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OnOutputButtonClicked...");
        if(!m_pOutputs[tag]->isChecked())
            RobotCommand(cpr_robot::Robot::COMMAND_DOUT_DISABLE, 0.0,tag);
        else
            RobotCommand(cpr_robot::Robot::COMMAND_DOUT_ENABLE, 0.0,tag);
    }
  

    //! \brief Callback slot handling changes to the override slider value.
    void RobotPanel::OnOverrideSliderValueChanged(int value)
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OnOverrideSliderValueChanged...");
        RobotCommand(cpr_robot::Robot::COMMAND_OVERRIDE, 0.01*(double)m_OverrideSlider.value(),0);
    }

    //! \brief Callback slot handling clicks to the "Connect/Disconnect" button.
    void RobotPanel::OnConnectButtonClicked(bool bChecked)
    {
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OnConnectButtonClicked...");
        if(m_StatusFlags&cpr_robot::Robot::STATUSFLAG_DISCONNECTED)
            RobotCommand(cpr_robot::Robot::COMMAND_CONNECT, 0.0, 0);
        else
            RobotCommand(cpr_robot::Robot::COMMAND_DISCONNECT, 0.0, 0);
    }

    //! \brief Callback slot handling clicks to the "Reference" button.
    void RobotPanel::OnReferenceButtonClicked(bool bChecked)
    {
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OnReferenceButtonClicked...");
        RobotCommand(cpr_robot::Robot::COMMAND_STARTREFERENCING, 0.0, 0);
    }

    //! \brief Callback slot handling clicks to the "Set Zero" button.
    void RobotPanel::OnZeroButtonClicked(bool bChecked)
    {
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OnZeroButtonClicked...");
        RobotCommand(cpr_robot::Robot::COMMAND_SETZERO, 0.0, 0);
    }

    //! \brief Callback slot handling clicks to the "Enable/Disable" button.
    void RobotPanel::OnEnableButtonClicked(bool bChecked)
    {
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OnEnableButtonClicked...");
        if(m_StatusFlags&cpr_robot::MotorModule::STATUSFLAG_MASK_ANY_ERROR)
            RobotCommand(cpr_robot::Robot::COMMAND_ENABLE, 0.0, 0);
        else
            RobotCommand(cpr_robot::Robot::COMMAND_DISABLE, 0.0, 0);
    }

    //! \brief Initializes all relevant members.
    void RobotPanel::InitializeState()
    {
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enter InitializeState...");
        cpr_ros2::srv::GetRobotInfo_Response rbtInfo;//=GetRobotInfo();
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GetRobotInfo...");
        rbtInfo.countjoints=6;
        rbtInfo.model="Unknown Model";
        m_CountJoints=rbtInfo.countjoints;
        //inputchannels=['Digital in 1', 'Digital in 2', 'Digital in 3']
        std::vector<std::string> inputchannels = {"Digital in 1", "Digital in 2", "Digital in 3"};
        
        //outputchannels=['Digital out 1', 'Digital out 2', 'Digital out 3', 'Digital out 4', 'Gripper open', 'Gripper enabled']
        std::vector<std::string> outputchannels = {"Digital out 1", "Digital out 2", "Digital out 3", "Digital out 4", "Gripper open", "Gripper enabled"};
        
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "InitializeState inputchannels.size() %d...",inputchannels.size());
        
        m_Inputs.clear();
        /*for(size_t i=0;i<rbtInfo.inputchannels.size();i++)
            m_Inputs.push_back(rbtInfo.inputchannels[i]);
        m_Outputs.clear();
        for(size_t i=0;i<rbtInfo.outputchannels.size();i++)
            m_Outputs.push_back(rbtInfo.outputchannels[i]);*/
            
        for(size_t i=0;i<inputchannels.size();i++)
            m_Inputs.push_back(inputchannels[i]);
        m_Outputs.clear();
        for(size_t i=0;i<outputchannels.size();i++)
            m_Outputs.push_back(outputchannels[i]);
                
        m_ModelName=rbtInfo.model;
        m_StatusFlags=cpr_robot::Robot::STATUSFLAG_DISCONNECTED;
        
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "InitializeState m_Inputs.size() %d...",m_Inputs.size());
    }
    
    //! \brief Sets up communication with ROS.
    void RobotPanel::InitializeROS()
    {
    	
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enter ROS...");
    	//std::shared_ptr<rclcpp::Node> node1 = rclcpp::Node::make_shared("robot_panel");
    	node_ptr_=getDisplayContext()->getRosNodeAbstraction().lock();
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "node_ptr_...");
    	node_ptr = node_ptr_->get_raw_node();
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "get_raw_node()...");
        //m_GetRobotInfoClient=m_Node.serviceClient<cpr_robot::GetRobotInfo>("/GetRobotInfo");
        m_GetRobotInfoClient = node_ptr->create_client<cpr_ros2::srv::GetRobotInfo>("/GetRobotInfo");
         //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GetRobotInfo()...");
        //m_RobotStateSubscriber=m_Node.subscribe("/robot_state",10,&RobotPanel::RobotStateCallback, this);
        m_RobotStateSubscriber = node_ptr->create_subscription<cpr_ros2::msg::RobotState>("/robot_state", 10, std::bind(&RobotPanel::RobotStateCallback, this, _1));
        
         //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robot_state()...");
        //m_RobotCommandClient= m_Node.serviceClient<cpr_robot::RobotCommand>("/RobotCommand");
        m_RobotCommandClient = node_ptr->create_client<cpr_ros2::srv::RobotCommand>("/RobotCommand");
        
         //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotCommand()...");
        
        //m_InputChannelsSubscriber=m_Node.subscribe("/InputChannels",10,&RobotPanel::InputChannelsCallback, this);
        m_InputChannelsSubscriber = node_ptr->create_subscription<cpr_ros2::msg::ChannelStates>("/InputChannels", 10, std::bind(&RobotPanel::InputChannelsCallback, this, _1));
        
         //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "InputChannels()...");
        //m_OutputChannelsSubscriber=m_Node.subscribe("/OutputChannels",10,&RobotPanel::OutputChannelsCallback, this);
        m_OutputChannelsSubscriber = node_ptr->create_subscription<cpr_ros2::msg::ChannelStates>("/OutputChannels", 10, std::bind(&RobotPanel::OutputChannelsCallback, this, _1));
        
         //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OutputChannels()...");
    }

    //! \brief Callback that handles messages received over the /RobotState ROS topic.
    //! \param msg The received message.
    //void RobotPanel::RobotStateCallback(const cpr_robot::RobotState::ConstPtr& msg)
    void RobotPanel::RobotStateCallback(const cpr_ros2::msg::RobotState::SharedPtr msg)
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotStateCallback... %d", msg->statusflags);
        if(m_StatusFlags!=msg->statusflags)
        {
            m_StatusFlags=msg->statusflags;
            OnStatusFlagsChanged();
        }
        if(m_Override!=msg->override)
        {
            m_Override=msg->override;
            OnOverrideChanged();
        }
        if(!m_OverrideSliderValid)
        {
            if((m_StatusFlags&0xffff)==0x0000)
            {
                m_OverrideSlider.setEnabled(true);
                m_OverrideSlider.setValue((int)(100.0*m_Override));
                m_OverrideSliderValid=true;
            }
        }
        for(uint32_t i=0;i<m_CountJoints;i++)
        {
            if(msg->statusflags&(((uint32_t)1)<<(16+i)))
                m_pJointControls[i]->set_IsReferenced(true);
            else
                m_pJointControls[i]->set_IsReferenced(false);
        }
    }

    //! \brief Callback that handles messages received over the /InputChannels ROS topic.
    //! \param msg The received message.
    //void RobotPanel::InputChannelsCallback(const cpr_robot::ChannelStates::ConstPtr& msg)
    void RobotPanel::InputChannelsCallback(const cpr_ros2::msg::ChannelStates::SharedPtr msg)
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "InputChannelsCallback...");
        for(size_t i=0;i<msg->state.size();i++)
        {
			//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "InputChannelsCallback...%d\t%d\td", i,msg->state.size(),m_pInputs[i]->isChecked());
			m_pInputs[i]->setCheckState(msg->state[i]?Qt::Checked:Qt::Unchecked);
			//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "At end InputChannelsCallback");
        }
    }

    //! \brief Callback that handles messages received over the /InputChannels ROS topic.
    //! \param msg The received message.
    //void RobotPanel::OutputChannelsCallback(const cpr_robot::ChannelStates::ConstPtr& msg)
    void RobotPanel::OutputChannelsCallback(const cpr_ros2::msg::ChannelStates::SharedPtr msg)
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OutputChannelsCallback...");
        for(size_t i=0;i<msg->state.size();i++)
        {
            m_pOutputs[i]->setChecked(msg->state[i]);
        }
    }

    //! Updates all UI elements associated with the override value of the robot.
    void RobotPanel::OnOverrideChanged()
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OnOverrideChanged...");
        std::stringstream sstr;
        sstr << ::std::fixed << std::setprecision(1) << 100.0*m_Override << " %";
        m_OverrideLabel.setText(sstr.str().c_str());
    }

    //! Updates all UI elements associated with the status flag reported by the robot.
    void RobotPanel::OnStatusFlagsChanged()
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OnStatusFlagsChanged...");
        if(m_StatusFlags&cpr_robot::Robot::STATUSFLAG_DISCONNECTED)
        { 
			//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "In if 1...");
            m_ConnectButton.setText(tr("Connect"));
            m_EnableButton.setEnabled(false);
            m_OverrideSlider.setEnabled(false);
            m_ReferenceButton.setEnabled(false);
            m_EnableButton.setText(tr("Enable"));
            for(uint32_t i=0;i<m_CountJoints;i++)
                m_pJointControls[i]->Disable();
            for(size_t i=0;i<m_Inputs.size();i++)
                m_pInputs[i]->setEnabled(false);
            for(size_t i=0;i<m_Outputs.size();i++)
                m_pOutputs[i]->setEnabled(false);
        }
        else
        {
			//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "In else 1...");
            m_ConnectButton.setText(tr("Disconnect"));
            m_EnableButton.setEnabled(true);
            m_OverrideSlider.setEnabled(false);
            if(m_StatusFlags&cpr_robot::MotorModule::STATUSFLAG_MASK_ANY_ERROR)
            { 
				//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "In if 2...");
			
				//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joints... %d", m_CountJoints);
                for(uint32_t i=0;i<m_CountJoints;i++)
                {
					//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loop %d...",i);
                    m_pJointControls[i]->Disable();
				}
                m_EnableButton.setText(tr("Enable"));
                m_ReferenceButton.setEnabled(false);
                for(size_t i=0;i<m_Inputs.size();i++)
                    m_pInputs[i]->setEnabled(false);
                for(size_t i=0;i<m_Outputs.size();i++)
                    m_pOutputs[i]->setEnabled(false);
            }
            else
            {
				//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "In else 2...");
                for(uint32_t i=0;i<m_CountJoints;i++)
                    m_pJointControls[i]->Enable();
                m_EnableButton.setText(tr("Disable"));
                m_ReferenceButton.setEnabled(true);
                for(size_t i=0;i<m_Inputs.size();i++)
                    m_pInputs[i]->setEnabled(true);
                for(size_t i=0;i<m_Outputs.size();i++)
                    m_pOutputs[i]->setEnabled(true);
            }
        }
        m_OverrideSliderValid=false;
    }
    
    void RobotPanel::handle_GetRobotInfo_response(rclcpp::Client<cpr_ros2::srv::GetRobotInfo>::SharedFuture future) {
	// Get the response when it becomes available.
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CB Triggered...");
		auto result = future.get();
		// Process the result here.
		m_CountJoints=result->countjoints;
        m_Inputs.clear();
        for(size_t i=0;i<result->inputchannels.size();i++)
            m_Inputs.push_back(result->inputchannels[i]);
        m_Outputs.clear();
        for(size_t i=0;i<result->outputchannels.size();i++)
            m_Outputs.push_back(result->outputchannels[i]);
        m_ModelName=result->model;
        m_StatusFlags=cpr_robot::Robot::STATUSFLAG_DISCONNECTED;
        InitializeUI();
    }

    //! \brief Queries information about the robot from the /GetRobotInfo ROS service.
    cpr_ros2::srv::GetRobotInfo_Response RobotPanel::GetRobotInfo()
    {
    	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Called GetRobotInfo...");
        //cpr_ros2::srv::GetRobotInfo srv;
        auto request = std::make_shared<cpr_ros2::srv::GetRobotInfo::Request>();
        request->sender="cpr_robot::RobotPanel";
        
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Before while...");
        while (!m_GetRobotInfoClient->wait_for_service(1s))
        {
			if(!rclcpp::ok()) {
				//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupted...");
			}
			//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available...");
		}
		//auto result1 = m_GetRobotInfoClient->async_send_request(request);
		//auto result2 = result1.get();		
        //if (rclcpp::spin_until_future_complete(node_ptr, result1) == rclcpp::FutureReturnCode::SUCCESS)
        //{
		//	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "After while...%d", result1.get()->countjoints);
		//}
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "After while...%d", result2.countjoints);
        
    	// Set up request parameters here if needed.

    	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Before async_send_requestcb...");
    	// Send the asynchronous request and provide a callback to handle the response.
    	//m_GetRobotInfoClient->async_send_request(request, std::bind(&RobotPanel::handle_GetRobotInfo_response, this, std::placeholders::_1));
    	
    	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "after async_send_requestcb...");
        
    	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Before async_send_request...");
        cpr_ros2::srv::GetRobotInfo::Response result = cpr_ros2::srv::GetRobotInfo::Response();
        //cpr_ros2::srv::GetRobotInfo::Response result = *m_GetRobotInfoClient->async_send_request(request);
        
        m_GetRobotInfoClient->async_send_request(request).get();
        
    	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "After async_send_request...");
       // if (m_GetRobotInfoClient.call(srv))
        //{
            //ROS_INFO("GetRobotInfo: model %s, %ui joints.",srv.response.Model.c_str(),srv.response.CountJoints);
        //}
        //else
        //{
            //ROS_ERROR("Failed to call service GetRobotInfo.");
          //  srv.response.model="Unknown Robot";
            //srv.response.countjoints=0;
        //}
        return result;
    }
	//
    //! \brief Sends a command to the robot using the /RobotCommand ROS service.
    //! \param commandId The ID of the command that will be sent.
    //! \param payloadFloat A floating point number that may be provided as data with the command.
    //! \param payloadInt An integer number that may be provided as data with the command.
    cpr_ros2::srv::RobotCommand_Response RobotPanel::RobotCommand(const uint32_t commandId, const double payloadFloat, const int64_t payloadInt)
    {
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotCommand...");
    	auto request = std::make_shared<cpr_ros2::srv::RobotCommand::Request>();
       
        request->sender="cpr_robot::RobotPanel";
        request->commandid=commandId;
        request->payloadfloat=payloadFloat;
        request->payloadint=payloadInt;
        m_RobotCommandClient->async_send_request(request);
        cpr_ros2::srv::RobotCommand::Response result;
        //if (!m_RobotCommandClient.call(srv))
        //{
            //ROS_ERROR("Failed to call service RobotCommand.");
        //}
        return result;
    }


}

// Tell pluginlib about this class. 
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cpr_ros2::RobotPanel,rviz_common::Panel)

#include "moc_RobotPanel.cpp"
