#include <cpr_robot.h>
#include <iostream>

namespace cpr_robot
{
	//! \brief Sets the desired encoder tick increment.
	//! 
	//! The motor position will be incremented by from one SetJoint command to the next by the value provided.
	//! \param ticks Desired increment measured in ticks.
	void MotorModule::set_Increment(const int32_t ticks)
	{
		m_MotorIncrement=ticks;
	}
		
	//! \brief Gets the current encoder tick increment.
	//!
	//! The motor position is incremented by from one SetJoint command to the next by the returned value.
	//! \return Current increment measured in ticks.
	int32_t MotorModule::get_Increment() const
	{
		return m_MotorIncrement;
	}

	//! \brief Constructor of the MotorModule class.
	//! 
	//! Starts a write loop that will asynchronuously send SetJoint commands to the module over the CAN bus in periodic intervals.
	//! \param canbus The bus instance that will be used for communication.
	//! \param moduleId The ID of the DIN rail module that will be controled.
	MotorModule::MotorModule(Bus& canbus, const uint8_t moduleId) :
		m_ModuleId(moduleId),
		m_Bus(canbus),
		m_bIsRunning(true),
		m_pWriteThread(nullptr),
		m_MotorPosition(0),
		m_DOutputs(0),
		m_MotorIncrement(0),
		m_CurrentTimeStamp(0),
		m_MotorMinPosition(-32655),
		m_MotorMaxPosition(32655),
		m_MotorPositionOffset(0)
	{
		Start();
	}

	//! \brief Gets the current state of the digital inputs.
	//! \return Bitmask with the state of the digital inputs, one bit per channel.
	uint8_t MotorModule::get_DigitalInputs() const
	{
		uint8_t timeStamp;
		std::chrono::high_resolution_clock::time_point receptionTime;
		uint8_t errorFlags;
		uint8_t dinputs;
		get_CurrentPosition(timeStamp,receptionTime,errorFlags,dinputs);
		return dinputs;
	}


	//! \brief Gets the current state of the digital outputs.
	//! \return Bitmask with the state of the digital outputs, one bit per channel.
	uint8_t MotorModule::get_DigitalOutputs() const
	{
		return m_DOutputs;
	}
		
	//! \brief Sets the state of the digital outputs.
	//! \param doutputs The desrired states encoded as bitmask. Each bit corresponds to one channel.
	void MotorModule::set_DigitalOutputs(const uint8_t doutputs)
	{
		m_DOutputs=doutputs;
	}


	//! \brief Gets the constant offset to be added to the motor position ("zero position"), measured in ticks.
	//! \return The offset measured in ticks.
	int32_t MotorModule::get_Offset() const
	{
		return m_MotorPositionOffset;
	}
		
	//! \brief Sets the constant offset to be added to the motor position ("zero position"), measured in ticks.
	//! \param ticks The desrired offset measured in ticks.
	void MotorModule::set_Offset(const int32_t ticks)
	{
		m_MotorPositionOffset=ticks;
	}

	//! \brief Destructor of the MotorModule class
	//!
	//! Terminates and joins the thread that is spinning in the write loop that sends periodic SetJoint commands to the module over the CAN bus.
	MotorModule::~MotorModule()
	{
		Stop();
	}

	//! \brief Terminates and joins the thread that is spinning in the write loop that sends periodic SetJoint commands to the module over the CAN bus.
	void MotorModule::Stop()
	{
		m_bIsRunning = false;
		if (m_pWriteThread != nullptr)
		{
			m_pWriteThread->join();
			delete m_pWriteThread;
			m_pWriteThread = nullptr;
		}
	}
	
	//! \brief Starts the thread that is spinning in the write loop that sends periodic SetJoint commands to the module over the CAN bus.
	void MotorModule::Start()
	{
		m_bIsRunning = true;
		m_pWriteThread = new std::thread(WriteThread, this);
	}

	//! \brief Allows to query the ID of the DIN rail module.
	//! \return The ID of the DIN rail module.
	uint8_t MotorModule::get_ModuleId() const
	{
		return m_ModuleId;
	}

	//! \brief Entry point for the write thread.
	//! \param pModule A pointer to the Module instance associated with the write thread.
	void MotorModule::WriteThread(MotorModule * pModule)
	{
		pModule->WriteLoop();
	}

	//! \brief The write loop that will asynchronuously send setposition messages to the module over the CAN bus.
	void MotorModule::WriteLoop()
	{
		//ROS_INFO("Module %u: %s",m_ModuleId, "Write thread started.");
		while (m_bIsRunning)
		{
			std::chrono::high_resolution_clock::time_point last=std::chrono::high_resolution_clock::now();
			set_DesiredPosition(m_MotorPosition+m_MotorIncrement);
			Command_SetJoint(m_MotorPosition, m_DOutputs);
			std::chrono::high_resolution_clock::time_point current=std::chrono::high_resolution_clock::now();
			int64_t ms=(std::chrono::duration_cast<std::chrono::milliseconds>(current - last)).count();
			if(ms<50)
				std::this_thread::sleep_for(std::chrono::milliseconds(m_UpdateInterval-ms));
		}
		//ROS_INFO("Module %u: %s",m_ModuleId, "Write thread terminating.");
	}

	//! \brief Allows to query the desired position of the joint that is controled by the DIN rail module.
	//! \return The current desired joint position measured in encoder ticks.
	int32_t MotorModule::get_DesiredPosition() const
	{
		return m_MotorPosition;
	}

	//! \brief Allows to set the desired position of the joint that is controled by the DIN rail module.
	//! \param ticks The desired joint position measured in encoder ticks. The value will be clamped to the range of valid positions.
	void MotorModule::set_DesiredPosition(const int32_t ticks)
	{
		m_MotorPosition = ticks;
		int32_t M=std::max(m_MotorMinPosition,m_MotorMaxPosition); // values of min and max may be swapped if gear ratios are negative
		int32_t m=std::min(m_MotorMinPosition,m_MotorMaxPosition);
		if(m_MotorPosition<m)
		{
			//ROS_INFO("Module %u: Clamping position %i to minium of %i ticks.",m_ModuleId,m_MotorPosition,m);
			m_MotorPosition=m;
		}
		if(m_MotorPosition>M)
		{
			//ROS_INFO("Module %u: Clamping position %i to maximum of %i ticks.",m_ModuleId,m_MotorPosition,M);
			m_MotorPosition=M;
		}
	}

	//! \brief Gets the interval at which periodic SetJoint commands will be sent to the module in milliseconds. 
	//! 
	//! This value is currently hardcoded to 50ms.
	//! \return The interval measured in seconds.
	double MotorModule::get_UpdateInterval() const
	{
		return 0.001*(double)m_UpdateInterval;
	}


	//! \brief Sends a SetJoint command over the CAN bus to the DIN rail module.
	//! \param ticks The desired joint position given measured encoder ticks.
	//! \param doutput The desired state of the digital outputs.
	void MotorModule::Command_SetJoint(const int32_t ticks, const uint8_t doutput)
	{
		struct can_frame frame;
		frame.can_id = (canid_t)(m_ModuleId << 4);
		frame.can_dlc = 8;
		frame.data[0] = 0x14;
		frame.data[1] = 0x00;
		int32_t positionWithOffset=ticks+m_MotorPositionOffset;
		frame.data[2] = (((uint32_t)positionWithOffset) & 0xFF000000u) >> 24;
		frame.data[3] = (((uint32_t)positionWithOffset) & 0x00FF0000u) >> 16;
		frame.data[4] = (((uint32_t)positionWithOffset) & 0x0000FF00u) >> 8;
		frame.data[5] = ((uint32_t)positionWithOffset) & 0x000000FFu;
		frame.data[6] = m_CurrentTimeStamp;
		m_CurrentTimeStamp=(uint8_t)((((uint32_t)m_CurrentTimeStamp)+1)&0xFF);
		frame.data[7] = doutput;
		m_Bus.WriteFrame(frame);
	}

	//! \brief Sends a ResetError command over the CAN bus to the DIN rail module.
	void MotorModule::Command_ResetError()
	{
		struct can_frame frame;
		frame.can_id = (canid_t)(m_ModuleId << 4);
		frame.can_dlc = 2;
		frame.data[0] = 0x01;
		frame.data[1] = 0x06;
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
		frame.data[4] = 0x00;
		frame.data[5] = 0x00;
		frame.data[6] = 0x00;
		frame.data[7] = 0x00;
		m_Bus.WriteFrame(frame);
	}

	//! \brief Sends a DisableMotor command over the CAN bus to the DIN rail module.
	void MotorModule::Command_DisableMotor()
	{
		struct can_frame frame;
		frame.can_id = (canid_t)(m_ModuleId << 4);
		frame.can_dlc = 2;
		frame.data[0] = 0x01;
		frame.data[1] = 0x0a;
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
		frame.data[4] = 0x00;
		frame.data[5] = 0x00;
		frame.data[6] = 0x00;
		frame.data[7] = 0x00;
		m_Bus.WriteFrame(frame);
	}

	//! \brief Sends a EnableMotor command over the CAN bus to the DIN rail module.
	void MotorModule::Command_EnableMotor()
	{
		struct can_frame frame;
		frame.can_id = (canid_t)(m_ModuleId << 4);
		frame.can_dlc = 2;
		frame.data[0] = 0x01;
		frame.data[1] = 0x09;
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
		frame.data[4] = 0x00;
		frame.data[5] = 0x00;
		frame.data[6] = 0x00;
		frame.data[7] = 0x00;
		m_Bus.WriteFrame(frame);
	}

	//! \brief Sends a StartReferencing command over the CAN bus to the DIN rail module.
	void MotorModule::Command_StartReferencing()
	{
		struct can_frame frame;
		frame.can_id = (canid_t)(m_ModuleId << 4);
		frame.can_dlc = 2;
		frame.data[0] = 0x01;
		frame.data[1] = 0x0b;
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
		frame.data[4] = 0x00;
		frame.data[5] = 0x00;
		frame.data[6] = 0x00;
		frame.data[7] = 0x00;
		m_Bus.WriteFrame(frame);
	}

	//! \brief Sends a SetZeroPosition command over the CAN bus to the DIN rail module.
	void MotorModule::Command_SetZeroPosition()
	{
		struct can_frame frame;
		frame.can_id = (canid_t)(m_ModuleId << 4);
		frame.can_dlc = 4;
		frame.data[0] = 0x01;
		frame.data[1] = 0x08;
		frame.data[2] = 0x00;
		frame.data[3] = 0x00;
		frame.data[4] = 0x00;
		frame.data[5] = 0x00;
		frame.data[6] = 0x00;
		frame.data[7] = 0x00;
		m_Bus.WriteFrame(frame);
	}


	//! \brief Allows to query the last response to a SetJoint command that has been received from the module.
	//! 
	//! If no response has been received yet, zero values will be returned for the current motor position, error flags and data bits.
	//! \param timeStamp The timeStamp sent by the module in the last response to a SetJoint command.
	//! \param receptionTime The time at which the last response to a SetJoint command was received.
	//! \param errorFlags The error flags reported by the module in the last response to a SetJoint command.
	//! \param dinputs The state of the digital inputs as reported by the module in the last response to a SetJoint command.
	//! \return The physical motor position reported by the module in the last response to a SetJoint command.
	int32_t MotorModule::get_CurrentPosition(uint8_t& timeStamp, std::chrono::high_resolution_clock::time_point& receptionTime, uint8_t& errorFlags, uint8_t& dinputs) const
	{
		const canid_t id = (canid_t)((m_ModuleId << 4) + 1);
		bool bValid=false;
		struct can_frame frame = m_Bus.get_LastFrame(id, receptionTime, bValid);
		if(bValid)
		{
			timeStamp=(uint8_t)frame.data[5];
			errorFlags=(uint8_t)frame.data[0];
			dinputs=(uint8_t)frame.data[7];
			return ((int32_t)((((uint32_t)frame.data[1]) << 24) | (((uint32_t)frame.data[2]) << 16) | (((uint32_t)frame.data[3]) << 8) | ((uint32_t)frame.data[4])))-m_MotorPositionOffset;
		}
		else
		{
			receptionTime=std::chrono::high_resolution_clock::now();
			errorFlags=0x00;
			dinputs=0x00;
			return 0;
		}
	}

	//! \brief Enables motor motion.
	void MotorModule::Enable()
	{
		//ROS_INFO("Module %u: %s",m_ModuleId, "Enabling motor.");
		Command_ResetError();
		uint8_t timeStamp;
		uint8_t errorFlags;
		std::chrono::high_resolution_clock::time_point updateTime;
		uint8_t dataBits;
		int32_t real = get_CurrentPosition(timeStamp, updateTime, errorFlags, dataBits);
		m_MotorPosition = real;
		Command_EnableMotor();
		std::this_thread::sleep_for (std::chrono::milliseconds(20));
		Command_EnableMotor();
	}

	//! \brief Disables motor motion.
	void MotorModule::Disable()
	{
		//ROS_INFO("Module %u: %s",m_ModuleId, "Disabling motor.");
		Command_DisableMotor();
	}

	//! \brief Starts the referencing procedure.
	void MotorModule::StartReferencing()
	{
		//ROS_INFO("Module %u: %s",m_ModuleId, "Referencing started.");
		Command_StartReferencing();
		std::this_thread::sleep_for (std::chrono::milliseconds(20));
		Command_StartReferencing();
	}

	//! \brief Sets the encoder position to zero.
	void MotorModule::SetZero()
	{
		//ROS_INFO("Module %u: %s",m_ModuleId, "Setting to zero.");
		Command_SetZeroPosition();
		std::this_thread::sleep_for (std::chrono::milliseconds(20));
		Command_SetZeroPosition();
		uint8_t timeStamp;
		uint8_t errorFlags;
		std::chrono::high_resolution_clock::time_point updateTime;
		uint8_t dataBits;
		m_MotorPosition = get_CurrentPosition(timeStamp, updateTime, errorFlags, dataBits);
	}

	//! \brief Gets the maximally allowed encoder position.
	//! \return The current upper bound of the range of allowed motor positions. Measured in ticks.
	int32_t MotorModule::get_MaxPosition() const
	{
		return m_MotorMaxPosition;
	}
		
	//! \brief Sets the maximally allowed encoder position.
	//! \return The new upper bound of the range of allowed motor positions. Measured in ticks.
	void MotorModule::set_MaxPosition(const int32_t ticks)
	{
		m_MotorMaxPosition=ticks;
	}
		
	//! \brief Gets the minimally allowed encoder position.
	//! \return The current lower bound of the range of allowed motor positions. Measured in ticks.
	int32_t MotorModule::get_MinPosition() const
	{
		return m_MotorMinPosition;
	}
		
	//! \brief Sets the minimally allowed encoder position.
	//! \return The new lower bound of the range of allowed motor positions. Measured in ticks.
	void MotorModule::set_MinPosition(const int32_t ticks)
	{
		m_MotorMinPosition=ticks;
	}

}
