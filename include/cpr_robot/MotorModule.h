#pragma once

namespace cpr_robot
{
	//! \class MotorModule MotorModule.h <cpr_robot.h>
	//! \brief Represents a DIN rail motorcontrol module from Commonplace Robotics GmbH. 
	//!
	//! Instances of this class will hold information about the current state
	//! of the motor connected to the module and allow to control the module and motor. All position information is stored in ticks. Conversion to actual
	//! joint positions (angles measured in degrees) is done by the Joint class which holds a pointer to an instance of the MotorModule class.
	class MotorModule
	{
    public: 
		//! Error flag indicating that the microcontroller restarted after a brown out because the suppy voltage was too low or µC got stuck.
        static constexpr uint32_t STATUSFLAG_BROWN_OUT = 0x00000001;
        //! Error flag indicating that velocity changes too fast.
		static constexpr uint32_t STATUSFLAG_VELOCITY_LAG = 0x00000002; 
        //! Flag indicating that the motor needs to be enabled by explicit command.
        static constexpr uint32_t STATUSFLAG_MOTOR_NOT_ENABLED = 0x00000004; 
		//! Error flag indicating that the interval without command to the module was too long.
		static constexpr uint32_t STATUSFLAG_COMM_WATCHDOG = 0x00000008; 
		//! Error flag indicating that the position of the motor is too far away from the setpoint position.
        static constexpr uint32_t STATUSFLAG_POSITION_LAG = 0x00000010; 
		//! Error flag indicating that the sequence of quadrature encoder pulses did not fit.
        static constexpr uint32_t STATUSFLAG_ENCODER_ERROR = 0x00000020; 
		//! Error flag indicating that the current value is too high.
        static constexpr uint32_t STATUSFLAG_OVERCURRENT = 0x00000040; 
		//! Error flag indicating that a CAN error occured.
        static constexpr uint32_t STATUSFLAG_CAN_ERROR = 0x00000080; 
		//! Bitmask that should be used to mask for the error bits in status flags.
        static constexpr uint32_t STATUSFLAG_MASK_ANY_ERROR = 0x000000ff; 
	private:
		//! The interval at which periodic SetJoint commands will be sent to the module in milliseconds.
		static constexpr int64_t m_UpdateInterval = 50; 
		//! The module ID of the DIN rail motorcontrol module hat the instance of this class will communicate with.
		const uint8_t m_ModuleId; 
		//! Reference to an instance of the Bus class, providing for communication over the CAN bus.
		Bus& m_Bus; 
		//! Flag indicating if the write loop should keep on spinning.
		std::atomic_bool m_bIsRunning; 
		//! Contains the current position of the motor measured in ticks.
		int32_t m_MotorPosition; 
		//! Contains the increment of the motor position to be made from one SetJoint command to the next. Measured in ticks.
		int32_t m_MotorIncrement; 
		//! Contains the lower bound of the range of allowed motor positions. Measured in ticks.
		int32_t m_MotorMinPosition; 
		//! Contains the upper bound of the range of allowed motor positions. Measured in ticks.
		int32_t m_MotorMaxPosition; 
		//! Contains the offset of the motor position (the "zero position"). Measured in ticks.
		int32_t m_MotorPositionOffset; 
		//! Contains the state of the digital outputs of the module.
		uint8_t m_DOutputs; 
		//! Pointer to the write thread. Will be nullptr if the module is stopped.
		std::thread* m_pWriteThread; 
		//! The timestamp value that will be sent to the module with the next setposition command.
		uint8_t m_CurrentTimeStamp; 
		static void WriteThread(MotorModule* pModule);
		void WriteLoop();
		void Command_SetJoint(const int32_t ticks, const uint8_t doutput);
		void Command_ResetError();
		void Command_DisableMotor();
		void Command_EnableMotor();
		void Command_StartReferencing();
		void Command_SetZeroPosition();
	public:
		uint8_t get_DigitalInputs() const;
		uint8_t get_DigitalOutputs() const;
		void set_DigitalOutputs(const uint8_t doutputs);
		double get_UpdateInterval() const;
		int32_t get_CurrentPosition(uint8_t& timeStamp, std::chrono::high_resolution_clock::time_point& receptionTime,uint8_t& errorFlags, uint8_t& dinputs) const;
		int32_t get_DesiredPosition() const;
		void set_DesiredPosition(const int32_t ticks);
		void set_Increment(const int32_t ticks);
		int32_t get_Increment() const;
		int32_t get_MaxPosition() const;
		void set_MaxPosition(const int32_t ticks);
		int32_t get_MinPosition() const;
		void set_MinPosition(const int32_t ticks);
		int32_t get_Offset() const;
		void set_Offset(const int32_t ticks);
		uint8_t get_ModuleId() const;
		MotorModule(Bus& canbus, const uint8_t moduleId);
		virtual ~MotorModule();
		void Enable();
		void Disable();
		void Stop();
		void Start();
		void StartReferencing();
		void SetZero();
	};
}