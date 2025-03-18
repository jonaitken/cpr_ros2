#pragma once

namespace cpr_robot
{
	//! \class Bus Bus.h <cpr_robot.h>
	//! \brief Used for communication over the CAN bus. 
	//!
	//! Allows to send frames and will spawn a reader thread that executes a loop that continuously checks for new frames on the CAN bus and keeps the last frame of any CAN object ID stored in a buffer to be queried at a later time. 
	class Bus
	{
	private:
		//! The hardcoded read timeout in milliseconds.
		static constexpr int32_t m_ReadTimeoutMilliseconds = 50; 
		//! Buffer containing the last received frame of each CAN object ID.	
		struct can_frame m_LastFramesById[0x800];	
		//! Buffer containing the timepoints at which frames of the respective CAN object IDs have been received.
		std::chrono::high_resolution_clock::time_point m_LastFrameTimesById[0x800]; 
		//! The interface name that will be used when creating the socket for communication over the CAN bus.		
		const std::string m_InterfaceName;	
		//! The socket used for communication over the CAN bus. 		
		int m_Socket;	
		//! Flag indicating if the CAN bus has been successfully connected through a socket.							
		bool m_bConnected;			
		//! Flag indicating if the read loop should continue spinning.						
		std::atomic_bool m_bIsRunning; 
		//! Pointer to the thread that asynchronously runs the read loop. Will be set to nullptr if the read loop is not spinning.				
		std::thread* m_pReadThread;	
		//! Mutex used to protect the buffers containing received frames and their associated timepoints from race conditions.		
		std::shared_mutex m_LastFramesMutex; 
		//! Mutex used to ensure that two write operations on the CAN bus cannot interfere with each other.
		std::mutex m_WriteMutex; 
		//! Mutex used to ensure that two read operations on the CAN bus cannot interfere with each other.			
		std::mutex m_ReadMutex;	 	
		void Connect();								
		void Disconnect();							
		static void ReadThread(Bus* pBus);
		void ReadLoop();
		void OnStart();
		void OnStop();
	public:
		double get_ReadTimeout() const;
		void Start();
		void Stop();
		struct can_frame get_LastFrame(const canid_t id, std::chrono::high_resolution_clock::time_point& receptionTime, bool& bValid);
		bool WriteFrame(const struct can_frame& frame);
		const std::string& get_InterfaceName() const;
		Bus(const std::string& interfaceName = "can0");
		virtual ~Bus();
		bool get_IsConnected() const;
	};
}