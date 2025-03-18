#include <cpr_robot.h>
#include <iostream>

namespace cpr_robot
{
	//! \brief The entry point for the reader thread.
	//! \param pBus Pointer to the Bus instance that the thread will be associated to.
	void Bus::ReadThread(Bus* pBus)
	{
		//ROS_INFO("Bus: Read thread started.");
		pBus->ReadLoop();
		//ROS_INFO("Bus: Read thread terminating.");
	}

	//! \brief Provides information about the timeout for read operations on the CAN bus. 
	//! 
	//! Currently the value is hardcoded to 50ms. If no message is received within that time the underlying socket will generate a timeout error, allowing the thread to yield CPU time to other processes or threads.
	//! \return The timeout delay in seconds.
	double Bus::get_ReadTimeout() const
	{
		return 0.001*(double)m_ReadTimeoutMilliseconds;
	}

	//! \brief The read loop. Will keep spinning until a stop is requested by calling the Stop method or destructor.
	void Bus::ReadLoop()
	{
		struct can_frame frame;
		while (m_bIsRunning)
		{
			{
				std::lock_guard<std::mutex> guard(m_ReadMutex);
				if (read(m_Socket, &frame, sizeof(frame)) == sizeof(frame))
				{
					std::chrono::high_resolution_clock::time_point timePoint=std::chrono::high_resolution_clock::now();
					std::unique_lock<std::shared_mutex> guard(m_LastFramesMutex);
					assert(frame.can_id <= 0x7FF);
					m_LastFramesById[frame.can_id] = frame;
					m_LastFrameTimesById[frame.can_id]=timePoint;
				}
			}
			std::this_thread::yield();
		}
	}

	//! \brief Returns the last frame of a specific CAN object ID that was received from the bus.
	//! \param id The CAN object ID of the requested frame.
	//! \param receptionTime Will be set to the timepoint at which the frame was received from the bus.
	//! \param bValid Will be true if a valid frame of the requested CAN object ID has been received. If false no frame of that CAN object ID has been received yet.
	//! \return The last frame of the requested CAN object ID that was received from the bus.
	struct can_frame Bus::get_LastFrame(const canid_t id, std::chrono::high_resolution_clock::time_point& receptionTime, bool& bValid)
	{
		std::shared_lock<std::shared_mutex> guard(m_LastFramesMutex);
		assert(id <= 0x7FF);
		receptionTime=m_LastFrameTimesById[id];
		bValid=!(m_LastFramesById[id].can_id==0 && m_LastFramesById[id].can_dlc==0);
		return m_LastFramesById[id];
	}

	//! \brief Writes a frame to the CAN bus
	//! \param frame The frame that will be written to the CAN bus.
	//! \return true if the operation was successfull, elsewise false.
	bool Bus::WriteFrame(const struct can_frame& frame)
	{
		std::lock_guard<std::mutex> guard(m_WriteMutex);
		bool bReturn = write(m_Socket, &frame, sizeof(frame)) == sizeof(frame);
		std::this_thread::sleep_for (std::chrono::milliseconds(5));
		return bReturn;
	}

	//! \brief Connects the CAN bus and starts a thread that will asynchronuously spin in a read loop.
	//! \param interfaceName The interface name of the CAN bus. Defaults to "can0".
	Bus::Bus(const std::string& interfaceName) :
		m_InterfaceName(interfaceName),
		m_bIsRunning(true),
		m_bConnected(false),
		m_pReadThread(nullptr),
		m_Socket(-1)
	{
		
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "In Bus Constructor...");
		OnStart();
	}

	//! \brief Requests the read loop to be started. 
	//! 
	//! This is a public thread-safe wrapper for the private OnStart method. A reader thread will be spawned that will poll the CAN bus asynchronuously.
	void Bus::Start()
	{
		std::lock_guard<std::mutex> guard(m_WriteMutex);
		OnStart();
	}

	//! \brief Starts the read loop. 
	//! 
	//! A reader thread will be spawned that will poll the CAN bus asynchronuously.
	void Bus::OnStart()
	{
		
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bus: Starting.");
		//ROS_INFO("Bus: Starting.");
		if(!m_bConnected)
		{
			m_bIsRunning=true;
			for(size_t i=0;i<0x800;i++)
			{
				m_LastFramesById[i].can_id=0;
				m_LastFramesById[i].can_dlc=0;
				m_LastFramesById[i].data[0]=0;
				m_LastFramesById[i].data[1]=0;
				m_LastFramesById[i].data[2]=0;
				m_LastFramesById[i].data[3]=0;
				m_LastFramesById[i].data[4]=0;
				m_LastFramesById[i].data[5]=0;
				m_LastFramesById[i].data[6]=0;
				m_LastFramesById[i].data[7]=0;
				m_LastFrameTimesById[i]=std::chrono::high_resolution_clock::now();
			}
			Connect();
			if (m_bConnected)
			{
				if (m_pReadThread == nullptr)
					m_pReadThread = new std::thread(ReadThread, this);
			}
			else
				m_bIsRunning = false;
		}
		else
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bus: A start request was received while already running.");
	}

	//! \brief Signals the read loop to stop spinning. 
	//! 
	//! This method will wait until the reader thread terminates and joins.
	void Bus::OnStop()
	{
		//ROS_INFO("Bus: Stopping.");
		m_bIsRunning = false;
		if (m_pReadThread != nullptr)
		{
			m_pReadThread->join();
			delete m_pReadThread;
			m_pReadThread = nullptr;
		}
		else
			//ROS_WARN("Bus: A stop request was received while not running.");
		if (m_bConnected)
			Disconnect();
	}

	//! \brief Requests to stop the read loop. 
	//! 
	//! This is a public thread-safe wrapper for the private OnStop method. The reader thread will be terminated and joined.
	void Bus::Stop()
	{
		std::lock_guard<std::mutex> guard(m_WriteMutex);
		OnStop();
	}

	//! \brief Will cause the read loop in case it is spinning. 
	//! 
	//! In that case the reader thread will be terminated and joined.
	Bus::~Bus()
	{
		OnStop();
	}

	//! Opens a socket for communication over the CAN bus.
	void Bus::Connect()
	{
		m_Socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (m_Socket == -1)
		{
			//ROS_ERROR("Bus: Failed to create socket.");
			return;
		}
		struct ifreq ifr;
		strcpy(ifr.ifr_name, m_InterfaceName.c_str());
		int errcode = ioctl(m_Socket, SIOCGIFINDEX, &ifr);
		if (errcode == -1)
		{
			//ROS_ERROR("Bus: Failed to select the CAN interface.");
			return;
		}

		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = m_ReadTimeoutMilliseconds * 1000;
		errcode = setsockopt(m_Socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)& tv, sizeof tv);
		if (errcode == -1)
		{
			//ROS_ERROR("Bus: Failed to set the read timout.");
			return;
		}

		struct sockaddr_can addr;
		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;
		errcode = bind(m_Socket, (struct sockaddr*) & addr, sizeof(addr));
		if (errcode == -1)
		{
			//ROS_ERROR("Bus: Failed to bind to the socket.");
			return;
		}
		//ROS_INFO("Bus: Succesfully connected to CAN-Bus.");
		m_bConnected = true;
	}

	//! \brief Closes the socket that was used for communication over the CAN bus.
	void Bus::Disconnect()
	{
		if (m_Socket == -1)
		{
			//ROS_ERROR("Bus: Attempting to disconnect non-existent socket.");
			return;
		}
		int errcode = close(m_Socket);
		if (errcode == -1)
		{
			//ROS_ERROR("Bus: Failed to close socket.");
		}
		else
		{
			//ROS_INFO("Bus: Succesfully disconnected from CAN-Bus.");
		}
		m_Socket = -1;
		m_bConnected = false;
	}

	//! \brief Allows to query the interface name that was used to open the socket for communication over the CAN bus.
	//! \return The name of the interface.
	const std::string& Bus::get_InterfaceName() const
	{
		return m_InterfaceName;
	}

	//! \brief Provides information about the state of the CAN bus.
	//! \return Returns true if the CAN bus is connected, elsewise false is returned.
	bool Bus::get_IsConnected() const
	{
		return m_bConnected;
	}

}
