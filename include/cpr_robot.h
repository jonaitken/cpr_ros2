#pragma once

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstdint>
#include <cstring>
#include <atomic>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <cassert>
#include <chrono>
#include <sstream>

#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif


#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#define _USE_MATH_DEFINES
#include <cmath>

#include "cpr_ros2/srv/get_robot_info.hpp"
#include "cpr_ros2/srv/get_joint_info.hpp"
#include "cpr_ros2/msg/robot_state.hpp"
#include "cpr_ros2/msg/channel_states.hpp"
#include "cpr_ros2/srv/robot_command.hpp"

#include "cpr_robot/Bus.h"
#include "cpr_robot/MotorModule.h"
#include "cpr_robot/Joint.h"
#include "cpr_robot/Robot.h"
#include "cpr_robot/CPRMover6.h"

//! \namespace cpr_robot Provides everything needed to control a robot over a CAN bus connection within a ROS environment.
