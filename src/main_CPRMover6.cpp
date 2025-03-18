#include <rclcpp/rclcpp.hpp>
#include <cpr_robot.h>
#include <chrono>

using namespace std::chrono_literals;

class CPRMoverNode : public rclcpp::Node
{
public:
  CPRMoverNode()
      : Node("CPRMover6")
  {
  
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "In CPRMoverNode constructor...");
    // Initialize robot
    // robot_.Init(this->shared_from_this());

    // Create a timer to call the update method at a frequency of 10Hz
    //timer_ = this->create_wall_timer(100ms, std::bind(&CPRMoverNode::update, this));
  }

private:
  // Timer callback function that mimics the behavior of the original loop
  void update()
  {
	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Blah...");
    //robot_.Read();
    //robot_.PublishState();
    //robot_.Write();
  }
  // The CPRMover6 robot instance
  //cpr_robot::CPRMover6 robot_;
  
  // A timer to handle the periodic execution of update()
  
};

/*void update(cpr_robot::CPRMover6 robot_)
{
  // Your update logic goes here
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Blah...");
  robot_.Read();
  robot_.PublishState();
  robot_.Write();
}*/

int main(int argc, char **argv)
{
  // Initialize the ROS2 system
  rclcpp::init(argc, argv);

  // Create the node
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("CPRMoverNode");

  // Initialize CPRMover6 with the node
  cpr_robot::CPRMover6 myRobot(node);

  // Create a timer that calls the update function every 100ms
  // Using a lambda function with explicit capture
  /*auto timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [&myRobot]() { update(myRobot); }
  );*/

  // Spin the node to process callbacks
  rclcpp::spin(node);

  // Shutdown the ROS2 system
  rclcpp::shutdown();

  return 0;
}

