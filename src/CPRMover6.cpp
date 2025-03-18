#include <cpr_robot.h>


namespace cpr_robot
{
    CPRMover6::CPRMover6(rclcpp::Node::SharedPtr node) :
        Robot(6,0, node)
    {
		
        //set_ModelName("CPRMover6");
        set_GearRatio(0,-528.2);
        set_GearRatio(1,-528.2);
        set_GearRatio(2,528.2);
        set_GearRatio(3,-499.1);
        set_GearRatio(4,1.0);
        set_GearRatio(5,1.0);
        set_JointName(0,"joint1");
        set_JointName(1,"joint2");
        set_JointName(2,"joint3");
        set_JointName(3,"joint4");
        set_JointName(4,"joint5");
        set_JointName(5,"joint6");
        set_TicksPerMotorRotation(0,1440);
        set_TicksPerMotorRotation(1,1440);
        set_TicksPerMotorRotation(2,1440);
        set_TicksPerMotorRotation(3,1440);
        set_TicksPerMotorRotation(4,1024);
        set_TicksPerMotorRotation(5,1024);
        set_MaxVelocity(0,30.0);
        set_MaxVelocity(1,30.0);
        set_MaxVelocity(2,30.0);
        set_MaxVelocity(3,30.0);
        set_MaxVelocity(4,40.0);
        set_MaxVelocity(5,100.0);
        set_MinPosition(0,-130.0);
        set_MaxPosition(0,130.0);
        set_MinPosition(1,-50.0);
        set_MaxPosition(1,60.0);
        set_MinPosition(2,-110.0);
        set_MaxPosition(2,75.0);
        set_MinPosition(3,-140.0);
        set_MaxPosition(3,140.0);
        set_MinPosition(4,-70.0);
        set_MaxPosition(4,60.0);
        set_MinPosition(5,-120.0);
        set_MaxPosition(5,120.0);
        set_MotorOffset(0,0);
        set_MotorOffset(1,0);
        set_MotorOffset(2,0);
        set_MotorOffset(3,0);
        set_MotorOffset(4,500);
        set_MotorOffset(5,509);
        define_Output(false,0,0,"Digital out 1");
        define_Output(false,0,1,"Digital out 2");
        define_Output(false,0,2,"Digital out 3");
        define_Output(false,0,3,"Digital out 4");
        define_Output(false,3,0,"Gripper open");
        define_Output(false,3,1,"Gripper enabled");
        define_Input(false,0,0,"Digital in 1");
        define_Input(false,0,1,"Digital in 2");
        define_Input(false,0,2,"Digital in 3");
    }
            
    CPRMover6::~CPRMover6()
    {

    }

}
