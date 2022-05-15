#include "position_controller.h"

PositionController::PositionController(ros::NodeHandle& nh)
{
    positionFeedbackSubscriber = nh.subscribe("/turtle1/pose", 1000, &PositionController::positionFeedback, this);
    ROS_INFO("Starting command controller node");
}

void PositionController::positionFeedback(const turtlesim::Pose::ConstPtr& msg)
{
    ROS_INFO("testing");
}

void PositionController::sendCommand(double linearVelocity, double angularVelocity)
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_controller");
    ros::NodeHandle nh;
    PositionController positionController = PositionController(nh);
    ros::spin();
    return 0;
}