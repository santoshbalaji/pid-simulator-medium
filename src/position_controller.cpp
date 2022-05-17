#include "position_controller.h"

PositionController::PositionController(ros::NodeHandle& nh)
{
    positionFeedbackSubscriber = nh.subscribe("/turtle1/pose", 1000, &PositionController::positionFeedback, this);
    commandPublisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    ROS_INFO("Starting command controller node");
}

void PositionController::positionFeedback(const turtlesim::Pose::ConstPtr& msg)
{
    ROS_INFO("testing");
    sendCommand(1, 0);
}

void PositionController::sendCommand(double linearVelocity, double angularVelocity)
{
    geometry_msgs::Twist twist;
    twist.linear.x = linearVelocity;
    commandPublisher.publish(twist);
}

void PositionController::computeVelocities()
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