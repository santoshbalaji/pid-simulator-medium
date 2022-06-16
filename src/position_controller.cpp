#include "position_controller.h"

PositionController::PositionController(ros::NodeHandle& nh)
{
    positionFeedbackSubscriber = nh.subscribe("/turtle1/pose", 1000, &PositionController::positionFeedback, this);
    commandPublisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    ROS_INFO("Starting command controller node");
}

void PositionController::positionFeedback(const turtlesim::Pose::ConstPtr& msg)
{
    currentX = msg->x;
    currentY = msg->y;
    currentTheta = msg->theta;
}

void PositionController::sendCommand(double linearVelocity, double angularVelocity)
{
    geometry_msgs::Twist twist;
    twist.linear.x = linearVelocity;
    twist.angular.z = angularVelocity;
    commandPublisher.publish(twist);
}

void PositionController::moveTo(double x, double y)
{
    Pid pid(0.1, 0.1, 0.1);
    while (currentX != x)
    {
       linearVelocity = pid.runNextIteration(currentX, x) + linearVelocity;
       sendCommand(linearVelocity, angularVelocity); 
       ROS_INFO("Linear velocity: %f, Angular velocity: %f", linearVelocity, angularVelocity);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_controller");
    ros::NodeHandle nh;
    PositionController positionController = PositionController(nh);
    positionController.moveTo(1, 0);
    ros::spin();
    return 0;
}