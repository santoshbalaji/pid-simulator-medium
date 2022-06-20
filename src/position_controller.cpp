#include "position_controller.h"

PositionController::PositionController(ros::NodeHandle& nh)
{
    positionFeedbackSubscriber = nh.subscribe("/turtle1/pose", 1000, &PositionController::positionFeedback, this);
    commandPublisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    pidLinearPtr = new Pid(0.4, 0.2, 0.5, 0.5, -0.5, true);
    pidAngularPtr = new Pid(0.4, 0.2, 0.5, 0.5, -0.5, true);
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

double PositionController::computeTangent()
{
    double adjacentDistance = sqrt((this->expectedX * this->expectedX) + (this->expectedY * this->expectedY));
    double oppositeDistance = sqrt((this->currentX * this->currentX) + (this->currentY * this->currentY));
    double theta = tan(oppositeDistance / adjacentDistance);
    return theta;
}

void PositionController::moveToTimer(const ros::TimerEvent& event)
{
    double expectedTheta = computeTangent();   
    angularVelocity = pidAngularPtr->runNextIteration(currentTheta, expectedTheta);
    linearVelocity = pidLinearPtr->runNextIteration(currentX, expectedX);
    sendCommand(linearVelocity, angularVelocity); 
    ROS_INFO("Linear velocity: %f, Angular velocity: %f, currentX: %f, expectedX: %f, currentY: %f, expectedY: %f, currentTheta: %f, expectedTheta: %f", 
        linearVelocity, angularVelocity, currentX, expectedX, currentY, expectedY, currentTheta, expectedTheta);
}

void PositionController::moveTo(double x, double y)
{
    this->expectedX = x;
    this->expectedY = y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_controller");
    ros::NodeHandle nh;
    PositionController positionController = PositionController(nh);
    positionController.moveTo(2.5, 5.5);
    ros::Timer timer = nh.createTimer(ros::Duration(0.5), &PositionController::moveToTimer, &positionController);
    ros::spin();

    return 0;
}