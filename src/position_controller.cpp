#include "position_controller.h"

PositionController::PositionController(ros::NodeHandle& nh)
{
    positionFeedbackSubscriber = nh.subscribe("/turtle1/pose", 1000, &PositionController::positionFeedback, this);
    commandPublisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    pidLinearPtr = new Pid(0.4, 0.2, 0.5, 0.5, -0.5, false);
    pidAngularPtr = new Pid(0.085, 0.075, 0.5, 0.1, -0.1, false);
    ROS_INFO("Starting command controller node");
}

void PositionController::positionFeedback(const turtlesim::Pose::ConstPtr& msg)
{
    currentX = msg->x;
    currentY = msg->y;
    double angle = msg->theta * (180/3.14);
    angle = angle < 0 ? 360 + angle : angle; 
    currentTheta = angle;
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
    double theta = atan2 (this->expectedY - this->currentY, this->expectedX - this->currentX);
    theta = theta * (180/3.14);
    theta = theta < 0 ? 360 + theta : theta; 
    return theta;
}

double PositionController::computeDistance()
{
    double distance = sqrt((expectedX - currentX) * (expectedX - currentX)) + ((expectedY - currentY) * (expectedY - currentY));
    return distance;
}

void PositionController::moveToTimer(const ros::TimerEvent& event)
{
    double expectedTheta = computeTangent();
    ROS_INFO("The expected Theta: %lf", expectedTheta);
    double distance = computeDistance();
    ROS_INFO("%lf %lf %lf", currentTheta, expectedTheta, distance);

    if(!indicator)
    {
        angularVelocity = pidAngularPtr->runNextIteration(currentTheta, expectedTheta);
        angularVelocity = abs(angularVelocity);
        double value = round(currentTheta - expectedTheta);
        if(value > -5 && value < 5)
        {
            indicator = true;
            angularVelocity = 0;
        }
    }
    else
    {
        linearVelocity = pidLinearPtr->runNextIteration(round(distance), 0);
    }

    sendCommand(linearVelocity, angularVelocity); 
    ROS_INFO("Linear velocity: %lf, Angular velocity: %lf, currentX: %lf, expectedX: %lf, currentY: %lf, expectedY: %lf, currentTheta: %lf, expectedTheta: %lf", 
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
    positionController.moveTo(3.5, 3.5);
    ros::Timer timer = nh.createTimer(ros::Duration(0.3), &PositionController::moveToTimer, &positionController);
    ros::spin();

    return 0;
}