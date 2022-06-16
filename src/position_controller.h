#ifndef PositionController_h
#define PositionController_h

#include "string.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "pid.h"


class PositionController
{
    public:
        PositionController(ros::NodeHandle& nh);
        void positionFeedback(const turtlesim::Pose::ConstPtr& msg);
        void sendCommand(double linearVelocity, double angularVelocity);
        void moveTo(double x, double y);

    private:
        ros::Publisher commandPublisher;
        ros::Subscriber positionFeedbackSubscriber;
        double currentX, currentY, currentTheta;
        double linearVelocity, angularVelocity;

};

#endif