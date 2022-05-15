#ifndef PositionController_h
#define PositionController_h

#include "string.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "turtlesim/Pose.h"


class PositionController
{
    public:
        PositionController(ros::NodeHandle& nh);
        void computeVelocities();
        void positionFeedback(const turtlesim::Pose::ConstPtr& msg);
        void sendCommand(double linearVelocity, double angularVelocity);

    private:
        ros::Publisher commandPublisher;
        ros::Subscriber positionFeedbackSubscriber;
        
};

#endif