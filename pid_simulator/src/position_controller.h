#ifndef PositionController_h
#define PositionController_h

#include "string.h"
#include "math.h"
#include <cmath>
#include "ros/ros.h"
#include "ros/console.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "pid_simulator_msgs/Target.h"
#include "pid_simulator_msgs/PidTuner.h"
#include "pid.h"

#define NODE_NAME "position_controller"
#define TIMER_FREQUENCY 0.3
#define MAX_LINEAR_VELOCITY 0.5
#define MIN_LINEAR_VELOCITY -0.5
#define MAX_ANGULAR_VELOCITY 0.5
#define MIN_ANGULAR_VELOCITY -0.5
#define DISTANCE_THRESHOLD 1
#define STEERING_ANGLE_THRESHOLD 0.5


class PositionController
{
    public:
        PositionController(ros::NodeHandle& nh);
        void moveToTimer(const ros::TimerEvent& event);

    private:
        ros::Publisher velocityPublisher;
        ros::Subscriber pidLinearParamsSubscriber;
        ros::Subscriber pidAngularParamsSubscriber;
        ros::Subscriber targetSubscriber;
        ros::Subscriber positionSubscriber;

        double currentX, currentY, currentTheta;
        double expectedX, expectedY, expectedTheta;
        double linearVelocity, angularVelocity;
        double lp, li, ld;
        double ap, ai, ad;

        Pid *pidLinearPtr, *pidAngularPtr;

        void setTarget(const pid_simulator_msgs::Target::ConstPtr& msg);
        void setPidLinearParms(const pid_simulator_msgs::PidTuner::ConstPtr& msg);
        void setPidAngularParms(const pid_simulator_msgs::PidTuner::ConstPtr& msg);
        void setPosition(const turtlesim::Pose::ConstPtr& msg);

        double computeTangent(double fromX, double fromY, double toX, double toY);
        double computeDistance(double fromX, double fromY, double toX, double toY);
};

#endif