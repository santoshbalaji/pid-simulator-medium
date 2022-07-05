#include "position_controller.h"

PositionController::PositionController(ros::NodeHandle& nh)
{
    positionSubscriber = nh.subscribe("/turtle1/pose", 1000, &PositionController::setPosition, this);
    targetSubscriber = nh.subscribe("/simulator/target", 1000, &PositionController::setTarget, this);
    pidLinearParamsSubscriber = nh.subscribe("simulator/pid_linear_params", 1000, &PositionController::setPidLinearParms, this);
    pidAngularParamsSubscriber = nh.subscribe("simulator/pid_angular_params", 1000, &PositionController::setPidAngularParms, this);
    velocityPublisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

    pidLinearPtr = new Pid(this->lp, this->li, this->ld, MAX_LINEAR_VELOCITY, MIN_LINEAR_VELOCITY, false);
    pidAngularPtr = new Pid(this->ap, this->ai, this->ad, MAX_ANGULAR_VELOCITY, MIN_ANGULAR_VELOCITY, false);
    ROS_INFO("Starting command controller node");
}

/**
 * @brief Callback used by timer. Holds entire logic to move agent towards target
 * @param event 
 */
void PositionController::moveToTimer(const ros::TimerEvent& event)
{
    double expectedTheta = computeTangent(this->currentX, this->currentY, this->expectedX, this->expectedY);
    double distance = computeDistance(this->currentX, this->currentY, this->expectedX, this->expectedY);

    angularVelocity = pidAngularPtr->runNextIteration(currentTheta, expectedTheta);
    linearVelocity = pidLinearPtr->runNextIteration(round(distance), 0);

    double steeringAngleError = round(currentTheta - expectedTheta);
    if(steeringAngleError < STEERING_ANGLE_THRESHOLD)
    {
        angularVelocity = 0;
    }
    if(distance < DISTANCE_THRESHOLD)
    {
        linearVelocity = 0;
    }

    geometry_msgs::Twist twist;
    twist.linear.x = linearVelocity;
    twist.angular.z = angularVelocity;
    velocityPublisher.publish(twist);

    ROS_INFO("Linear velocity: %lf, Angular velocity: %lf, currentX: %lf, expectedX: %lf, currentY: %lf, expectedY: %lf, currentTheta: %lf, expectedTheta: %lf", 
        linearVelocity, angularVelocity, currentX, expectedX, currentY, expectedY, currentTheta, expectedTheta);
}

/**
 * @brief Callback method to set new target for agent
 * @param msg (pid_simulator_msgs::Target)
 */
void PositionController::setTarget(const pid_simulator_msgs::Target::ConstPtr& msg)
{
    this->expectedX = msg->x;
    this->expectedY = msg->y;
    this->expectedTheta = msg->theta;
}

/**
 * @brief Callback method to set params for PID algoirthm (Linear velocity tuning) 
 * @param msg (pid_simulator_msgs::PidTuner)
 */
void PositionController::setPidLinearParms(const pid_simulator_msgs::PidTuner::ConstPtr& msg)
{
    this->lp = msg->p;
    this->li = msg->i;
    this->ld = msg->d;
}

/**
 * @brief Callback method to set params for PID algoirthm (Angular velocity tuning) 
 * @param msg (pid_simulator_msgs::PidTuner)
 */
void PositionController::setPidAngularParms(const pid_simulator_msgs::PidTuner::ConstPtr& msg)
{
    this->ap = msg->p;
    this->ai = msg->i;
    this->ad = msg->d;
}

/**
 * @brief Callback method to set position information from  
 * @param msg (turtlesim::Pose)
 */
void PositionController::setPosition(const turtlesim::Pose::ConstPtr& msg)
{
    this->currentX = msg->x;
    this->currentY = msg->y;
    double angle = msg->theta * (180/3.14);
    angle = angle < 0 ? 360 + angle : angle; 
    this->currentTheta = angle;
}

/**
 * @brief Method to compute distance between two given points
 * @return double 
 */
double PositionController::computeDistance(double fromX, double fromY, double toX, double toY)
{
    double distance = sqrt((toX - fromX) * (toX - fromX)) + ((toY - fromY) * (toY - fromY));
    return distance;
}

/**
 * @brief Method to compute angle between two given points
 * @return double 
 */
double PositionController::computeTangent(double fromX, double fromY, double toX, double toY)
{
    double theta = atan2 (toY - fromY, toX - fromX);
    theta = theta * (180/3.14);
    theta = theta < 0 ? 360 + theta : theta; 
    return theta;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;
    PositionController positionController = PositionController(nh);
    ros::Timer timer = nh.createTimer(ros::Duration(TIMER_FREQUENCY), &PositionController::moveToTimer, &positionController);
    ros::spin();
    return 0;
}