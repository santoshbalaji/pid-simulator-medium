#ifndef PositionController_h
#define PositionController_h

#include "ros/ros.h"
#include "std_msgs/String.h"


class PositionController
{
    public:
        void initialiseNode(int argc, char **argv);
        void computeVelocities();
};

#endif