#include "pid.h"

Pid::Pid(float kp, float kd, float ki) : kp(kp), kd(kd), ki(ki) {}

float Pid::runNextIteration(float input, float setPoint) 
{
    return 0;
}

void Pid::updateParameters(float kp, float kd, float ki) 
{

}