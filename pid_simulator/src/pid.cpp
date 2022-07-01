#include "pid.h"

Pid::Pid(double kp, double kd, double ki, double maxLimit, double minLimit, bool reverse) 
{
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
    this->maxLimit = maxLimit;
    this->minLimit = minLimit;
    this->reverse = reverse;
}

double Pid::runNextIteration(double output, double setPoint) 
{
    double error = output - setPoint;
    double transit = (kp * error) + (ki * previousError);
    if(transit < this->maxLimit && transit > this->minLimit)
    {
        input = transit;
    }
    else if(transit < this->minLimit)
    {
        input = this->minLimit;
    }
    else if(transit > this->maxLimit)
    {
        input = this->maxLimit;
    }
    previousError = error;
    return reverse ? -input : input;
}

void Pid::updateParameters(double kp, double kd, double ki) 
{
    this->kd = kd;
    this->ki = ki;
    this->kp = kp;
}

void Pid::reset()
{
    currentError = 0;
    previousError = 0;
    setPoint = 0;
    input = 0;
    output = 0;
    for(int i = 0; i < STACK_SIZE; i++)
    {
        previousErrors[i] = 0;
    }
}

double Pid::computeGrossError()
{
    double value = 0;
    double newSetofErrors[10];
    int i = 0;
    for(i = 0; i < sizeof(previousErrors); i++)
    {
        value = value + previousErrors[i];
        if(i != 0)
        {
            newSetofErrors[i - 1] = previousErrors[i];
        }
    }
    newSetofErrors[i != 0 ? i - 1 : i] = currentError;
    return value;
}
