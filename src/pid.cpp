#include "pid.h"

Pid::Pid(double kp, double kd, double ki) : kp(kp), kd(kd), ki(ki) {}

double Pid::runNextIteration(double output, double setPoint) 
{
    double error = output - setPoint;
    input = (kp * error)+ (ki * previousError) + (kd * computeGrossError());
    previousError = error;
    

    return input;
}

void Pid::updateParameters(double kp, double kd, double ki) 
{
    this->kd = kd;
    this->ki = ki;
    this->kp = kp;
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