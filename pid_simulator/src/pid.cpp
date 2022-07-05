#include "pid.h"

Pid::Pid(double kp, double kd, double ki, double maxLimit, double minLimit, bool reverse) 
{
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
    this->maxLimit = maxLimit;
    this->minLimit = minLimit;
    this->reverse = reverse;

    for(int i = 0; i < STACK_SIZE; i++)
    {
        this->errors[i] = 0;
    }
}

/**
 * @brief Method to run entire pid algorithm once (single iteration run)
 * @param output (double) 
 * @param setPoint (double)
 * @return double 
 */
double Pid::runNextIteration(double output, double setPoint) 
{
    double error = output - setPoint;
    double transit = (this->kp * error) + (this->ki * previousError) + (this->kd * computeGrossError());
    if(transit < this->maxLimit && transit > this->minLimit)
    {
        this->input = transit;
    }
    else if(transit < this->minLimit)
    {
        this->input = this->minLimit;
    }
    else if(transit > this->maxLimit)
    {
       this-> input = this->maxLimit;
    }
    this->previousError = error;
    return reverse ? -this->input : this->input;
}

/**
 * @brief Method to update parameters for PID dynamically
 * @param kp (double) 
 * @param kd (double)
 * @param ki (double)
 */
void Pid::updateParameters(double kp, double kd, double ki) 
{
    this->kd = kd;
    this->ki = ki;
    this->kp = kp;
}

/**
 * @brief Method to reset the PID controller
 */
void Pid::reset()
{
    this->currentError = 0;
    this->previousError = 0;
    this->setPoint = 0;
    this->input = 0;
    this->output = 0;
    for(int i = 0; i < STACK_SIZE; i++)
    {
        this->errors[i] = 0;
    }
}

/**
 * @brief Method to compute gross error over period of time (based on stack size)
 * @return double 
 */
double Pid::computeGrossError()
{
    double value = 0;
    for(int i = 0; i < STACK_SIZE; i++)
    {
        value = value + this->errors[i];
        if(i >= STACK_SIZE)
        {
            this->errors[i] = this->currentError;
        }
        else
        {
            this->errors[i] = this->errors[i+1];
        }
    }
    return value;
}
