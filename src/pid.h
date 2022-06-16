#ifndef Pid_h
#define Pid_h

#define STACK_SIZE 10

class Pid
{
    private:
        double kp, ki, kd;
        double currentError, previousError;
        double previousErrors[STACK_SIZE];
        double setPoint, input, output;
        double computeGrossError();
    public:
        Pid(double kp, double kd, double ki);
        double runNextIteration(double output, double setPoint);
        void updateParameters(double kp, double ki, double kd);
        void reset();
};

#endif