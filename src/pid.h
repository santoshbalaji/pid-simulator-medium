#ifndef Pid_h
#define Pid_h

#define STACK_SIZE 10

class Pid
{
    private:
        double kp;
        double kd;
        double ki;
        double currentError;
        double previousError;
        double previousErrors[STACK_SIZE];
        double setPoint;
        double input;
        double output;
        double computeGrossError();
    public:
        Pid(double kp, double kd, double ki);
        double runNextIteration(double output, double setPoint);
        void updateParameters(double kp, double ki, double kd);
};

#endif