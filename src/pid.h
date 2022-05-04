#ifndef Pid_h
#define Pid_h

class Pid
{
    private:
        float kp;
        float kd;
        float ki;
        float setPoint;
        float input;
        float output;
    public:
        Pid(float kp, float kd, float ki);
        float runNextIteration(float input, float setPoint);
        void updateParameters(float kp, float ki, float kd);
};

#endif