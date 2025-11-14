#ifndef PID_H
#define PID_H


#include <math.h>

class PID
{
private:
    float min_val_,max_val_;
    float KP, KI, KD;

    float kp,kpT,ki,kiT,kd,kdT;

    float angular_vel_Prev;

    struct e
    {
        float proportional;
        float integral;
        float derivative;
        float u;
        float preveious;
    }err;

    float lowpass_filt = 0;
    float lowpass_prev = 0;
    float radian = 0;
    float eProportional;
    float deg2target = 0;

    float encPrev;

    float angular_vel_Filt = 0;
    float angular_vel = 0;

    float PPR = 0;

    float error_integral = 0;
    float error_previous = 0;

    float eIntegral = 0;
    float prevError = 0;

public:

    void ppr_total(float total_ppr)
    {
       PPR = total_ppr; 
    }
    
    PID(float MIN_VAL, float MAX_VAL, float kp_, float ki_, float kd_):
    min_val_(MIN_VAL),max_val_(MAX_VAL),
    KP(kp_),
    KI(ki_),
    KD(kd_)
    {

    }

    float control_base(float error , float deltaT)
    {
        err.proportional = error;
        err.integral += err.proportional * deltaT;

        float eDerivative = (err.proportional - err.preveious) / deltaT;
        err.preveious = err.proportional;

        float u =  KP * err.proportional + KI * err.integral + KD * eDerivative ;

        return fmax(min_val_, fmin(u, max_val_));

    }

    float control_base_rotation(float error, float deltaT)
    {
        err.proportional = error;

        if(err.proportional > 180)
        {
            err.proportional -= 360;
        }
        else if(err.proportional < -180)
        {
            err.proportional += 360;
        }
        err.integral += err.proportional * deltaT;

        float eDerivative = (err.proportional - err.preveious) / deltaT;
        err.preveious = err.proportional;

        float u = KP * err.proportional + KI * err.integral + KD * eDerivative;

        return fmax(min_val_, fmin(u, max_val_));

    }

    float get_filt_vel()
    {
        return angular_vel_Filt;
    }
};

#endif