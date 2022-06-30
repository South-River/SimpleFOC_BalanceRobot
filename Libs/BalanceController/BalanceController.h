#ifndef BALANCECONTROLLER_H
#define BALANCECONTROLLER_H

#include <Arduino.h>

#define FLT_MAX (1e10)
#define FLT_MIN (-1e10)

class PID
{
public:
    PID(const float& _Kp, const float& _Ki, const float& _Kd,
        const float& _min = FLT_MIN, const float& _max = FLT_MAX,
        const float& _integral_error_min = FLT_MIN, const float& _integral_error_max = FLT_MAX);
    float calculate(const float& _error);
    void reset();

private:
    float __constrain(const float& _value, const float& _min, const float& _max);

public:
    float Kp;
    float Ki;
    float Kd;

private:    
    float min;
    float max;
    
    float integral_error_min;
    float integral_error_max;

    float integral_error=0.f;
    float derivative_error=0.f;
    float prev_error=0.f;
    
    float prev_timestamp=0.f;
};

#define B_KP (0)
#define B_KD (1)
#define V_KP (2)
#define V_KI (3)
#define R_KP (4)
#define R_KI (5)
#define R_KD (6)

namespace BalanceController
{
    class BalanceController
    {
    public:
        void updateParam(const unsigned short& param_name, const float& param_value);
        float balancePID(const float& target_angle, const float& angle);
        float velocityPID(const float& target_velocity, const float& velocity);
        float rotatePID(const float& target_angular_velocity,const float &angular_velocity);
        void reset();

    public:
        // PID balance_PID = PID(.27, .0f, 0.013f, -12.f, 12.f);//1.3, 0.07
        // PID velocity_PID = PID(.1f, 0.0002f, .0f, -12.f, 12.f, -10000.f, 10000.f);
        // PID rotate_PID = PID(0.f, 0.f, 0.f);
        PID balance_PID = PID(0.245000f, .0f, 0.011500f, -12.f, 12.f);//1.3, 0.07
        PID velocity_PID = PID(0.205000f, 0.000456f, .0f, -12.f, 12.f, -10000.f, 10000.f);
        PID rotate_PID = PID(0.300000f, 0.f, 0.010000f);
    };
}

#endif
