#include "BalanceController.h"

PID::PID(const float& _Kp, const float& _Ki, const float& _Kd,
         const float& _min, const float& _max,
         const float& _integral_error_min, const float& _integral_error_max)
{
    Kp=_Kp;
    Ki=_Ki;
    Kd=_Kd;
    
    if(_min>_max)
    {
        Serial.println("PID Error, min output shoud small than max output");
    }

    min=_min;
    max=_max;
    integral_error_min=_integral_error_min;
    integral_error_max=_integral_error_max;
    prev_timestamp=float(micros())*1e-6;
}

float PID::calculate(const float& _error)
{
    float timestamp = float(micros())*1e-6;
    float delta_t = timestamp - prev_timestamp;
    prev_timestamp = timestamp;

    float propotional = Kp*_error;
    
    if((prev_error == 0.f)&&(integral_error==0.f))
    {
        integral_error += delta_t*_error;
    }
    else
    {
        integral_error += .5f*delta_t*(prev_error + _error);
    }
    integral_error = __constrain(integral_error, FLT_MIN, FLT_MAX);
    float integral = Ki*integral_error;

    if((prev_error == 0.f)&&(derivative_error==0.f))
    {
        derivative_error=0.f;
    }
    else
    {
        derivative_error = (_error - prev_error)/delta_t;
    }
    float derivative = Kd*derivative_error;

    float output = __constrain((propotional + integral + derivative), min, max);

    prev_error = _error;

    return output;
}

float PID::__constrain(const float& _value, const float& _min, const float& _max)
{
    float value = ((_value<_min)?_min:((_value>_max)?_max:_value));
    return value;
}

void PID::reset()
{
    integral_error = 0.f;
    prev_error = 0.f;

    // prev_timestamp=float(micros())*1e-6;
}

namespace BalanceController
{
    void BalanceController::updateParam(const unsigned short& param_name, const float& param_value)
    {
        switch(param_name)
        {
            case B_KP: balance_PID.Kp=param_value; break;
            case B_KD: balance_PID.Kd=param_value; break;
            case V_KP: velocity_PID.Kp=param_value; break;
            case V_KI: velocity_PID.Ki=param_value; break;
            case R_KP: rotate_PID.Kp=param_value; break;
            case R_KI: rotate_PID.Ki=param_value; break;
            case R_KD: rotate_PID.Kd=param_value; break;
            default: break;
        }
    }

    float BalanceController::balancePID(const float& target_angle, const float& angle)
    {
        float error = target_angle - angle;
        return balance_PID.calculate(error); // return a force
    }

    float BalanceController::velocityPID(const float& target_velocity, const float& velocity)
    {
        float error = target_velocity - velocity;
        return velocity_PID.calculate(error); // return a angle
    }

    float BalanceController::rotatePID(const float& target_angular_velocity,const float &angular_velocity)
    {
        float error = target_angular_velocity - angular_velocity;
        return rotate_PID.calculate(error); //return a force;
    }

    void BalanceController::reset()
    {
        velocity_PID.reset();
    }
}