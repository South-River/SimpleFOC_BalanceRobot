#include "IMU.h"

namespace IMU
{
    IMU_6DOF::IMU_6DOF(const float& freq, const unsigned short& _filter_type)
    {
        sample_freq = freq;
        inv_sample_freq = 1.f/sample_freq;

        filter_type = _filter_type;
        
        if(filter_type == MAHONY)
        {    
            twoKp = 2.f*15.f;
            twoKi = 2.f*0.f;
        }
        else if(filter_type == MADGWICK)
        {
            beta = .5f;
        }
        else if(filter_type == EKF)
        {} //TODO: finish EKF
        else if(filter_type == UKF)
        {} //TODO: finish UKF
    }

    void IMU_6DOF::getData(const float& accel_x, const float& accel_y, const float& accel_z, 
                        const float& gyro_x, const float& gyro_y, const float& gyro_z)
    {
        accel[0] = (float)accel_x;
        accel[1] = (float)accel_y;
        accel[2] = (float)accel_z;

        gyro[0] = (float)gyro_x;
        gyro[1] = (float)gyro_y;
        gyro[2] = (float)gyro_z;

        accelCalibrate();
        gyroCalibrate();
    }

    void IMU_6DOF::getAccelData(const float& accel_x, const float& accel_y, const float& accel_z)
    {
        accel[0] = (float)accel_x;
        accel[1] = (float)accel_y;
        accel[2] = (float)accel_z;

        accelCalibrate();
    }

    void IMU_6DOF::getGyroData(const float& gyro_x, const float& gyro_y, const float& gyro_z)
    {
        gyro[0] = (float)gyro_x;
        gyro[1] = (float)gyro_y;
        gyro[2] = (float)gyro_z;

        gyroCalibrate();
    }

    void IMU_6DOF::accelCalibrate()
    {
        for(int i=0; i<3; i++)
        {
            accel[i] = accel_scale[i] * (accel[i] - accel_bias[i]);
        }
    }

    void IMU_6DOF::gyroCalibrate()
    {
        for(int i=0; i<3; i++)
        {
            gyro[i] = gyro[i] - gyro_bias[i];
            // TODO: finish Deadzone
            // gyro[i] = (((gyro[i]<0.01745329f)?0.f:gyro[i])>-0.01745329f)?0.f:gyro[i];
        }
    }

    float IMU_6DOF::fastInvSqrt(float x)
    {
        float half_x = 0.5f*x;
        float y = x;

        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y*(1.5f-(half_x*y*y));

        return y;
    }

    void IMU_6DOF::fromQuaternion2RPY()
    {
        roll = atan2(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2]);
        pitch = asin(-2.0f * (q[1]*q[3] - q[0]*q[2]));
        yaw = atan2(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]);

        // yaw *= 3.f;
        // while(true)
        // {
        //     if((yaw>-1.f*IMU_PI)&&yaw<IMU_PI)
        //     {
        //         break;
        //     }
        //     else if(yaw>IMU_PI) yaw-=2*IMU_PI;
        //     else if(yaw<-1.f*IMU_PI) yaw+=2*IMU_PI;
        //     else break;
        // }
        
        euler[0] = roll;
        euler[1] = pitch;
        euler[2] = yaw;
    }

    void IMU_6DOF::changeParam(const unsigned short& param_name, const float& param_value)
    {
        if(param_name == IMU_Kp)
        {
            twoKp = param_value;
        }
        else if(param_name == IMU_Ki)
        {
            twoKi = param_value;
        }
        else if(param_name == IMU_SampleFreq)
        {
            sample_freq = param_value;
            inv_sample_freq = 1./sample_freq;
        }
        else if(param_name == IMU_BETA)
        {
            beta = param_value;
        }
    }

    void IMU_6DOF::updateAccelOffset(const float& x_bias, const float& y_bias, const float& z_bias,
                            const float& x_scale, const float& y_scale, const float& z_scale)
    {
        accel_bias[0] = x_bias;
        accel_bias[1] = y_bias;
        accel_bias[2] = z_bias;

        accel_scale[0] = x_scale;
        accel_scale[1] = y_scale;
        accel_scale[2] = z_scale;
    }

    void IMU_6DOF::updateGyroOffset(const float& x_bias, const float& y_bias, const float& z_bias)
    {
        gyro_bias[0] = x_bias;
        gyro_bias[1] = y_bias;
        gyro_bias[2] = z_bias;
    }

    void IMU_6DOF::updateState()
    {
        if(filter_type == MAHONY)
        {
            mahony();
        }
        else if(filter_type == MADGWICK)
        {
            madgwick();
        }
        else if(filter_type == EKF)
        {   
        }
        else if(filter_type == UKF)
        {
        }
    }

    void IMU_6DOF::updateState(const float &dt)
    {
        if(filter_type == MAHONY)
        {
            mahony(dt);
        }
        else if(filter_type == MADGWICK)
        {
            madgwick(dt);
        }
        else if(filter_type == EKF)
        {   
        }
        else if(filter_type == UKF)
        {
        }
    }

    void IMU_6DOF::mahony()
    {
        float recip_norm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float q0, q1, q2, q3;

        float ax = accel[0], ay = accel[1], az = accel[2];
        float gx = gyro[0],  gy = gyro[1],  gz = gyro[2];

        if(!((ax == 0.0f)&&(ay == 0.0f)&&(az == 0.0f)))
        {
            recip_norm=fastInvSqrt(ax*ax+ay*ay+az*az);
            ax *= recip_norm;
            ay *= recip_norm;
            az *= recip_norm;

            halfvx = q[1]*q[3]-q[0]*q[2];
            halfvy = q[0]*q[1]+q[2]*q[3];
            halfvz = q[0]*q[0]-0.5f+q[3]*q[3];
            
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            if(twoKi > 0.f)
            {
                integralFBx += twoKi * halfex * inv_sample_freq;
                integralFBy += twoKi * halfey * inv_sample_freq;
                integralFBz += twoKi * halfez * inv_sample_freq;
                
                gx += integralFBx;
                gy += integralFBy;
                gz += integralFBz;
            }
            else
            {
                integralFBx = 0.f;
                integralFBy = 0.f;
                integralFBz = 0.f;
            }

            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }
        
        gx *= (0.5f*inv_sample_freq);
        gy *= (0.5f*inv_sample_freq);
        gz *= (0.5f*inv_sample_freq);

        q0 = q[0];
        q1 = q[1];
        q2 = q[2];
        q3 = q[3];
        
        q[0] += (-q1 * gx - q2 * gy - q3 * gz);
        q[1] += (q0 * gx + q2 * gz - q3 * gy);
        q[2] += (q0 * gy - q1 * gz + q3 * gx);
        q[3] += (q0 * gz + q1 * gy - q2 * gx);

        recip_norm = fastInvSqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);

        for(int i=0; i<4; i++)
        {
            q[i] *= recip_norm;
        }

        fromQuaternion2RPY();
    }

    void IMU_6DOF::mahony(const float &dt)
    {
        float recip_norm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float q0, q1, q2, q3;

        float ax = accel[0], ay = accel[1], az = accel[2];
        float gx = gyro[0],  gy = gyro[1],  gz = gyro[2];

        if(!((ax == 0.0f)&&(ay == 0.0f)&&(az == 0.0f)))
        {
            recip_norm=fastInvSqrt(ax*ax+ay*ay+az*az);
            ax *= recip_norm;
            ay *= recip_norm;
            az *= recip_norm;

            halfvx = q[1]*q[3]-q[0]*q[2];
            halfvy = q[0]*q[1]+q[2]*q[3];
            halfvz = q[0]*q[0]-0.5f+q[3]*q[3];
            
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            if(twoKi > 0.f)
            {
                integralFBx += twoKi * halfex * dt;
                integralFBy += twoKi * halfey * dt;
                integralFBz += twoKi * halfez * dt;
                
                gx += integralFBx;
                gy += integralFBy;
                gz += integralFBz;
            }
            else
            {
                integralFBx = 0.f;
                integralFBy = 0.f;
                integralFBz = 0.f;
            }

            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }
        
        gx *= (0.5f*dt);
        gy *= (0.5f*dt);
        gz *= (0.5f*dt);

        q0 = q[0];
        q1 = q[1];
        q2 = q[2];
        q3 = q[3];
        
        q[0] += (-q1 * gx - q2 * gy - q3 * gz);
        q[1] += (q0 * gx + q2 * gz - q3 * gy);
        q[2] += (q0 * gy - q1 * gz + q3 * gx);
        q[3] += (q0 * gz + q1 * gy - q2 * gx);

        recip_norm = fastInvSqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);

        for(int i=0; i<4; i++)
        {
            q[i] *= recip_norm;
        }

        fromQuaternion2RPY();
    }

    void IMU_6DOF::madgwick()
    {
        float recip_norm;
        float s0, s1, s2, s3;
        float q_dot0, q_dot1, q_dot2, q_dot3;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        float ax = accel[0], ay = accel[1], az = accel[2];
        float gx = gyro[0],  gy = gyro[1],  gz = gyro[2];

        q_dot0 = 0.5f*(-q[1]*gx - q[2]*gy - q[3]*gz);
        q_dot1 = 0.5f*( q[0]*gx + q[2]*gz - q[3]*gy);
        q_dot2 = 0.5f*( q[0]*gy - q[1]*gz + q[3]*gx);
        q_dot3 = 0.5f*( q[0]*gz + q[1]*gy - q[2]*gx);

        if(!((ax == 0.0f)&&(ay == 0.0f)&&(az == 0.0f)))
        {
            recip_norm = fastInvSqrt(ax*ax+ay*ay+az*az);
            ax *= recip_norm;
            ay *= recip_norm;
            az *= recip_norm;

            _2q0 = 2.0f * q[0];
            _2q1 = 2.0f * q[1];
            _2q2 = 2.0f * q[2];
            _2q3 = 2.0f * q[3];
            _4q0 = 4.0f * q[0];
            _4q1 = 4.0f * q[1];
            _4q2 = 4.0f * q[2];
            _8q1 = 8.0f * q[1];
            _8q2 = 8.0f * q[2];
            q0q0 = q[0] * q[0];
            q1q1 = q[1] * q[1];
            q2q2 = q[2] * q[2];
            q3q3 = q[3] * q[3];

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;
            
            recip_norm = fastInvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recip_norm;
            s1 *= recip_norm;
            s2 *= recip_norm;
            s3 *= recip_norm;

            // Apply feedback step
            q_dot0 -= beta * s0;
            q_dot1 -= beta * s1;
            q_dot2 -= beta * s2;
            q_dot3 -= beta * s3;
        }

        q[0] += q_dot0 * inv_sample_freq;
        q[1] += q_dot1 * inv_sample_freq;
        q[2] += q_dot2 * inv_sample_freq;
        q[3] += q_dot3 * inv_sample_freq;

        recip_norm = fastInvSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] *= recip_norm;
        q[1] *= recip_norm;
        q[2] *= recip_norm;
        q[3] *= recip_norm;

        fromQuaternion2RPY();
    }

    void IMU_6DOF::madgwick(const float &dt)
    {
        float recip_norm;
        float s0, s1, s2, s3;
        float q_dot0, q_dot1, q_dot2, q_dot3;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        float ax = accel[0], ay = accel[1], az = accel[2];
        float gx = gyro[0],  gy = gyro[1],  gz = gyro[2];

        q_dot0 = 0.5f*(-q[1]*gx - q[2]*gy - q[3]*gz);
        q_dot1 = 0.5f*( q[0]*gx + q[2]*gz - q[3]*gy);
        q_dot2 = 0.5f*( q[0]*gy - q[1]*gz + q[3]*gx);
        q_dot3 = 0.5f*( q[0]*gz + q[1]*gy - q[2]*gx);

        if(!((ax == 0.0f)&&(ay == 0.0f)&&(az == 0.0f)))
        {
            recip_norm = fastInvSqrt(ax*ax+ay*ay+az*az);
            ax *= recip_norm;
            ay *= recip_norm;
            az *= recip_norm;

            _2q0 = 2.0f * q[0];
            _2q1 = 2.0f * q[1];
            _2q2 = 2.0f * q[2];
            _2q3 = 2.0f * q[3];
            _4q0 = 4.0f * q[0];
            _4q1 = 4.0f * q[1];
            _4q2 = 4.0f * q[2];
            _8q1 = 8.0f * q[1];
            _8q2 = 8.0f * q[2];
            q0q0 = q[0] * q[0];
            q1q1 = q[1] * q[1];
            q2q2 = q[2] * q[2];
            q3q3 = q[3] * q[3];

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;
            
            recip_norm = fastInvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recip_norm;
            s1 *= recip_norm;
            s2 *= recip_norm;
            s3 *= recip_norm;

            // Apply feedback step
            q_dot0 -= beta * s0;
            q_dot1 -= beta * s1;
            q_dot2 -= beta * s2;
            q_dot3 -= beta * s3;
        }

        q[0] += q_dot0 * dt;
        q[1] += q_dot1 * dt;
        q[2] += q_dot2 * dt;
        q[3] += q_dot3 * dt;

        recip_norm = fastInvSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] *= recip_norm;
        q[1] *= recip_norm;
        q[2] *= recip_norm;
        q[3] *= recip_norm;

        fromQuaternion2RPY();
    }
}