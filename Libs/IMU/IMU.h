#ifndef IMU_H
#define IMU_H

#include <math.h>

#define IMU_PI (3.1415926535897932384626433832795028841971693993751058209f)
#define RAD2ANG (180.f/3.1415926535897932384626433832795028841971693993751058209f)
#define ANG2RAD (3.1415926535897932384626433832795028841971693993751058209f/180.f)

#define IMU_SampleFreq (0)
#define IMU_Ki (1)
#define IMU_Kp (2)
#define IMU_BETA (3)

#define MAHONY (0)
#define MADGWICK (1)
#define EKF (2)
#define UKF (3)

namespace IMU
{    
    class IMU_6DOF
    {
    public:
        IMU_6DOF(const float& freq = 400., const unsigned short& _filter_type = 0);
        void getData(const float& accel_x, const float& accel_y, const float& accel_z, 
                    const float& gyro_x,  const float& gyro_y,  const float& gyro_z);
        void getAccelData(const float& accel_x, const float& accel_y, const float& accel_z);
        void getGyroData(const float& gyro_x, const float& gyro_y, const float& gyro_z);
        void accelCalibrate();
        void gyroCalibrate();
        void fromQuaternion2RPY();
        void changeParam(const unsigned short& param_name, const float& param_value);
        void updateAccelOffset(const float& x_bias, const float& y_bias, const float& z_bias,
                            const float& x_scale = 1., const float& y_scale = 1., const float& z_scale = 1.);
        void updateGyroOffset(const float& x_bias, const float& y_bias, const float& z_bias);
        
    public:
        void updateState();
        void updateState(const float &dt);
        
    private:
        float fastInvSqrt(float x);

    private:
        void mahony();
        void mahony(const float &dt);
        void madgwick();
        void madgwick(const float &dt);

    public:
        float accel[3]={0.};
        float gyro[3]={0.};
        float q[4]={1., 0., 0., 0.};
        float euler[3]={0.};
        float roll=0., pitch=0., yaw=0.;
        float sample_freq;

    private:
        float accel_bias[3]={0.};
        float accel_scale[3]={1.,1.,1.};
        float gyro_bias[3]={0.};
        
        float twoKp;
        float twoKi;
        
        float beta;

        float filter_type;

        float integralFBx = 0., integralFBy = 0., integralFBz = 0.;
        float inv_sample_freq; 
    };
}

#endif