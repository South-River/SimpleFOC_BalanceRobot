#include <SimpleFOC.h>

/* global vars */
float l_force = 0.f;
float r_force = 0.f;

float l_velocity = 0.f;
float r_velocity = 0.f;

MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor0 = BLDCMotor(11);
BLDCDriver3PWM driver0 = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

InlineCurrentSense current_sense0 = InlineCurrentSense(0.01, 50.0, 39, 36);
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, 35, 34);

/* I2C cfgs */
TwoWire I2C0 = TwoWire(0);
TwoWire I2C1 = TwoWire(1);

// #define SLAVE_ADDR (0x02)

#define I2C0_SDA (19)
#define I2C0_SCL (18)
#define I2C1_SDA (23)
#define I2C2_SCL (5)

/* FreeRTOS */
#define CORE_0 (0)
#define CORE_1 (1)

TaskHandle_t Task0;
#define TASK_STACK0 (4*1024)
#define TASK_PRIO0 (10)

TaskHandle_t Task1;
#define TASK_STACK1 (4*1024)
#define TASK_PRIO1 (10)

void task0(void *pvParameters);
void task1(void *pvParameters);

/* function prototype declaration */
void MotorInit();
void MotorRun();
void RcvForce();
void SendVel();

/* main function */
void setup()
{
  /* USB Serial to print data */
  Serial.begin(5e6UL);

  I2C0.begin(I2C0_SDA, I2C0_SCL, 400000UL);
  I2C1.begin(I2C1_SDA, I2C1_SCL, 400000UL);

  /* maximize cpu frequency */
  setCpuFrequencyMhz(240);
  // Serial.println("CPU Frequency: " + String(getCpuFrequencyMhz()));

  MotorInit();

  /* create tasks */
  xTaskCreatePinnedToCore(task0, "Task_MotorRun", TASK_STACK0, NULL, TASK_PRIO0, &Task_0, CORE_0);
  xTaskCreatePinnedToCore(task1, "Task_Msg", TASK_STACK1, NULL, TASK_PRIO1, &Task_1, CORE_1);
}

void loop(){}

void task0(void *pvParameters)
{
    while(true)
    {
        MotorRun();
    }
}

void task1(void *pvParameters)
{
  while(true)
  {
    SendVel();
    RcvForce();
  }
}

void MotorInit()
{
  sensor0.init(&I2C0);
  sensor1.init(&I2C1);

  motor0.linkSensor(&sensor0);
  motor1.linkSensor(&sensor1);

  driver0.voltage_power_supply=12;
  driver0.init();
  motor0.linkDriver(&driver0);

  driver1.voltage_power_supply=12;
  driver1.init();
  motor1.linkDriver(&driver1);

  current_sense0.init();
  current_sense0.gain_b *= -1;
  current_sense0.skip_align = true;
  motor0.linkCurrentSense(&current_sense0);

  current_sense1.init();
  current_sense1.gain_b *= -1;
  current_sense1.skip_align = true;
  motor1.linkCurrentSense(&current_sense1);

  motor0.torque_controller = TorqueControlType::foc_current; 
  motor0.controller = MotionControlType::torque;
  motor1.torque_controller = TorqueControlType::foc_current; 
  motor1.controller = MotionControlType::torque;

  // FOC电流控制PID参数
  motor0.PID_current_q.P = 5;
  motor0.PID_current_q.I= 1000;
  motor0.PID_current_d.P= 5;
  motor0.PID_current_d.I = 1000;
  motor0.LPF_current_q.Tf = 0.002; // 1ms default
  motor0.LPF_current_d.Tf = 0.002; // 1ms default

  motor1.PID_current_q.P = 5;
  motor1.PID_current_q.I= 1000;
  motor1.PID_current_d.P= 5;
  motor1.PID_current_d.I = 1000;
  motor1.LPF_current_q.Tf = 0.002; // 1ms default
  motor1.LPF_current_d.Tf = 0.002; // 1ms default

  // 速度环PID参数
  motor0.PID_velocity.P = 0.05;
  motor0.PID_velocity.I = 1;
  motor0.PID_velocity.D = 0;

  motor1.PID_velocity.P = 0.05;
  motor1.PID_velocity.I = 1;
  motor1.PID_velocity.D = 0;
  
  // 速度限制
  motor0.velocity_limit = 30;
  motor1.velocity_limit = 30;

  //电机初始化
  motor0.init();
  // align encoder and start FOC
  motor0.initFOC(); 
  
  motor1.init();
  // align encoder and start FOC
  motor1.initFOC(); 
}

void MotorRun()
{
  // 目标值
  motor0.target = l_force;
  motor1.target = r_force;
  
  // iterative setting FOC phase voltage
  motor0.loopFOC();
  motor1.loopFOC();

  // iterative function setting the outter loop target
  motor0.move();
  motor1.move();

  l_velocity = motor0.shaftVelocity();
  r_velocity = motor1.shaftVelocity();
}

void RcvForce()
{
  if(Serial.available())
  {
    String s="";

    while(Serial.available())
    {
      s+=char(Serial.read());
    }

    int cnt=0;
    String l_force_s="";
    String r_force_s="";

    while(s[cnt]!=',')
    {
      l_force_s+=s[cnt++];
    }
    cnt++;
    while(cnt<s.length())
    {
      r_force_s+=s[cnt++];
    }

    l_force=l_force_s.toFloat();
    r_force=r_force_s.toFloat();
  }
}

void SendVel()
{
  String l_velocity_s=String(l_velocity, 6);
  String r_velocity_s=String(r_velocity, 6);

  Serial.println(l_velocity_s+','+r_velocity);
}
