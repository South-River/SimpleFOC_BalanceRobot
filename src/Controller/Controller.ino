#include <SimpleFOC.h>

#include <BalanceController.h>

#include "Config.h"

/* global vars */
int state;

float roll=0.f;
float pitch=0.f;

float l_force = 0.f;
float r_force = 0.f;

float l_velocity = 0.f;
float r_velocity = 0.f;

float vel = 0.f;           // m/s
float angvel = 0.f;   // rad/s

/* motor vars */
MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor0 = BLDCMotor(11);
BLDCDriver3PWM driver0 = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

/* controller vars */
BalanceController::BalanceController balance_controller;
float target_velocity;
float target_angle;
float target_angular_velocity;
float vel_sum=0.f;

float robot_velocity;
float robot_angle;
float robot_angular_velocity;

/* I2C cfgs */
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

/* function prototype declaration */
void getSerialData();
void MotorInit();
void MotorRun();

#ifdef DEBUG
void printInfo();
void updateParam();
#endif

/* FreeRTOS */
#define CORE_0 (0)
#define CORE_1 (1)

TaskHandle_t Task0;
#define TASK_STACK0 (10*1024)
#define TASK_PRIO0 (2)

TaskHandle_t Task1;
#define TASK_STACK1 (10*1024)
#define TASK_PRIO1 (10)

void task0(void *pvParameters);
void task1(void *pvParameters);

/* main function */
void setup()
{
  /* USB Serial to print data */
  Serial.begin(Serial0BaudRate);
  Serial1.begin(Serial1BaudRate, SERIAL_8N1, Serial1_TX, Serial1_RX);

  I2Cone.begin(I2C0_SDA,I2C0_SCL, I2C0Freq);   //SDA0,SCL0
  I2Ctwo.begin(I2C1_SDA,I2C1_SCL, I2C1Freq);

  /* maximize cpu frequency */
  setCpuFrequencyMhz(240);
  Serial.println("CPU Frequency: " + String(getCpuFrequencyMhz()));

  MotorInit();

  /* create tasks */
  xTaskCreatePinnedToCore(task0, "Task_MotorRun", TASK_STACK0, NULL, TASK_PRIO0, &Task0, CORE_0);
  xTaskCreatePinnedToCore(task1, "Task_FSM", TASK_STACK1, NULL, TASK_PRIO1, &Task1, CORE_1);
}

void loop()
{}

void task0(void *pvParameters)
{
  while(true)
  {
    MotorRun();
    // vTaskDelay(1);
  }
}

void task1(void *pvParameters)
{
  float force = 0.f;
  float rotate_force = 0.f;
  l_force = 0.f, r_force = 0.f;

  /* target state */
  target_velocity = vel;                        //m/s
  target_angle = 0.f;                           //rad
  target_angular_velocity = angvel;             //rad/s
  
  /* current state */
  robot_velocity = 0.f;
  robot_angle = 0.f;
  robot_angular_velocity = 0.f;
  l_velocity = 0.f, r_velocity = 0.f;           //rad/s

  float len = 1.f;
  float wheel_radius=1.f;

  String l_force_s="", r_force_s="";

  uint8_t state = FALL;

  unsigned long FSM_timer=micros();
  unsigned long State_timer=micros();
  float FSM_freq=FSMFreq;

  vel_sum=0.f;

  while(true)
  {
    if (Serial1.available())
    {
      getSerialData();
    }
    #ifdef DEBUG
    if (Serial.available())
    {
      updateParam();
    }
    #endif
    
    if((micros()-FSM_timer)<1e6*1.f/FSM_freq &&(micros()>FSM_timer))
    {
      continue;
    }
    FSM_timer=micros();

    robot_angle=pitch;
    robot_velocity = ((l_velocity - r_velocity)/2)*wheel_radius;
    robot_angular_velocity = (l_velocity + r_velocity)/(2*len);

    target_velocity = vel;
    target_angular_velocity = angvel;
    
    if(fabs(robot_angle)<.5f*FALL_ANG&&roll<90)
    {
      if(state==FALL)
      {
        if(micros()-State_timer>2*1e6)
        {
          state=NORMAL;
          vel_sum=0.f;
        }
      }
    }
    else if(fabs(robot_angle)>FALL_ANG||roll>90)
    {
      state=FALL;
      State_timer=micros();
    }

    switch(state)
    {
      case NORMAL:
      {    
        float vel_err=target_velocity-robot_velocity;
        vel_sum+=vel_err;
        vel_sum=(vel_sum<MAX_SUM)?vel_sum:MAX_SUM;
        vel_sum=(vel_sum>MIN_SUM)?vel_sum:MIN_SUM;

        #ifdef CascadePID
        float new_target_angle = -balance_controller.velocity_PID.Kp*vel_err+balance_controller.velocity_PID.Ki*vel_sum;
        target_angle = .8f*target_angle +.2f*new_target_angle;
        target_angle = constrain(target_angle, -45, 45);

        float newforce = -balance_controller.balancePID(target_angle, robot_angle);
        force = .0f*force+1.f*newforce;
        #endif
        #ifdef ParallelPID
        force = -balance_controller.balancePID(0.f, robot_angle)-balance_controller.velocity_PID.Kp*vel_err+balance_controller.velocity_PID.Ki*vel_sum;
        // newforce -= balance_controller.velocityPID(target_velocity, robot_velocity);
        #endif
        rotate_force = 0.8f*rotate_force+0.2f*balance_controller.rotatePID(target_angular_velocity, robot_angular_velocity);
        // Serial.println(target_angular_velocity);
        l_force=force+rotate_force;
        r_force=-force+rotate_force;
        break;
      }
      case FALL:
      {      
        balance_controller.reset();
        force = 0.f;
        l_force=force;
        r_force=-force;
        break;
      }
      default: break;
    }
    #ifdef DEBUG
    // printInfo();
    #endif
  }
}

void getSerialData()
{
  String buffer="";
  while(Serial1.available())buffer+=char(Serial1.read());

  float tmp0=0.f;
  float tmp1=0.f;

  int value_num=sscanf(buffer.c_str(),"Roll:%fPitch:%f",&tmp0,&tmp1);
  
  if(value_num==2)
  {
    roll=tmp0;
    pitch=tmp1;
  }
}

void MotorInit()
{  
  sensor0.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  
  motor0.linkSensor(&sensor0);
  motor1.linkSensor(&sensor1);

  driver0.voltage_power_supply=VoltagePowerSupply;
  driver0.init();
  motor0.linkDriver(&driver0);

  driver1.voltage_power_supply=VoltagePowerSupply;
  driver1.init();
  motor1.linkDriver(&driver1);

  // torque control
  motor0.torque_controller = TorqueControlType::voltage;
  motor0.controller=MotionControlType::torque;
  motor1.torque_controller = TorqueControlType::voltage;
  motor1.controller=MotionControlType::torque;

  motor0.voltage_limit=12;
  motor1.voltage_limit=12;

  // motor init
  motor0.init();
  // align encoder and start FOC
  motor0.initFOC(3.57, Direction::CCW); 
  // motor0.initFOC(); 
  
  motor1.init();
  // align encoder and start FOC
  motor1.initFOC(4.21, Direction::CCW);
  // motor1.initFOC(); 

  Serial.println(F("Motor ready."));
}

void MotorRun()
{
  // target value
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

#ifdef DEBUG
void printInfo()
{
  String str="";  
  str+="Task1:   ";
  str+="State: ";
//  switch(state)
//  {
//    case NORMAL:str+="NORMAL\t";break;
//    case FALL:  str+="FALL\t";break;
//    default: break;
//  }
  str+="Target Vel: "+String(target_velocity, 6)+"\t";  
  str+="Curr Vel: "+String(robot_velocity, 6)+"\t";
  str+="Vel Err: "+String(target_velocity-robot_velocity, 6)+"\t";
  str+="Vel Sum: "+String(vel_sum, 6)+"\t";
  str+="Target Angle: "+String(target_angle, 6)+"\t";
  str+="Curr Angle: "+String(robot_angle, 6)+"\t";
  str+="Angle Err: "+String(target_angle-robot_angle, 6)+"\t";
  str+="Force: ["+String(l_force,6)+", "+String(r_force,6)+"]\t";
  str+="BP: "+String(balance_controller.balance_PID.Kp,6)+" \tBD: "+String(balance_controller.balance_PID.Kd,6)+"\t";
  str+="VP: "+String(balance_controller.velocity_PID.Kp,6)+" \tVI: "+String(balance_controller.velocity_PID.Ki,6)+"\t";
  
  Serial.println(str);
}

void updateParam()
{
  String param_name = "", param_value_s = "";
 
  for(int i=0; i<2; i++)
  {
    param_name += char(Serial.read());
  }
  while(Serial.available())
  {
    param_value_s += char(Serial.read());   
  }

  float param_value = param_value_s.toFloat();

  if(param_name == "bp")
  {
    if(param_value_s[0]=='+'&&param_value==0)
    {
      param_value=balance_controller.balance_PID.Kp*1.1f;
    }
    else if(param_value_s[0]=='-'&&param_value==0)
    {
      param_value=balance_controller.balance_PID.Kp*0.9f;
    }
    balance_controller.updateParam(B_KP, param_value);
    Serial.println("Balance PID Kp changed to: "+ String(param_value,8));
  }
  else if(param_name == "bd")
  {
    if(param_value_s[0]=='+'&&param_value==0)
    {
      param_value=balance_controller.balance_PID.Kd*1.1f;
    }
    else if(param_value_s[0]=='-'&&param_value==0)
    {
      param_value=balance_controller.balance_PID.Kd*0.9f;
    }
    balance_controller.updateParam(B_KD, param_value);
    Serial.println("Balance PID Kd changed to: "+ String(param_value,8));
  }
  else if(param_name == "vp")
  {
    if(param_value_s[0]=='+'&&param_value==0)
    {
      param_value=balance_controller.velocity_PID.Kp*1.1f;
    }
    else if(param_value_s[0]=='-'&&param_value==0)
    {
      param_value=balance_controller.velocity_PID.Kp*0.9f;
    }
    balance_controller.updateParam(V_KP, param_value);
    Serial.println("Velocity PID Kp changed to: "+ String(param_value,8));
  }
  else if(param_name == "vi")
  {
    if(param_value_s[0]=='+'&&param_value==0)
    {
      param_value=balance_controller.velocity_PID.Ki*1.1f;
    }
    else if(param_value_s[0]=='-'&&param_value==0)
    {
      param_value=balance_controller.velocity_PID.Ki*0.9f;
    }
    balance_controller.updateParam(V_KI, param_value);
    Serial.println("Velocity PID Ki changed to: "+ String(param_value,8));
  }
  else if(param_name == "vv")
  {
    if(param_value_s[0]=='+'&&param_value==0)
    {
      param_value=balance_controller.velocity_PID.Kp*1.1f;
    }
    else if(param_value_s[0]=='-'&&param_value==0)
    {
      param_value=balance_controller.velocity_PID.Kp*0.9f;
    }
    balance_controller.updateParam(V_KP, param_value);
    balance_controller.updateParam(V_KI, param_value/450.f);
    Serial.println("Velocity PID Kp changed to: "+ String(param_value,8));
    Serial.println("Velocity PID Ki changed to: "+ String(param_value/500.f,8));
  }
  else if(param_name == "rp")
  {
    if(param_value_s[0]=='+'&&param_value==0)
    {
      param_value=balance_controller.rotate_PID.Kp*1.1f;
    }
    else if(param_value_s[0]=='-'&&param_value==0)
    {
      param_value=balance_controller.rotate_PID.Kp*0.9f;
    }
    balance_controller.updateParam(R_KP, param_value);
    Serial.println("Rotate PID Kp changed to: "+ String(param_value,8));
  }
  else if(param_name == "ri")
  {
    if(param_value_s[0]=='+'&&param_value==0)
    {
      param_value=balance_controller.rotate_PID.Ki*1.1f;
    }
    else if(param_value_s[0]=='-'&&param_value==0)
    {
      param_value=balance_controller.rotate_PID.Ki*0.9f;
    }
    balance_controller.updateParam(R_KI, param_value);
    Serial.println("Rotate PID Ki changed to: "+ String(param_value,8));
  }
  else if(param_name == "rd")
  {
    if(param_value_s[0]=='+'&&param_value==0)
    {
      param_value=balance_controller.rotate_PID.Kd*1.1f;
    }
    else if(param_value_s[0]=='-'&&param_value==0)
    {
      param_value=balance_controller.rotate_PID.Kd*0.9f;
    }
    balance_controller.updateParam(R_KD, param_value);
    Serial.println("Rotate PID Kd changed to: "+ String(param_value,8));
  }
  else if(param_name == "ve")
  {
    vel=param_value;
  }
  else if(param_name == "av")
  {
    angvel=param_value;
  }
  else if(param_name == "rr")
  {
    vel_sum=0;
    // balance_controller.rotate_PID.reset();
  }
  else if(param_name == "pr")
  {
    Serial.println("BP: "+String(balance_controller.balance_PID.Kp,6)+" \tBD: "+String(balance_controller.balance_PID.Kd,6)+"\t"
                   +"VP: "+String(balance_controller.velocity_PID.Kp,6)+" \tVI: "+String(balance_controller.velocity_PID.Ki,6)+"\t"
                   +"RP: "+String(balance_controller.rotate_PID.Kp,6)+" \tRI: "+String(balance_controller.rotate_PID.Ki,6)+" \tRD: "+String(balance_controller.rotate_PID.Kd,6)+"\t");
  }
}
#endif
