#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <IMU.h>
#include <Filter.h>
#include <BalanceController.h>

#include "Status.h"
#include "Output.h"

float velocity = 0.f;             //m/s
float angular_velocity = 0.f;     //rad/s

/* I2C cfgs */
TwoWire I2C0 = TwoWire(0);
TwoWire I2C1 = TwoWire(1);

#define ADDR (0x02)

#define I2C0_SDA (1)
#define I2C0_SCL (2)
#define I2C1_SDA (3)
#define I2C2_SCL (4)

/* mpu6050 */
Adafruit_MPU6050 mpu;

/* imu vars */
float sample_freq = 1000.f;
IMU::IMU_6DOF imu(sample_freq, IMU::MAHONY);
unsigned long IMU_timer;

float ax = 0.f, ay = 0.f, az = 0.f;
float gx = 0.f, gy = 0.f, gz = 0.f;

Filter::MeanFilter axFilter, ayFilter, azFilter;
Filter::FOLPF gxFilter, gyFilter, gzFilter;

/* controller vars */
BalanceController::BalanceController balance_controller;

/* function prototype declaration */
void IMUInit(TwoWire& Wire);
void IMUUpdate();

/* FreeRTOS */
#define CORE_0 (0)
#define CORE_1 (1)

TaskHandle_t Task0;
#define TASK_STACK0 (4*1024)
#define TASK_PRIO0 (9)

TaskHandle_t Task1;
#define TASK_STACK1 (4*1024)
#define TASK_PRIO1 (10)

TaskHandle_t Task2;
#define TASK_STACK2 (2*1024)
#define TASK_PRIO2 (5)

TaskHandle_t Task3;
#define TASK_STACK3 (2*1024)
#define TASK_PRIO3 (1)

void task0(void *pvParameters);
void task1(void *pvParameters);
void task2(void *pvParameters);
void task3(void *pvParameters);

void setup()
{ 
  /* USB Serial to print data */
  Serial.begin(115200);

  I2C0.begin(I2C0_SDA, I2C0_SCL, 400000UL);
  I2C1.begin(I2C1_SDA, I2C1_SCL, 400000UL);

  /* maximize cpu frequency */
  setCpuFrequencyMhz(240);
  Serial.println("CPU Frequency: "+String(getCpuFrequencyMhz()));

  /* device init */
  IMUInit(I2C0);

  /* create tasks */
  xTaskCreatePinnedToCore(task0, "Task_IMU", TASK_STACK0, NULL, TASK_PRIO0, &Task_0, CORE_0);
  xTaskCreatePinnedToCore(task1, "Task_FSM", TASK_STACK1, NULL, TASK_PRIO1, &Task_1, CORE_1);
  // xTaskCreatePinnedToCore(task2, "Task_Msg", TASK_STACK2, NULL, TASK_PRIO2, &Task_2, CORE_0);
  // xTaskCreatePinnedToCore(task3, "Task_OLED", TASK_STACK3, NULL, TASK_PRIO3, &Task_3, CORE_1);
}

void loop(){}

void task0(void *pvParameters)
{
  IMU_timer = micros();

  while(true)
  {
    IMUUpdate();
  }
}

void task1(void *pvParameters)
{
  float force = 0.f;
  float l_force = 0.f, r_force = 0.f;
  float target_velocity = 0.f;
  float target_angle = 0.f;
  float robot_velocity = 0.f;
  float robot_angle = 0.f;
  uint8_t status = Status::FALL;

  while(true)
  {
    robot_angle = imu.pitch;
    target_velocity = velocity;
    
    if(Serial.available())
    {
      updateParam();
    }
    
    if(robot_angle<40&&robot_angle>-40)
    {
      status=Status::NORMAL;
    }
    else 
    {
      status=Status::FALL;
    }

    switch(status)
    {
      case Status::NORMAL:
      {    
        target_angle = balance_controller.velocityPID(target_velocity, robot_velocity);
        force = balance_controller.balancePID(target_angle, robot_angle);
        break;
      }
      case Status::FALL:
      {
        balance_controller.reset();
        force = 0.f;
        break;
      }
      default: break;
    }

    //TODO: send force
  }
}

void task2(void *pvParameters)
{
  //TODO: receive command from remote controller
}

void task3(void *pvParameters)
{
  //TODO: light oled screen
}

void IMUInit(TwoWire& Wire)
{
  if(!mpu.begin(0x68, &Wire))
  {
    Serial.println("Failed to find MPU6050 chip");
    while(true);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  IMU_timer = micros();
}

void IMUUpdate()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  axFilter.input(a.acceleration.x);
  ayFilter.input(a.acceleration.y);
  azFilter.input(a.acceleration.z);

  gxFilter.input(g.gyro.x);
  gyFilter.input(g.gyro.y);
  gzFilter.input(g.gyro.z);

  if ((micros() - IMU_timer >= (1e6 * 1.f) / sample_freq) || micros() < IMU_timer)
  {
    // delta = micros() - IMU_timer ;
    IMU_timer = micros(); /* imu get data */

    ax = axFilter.output(); axFilter.reset();
    ay = ayFilter.output(); ayFilter.reset();
    az = azFilter.output(); azFilter.reset();

    gx = gxFilter.output();
    gy = gyFilter.output();
    gz = gzFilter.output();

    imu.getData(ax, ay, az, gx, gy, gz);
    imu.updateState();
  }
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

  if(param_name == "vp")
  {
    balance_controller.updateParam(BalanceController::V_KP, param_value);
  }
  else if(param_name == "vd")
  {
    balance_controller.updateParam(BalanceController::V_KD, param_value);
  }
  else if(param_name == "pp")
  {
    balance_controller.updateParam(BalanceController::P_KP, param_value);
  }
  else if(param_name == "pi")
  {
    balance_controller.updateParam(BalanceController::P_KI, param_value);
  }
}
