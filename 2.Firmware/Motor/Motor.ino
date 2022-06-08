#include <SimpleFOC.h>

/* global vars */
float l_force = 0.f;
float r_force = 0.f;

/* I2C cfgs */
TwoWire I2C0 = TwoWire(0);
TwoWire I2C1 = TwoWire(1);

#define ADDR (0x02)

#define I2C0_SDA (1)
#define I2C0_SCL (2)
#define I2C1_SDA (3)
#define I2C2_SCL (4)

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

/* main function */
void setup()
{
  /* USB Serial to print data */
  Serial.begin(115200);

  I2C0.begin(I2C0_SDA, I2C0_SCL, 400000UL);
  I2C1.begin(I2C1_SDA, I2C1_SCL, 400000UL);

  /* maximize cpu frequency */
  setCpuFrequencyMhz(240);
  Serial.println("CPU Frequency: " + String(getCpuFrequencyMhz()));

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

void task1(void *pvParameters){}

void MotorInit(){}

void MotorRun(){}

void RcvForce(){}
