#include <SPI.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <BMI085.h>   
#include <IMU.h>
#include <Filter.h>
#include <BalanceController.h>

#include "Status.h"
#include "Output.h"
#include "Config.h"
#include "Bitmap.h"

float rc_x=0.f;
float rc_y=0.f;

float velocity = 0.f;           // m/s
float angular_velocity = 0.f;   // rad/s

/* SPI cfgs */
SPIClass hspi(HSPI);

/* accel object */
BMI085Accel accel(hspi, HSPI_CS0);
/* gyro object */
BMI085Gyro gyro(hspi, HSPI_CS1);

/* imu vars */
float sample_freq = 1000.f;
IMU::IMU_6DOF imu(sample_freq, MADGWICK);
unsigned long IMU_timer;
unsigned long delta;

float ax = 0.f, ay = 0.f, az = 0.f;
float gx = 0.f, gy = 0.f, gz = 0.f;

Filter::MeanFilter axFilter, ayFilter, azFilter;
Filter::FOLPF gxFilter, gyFilter, gzFilter;

/* controller vars */
BalanceController::BalanceController balance_controller;

/* OLED vars */
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

/* function prototype declaration */
void OLEDInit();
void IMUInit();
void IMUUpdate();
void getWheelVel(float &l_velocity, float &r_velocity);
void updateParam();
void angleVisual(const float& angle, const int &x, const int &y, const float &radius=14.f);

/* FreeRTOS */
#define CORE_0 (0)
#define CORE_1 (1)

TaskHandle_t Task0;
#define TASK_STACK0 (6*1024)
#define TASK_PRIO0 (4)

TaskHandle_t Task1;
#define TASK_STACK1 (6*1024)
#define TASK_PRIO1 (10)

TaskHandle_t Task2;
#define TASK_STACK2 (4*1024)
#define TASK_PRIO2 (5)

TaskHandle_t Task3;
#define TASK_STACK3 (4*1024)
#define TASK_PRIO3 (5)

void task0(void *pvParameters);
void task1(void *pvParameters);
void task2(void *pvParameters);
void task3(void *pvParameters);

void setup()
{ 
  /* USB Serial to print data */
  Serial.begin(2000000UL, SERIAL_8N1, SERIAL0_TX, SERIAL0_RX);
  Serial1.begin(2000000UL, SERIAL_8N1, SERIAL1_TX, SERIAL1_RX);

  /* SPI init */
  hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS0); //SCLK, MISO, MOSI, SS
  hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS1); //SCLK, MISO, MOSI, SS

  /* maximize cpu frequency */
  setCpuFrequencyMhz(240);
  Serial.println("CPU Frequency: " + String(getCpuFrequencyMhz()));

  /* device init */
  OLEDInit();
  IMUInit();

  delay(1000);

  /* create tasks */
  xTaskCreatePinnedToCore(task0, "Task_IMU", TASK_STACK0, NULL, TASK_PRIO0, &Task0, CORE_0);
  xTaskCreatePinnedToCore(task1, "Task_FSM", TASK_STACK1, NULL, TASK_PRIO1, &Task1, CORE_1);
  // xTaskCreatePinnedToCore(task2, "Task_Msg", TASK_STACK2, NULL, TASK_PRIO2, &Task2, CORE_1);
  xTaskCreatePinnedToCore(task3, "Task_OLED", TASK_STACK3, NULL, TASK_PRIO3, &Task3, CORE_0);
}

void loop(){}

void task0(void *pvParameters)
{
  IMU_timer = micros();
  int cnt=0;
  unsigned long OLED_timer=micros();
  while(true)
  {
    IMUUpdate();
    // printRPY(imu);
  }
}

void task1(void *pvParameters)
{
  float force = 0.f;
  float l_force = 0.f, r_force = 0.f;
  float target_velocity = velocity;                   //m/s
  float target_angle = 0.f;                           //rad
  float target_angular_velocity = angular_velocity;   //rad/s
  float robot_velocity = 0.f;
  float robot_angle = 0.f;
  float robot_angular_velocity = 0.f;
  float l_velocity = 0.f, r_velocity = 0.f;           //rad/s

  float len = 1.f;

  String l_force_s="", r_force_s="";

  uint8_t status = FALL;

  unsigned long FSM_timer=micros();
  int FSM_freq=1000;

  while(true)
  {
    if (Serial.available())
    {
      updateParam();
    }

    getWheelVel(l_velocity, r_velocity);

    if((micros()-FSM_timer)<1e6*1.f/FSM_freq&&(micros()>FSM_timer))
    {
      continue;
    }
    FSM_timer=micros();

    robot_velocity = (l_velocity + r_velocity)/2;
    robot_angle = imu.pitch;
    robot_angular_velocity = (l_velocity - r_velocity)/(2*len);

    target_velocity = velocity;
    target_angular_velocity = angular_velocity;
    
    if(robot_angle<40&&robot_angle>-40)
    {
      status=NORMAL;
    }
    else 
    {
      status=FALL;
    }

    switch(status)
    {
      case NORMAL:
      {    
        target_angle = balance_controller.velocityPID(target_velocity, robot_velocity);
        force = balance_controller.balancePID(target_angle, robot_angle);
        l_force=force;
        r_force=force;
        break;
      }
      case FALL:
      {
        balance_controller.reset();
        force = 0.f;
        l_force=force;
        r_force=force;
        break;
      }
      default: break;
    }

    l_force_s=String(l_force, 6);
    r_force_s=String(r_force, 6);

    //TODO: send force
    String s=l_force_s+','+r_force_s;
    Serial1.println(s);
    // vTaskDelay(1);
  }
}

void task2(void *pvParameters)
{
  //TODO: receive command from remote controller
}

void task3(void *pvParameters)
{
  float fps=10;
  unsigned long OLED_timer=micros();
  display.clearDisplay();
  while(true)
  {
    if(micros()-OLED_timer>1e6*1.f/fps)
    {      
      String str="FPS: "+String(1e6*1.f/(micros()-OLED_timer),2)+"\n";
      OLED_timer=micros();
      
      display.clearDisplay();
      
      display.drawRect(RC_X_COOR-RC_RADIUS, RC_Y_COOR-RC_RADIUS, 2*RC_RADIUS, 2*RC_RADIUS, SSD1306_WHITE);
      display.drawLine(RC_X_COOR-RC_RADIUS, RC_Y_COOR, RC_X_COOR+RC_RADIUS-1, RC_Y_COOR, SSD1306_WHITE); 
      display.drawLine(RC_X_COOR, RC_Y_COOR-RC_RADIUS, RC_X_COOR, RC_Y_COOR+RC_RADIUS-1, SSD1306_WHITE); 

      float x=RC_X_COOR+(rc_x/RC_X_RANGE)*RC_RADIUS;
      float y=RC_Y_COOR+(rc_y/RC_Y_RANGE)*RC_RADIUS;
      display.fillCircle(x, y, 2, SSD1306_WHITE);
      //display.drawLine(RC_X_COOR, RC_Y_COOR, x, y, SSD1306_WHITE); 

      angleVisual(imu.roll, 48, 48);
      angleVisual(imu.pitch, 80, 48);
      angleVisual(imu.yaw, 112, 48);

      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      str+=getRPYStr(imu,"\n");
      display.println(str);
      
      display.display();
    }
    vTaskDelay(1000.f/fps);
  }
}

void angleVisual(const float& angle, const int &x, const int &y, const float &radius)
{
  display.drawCircle(x, y, radius, SSD1306_WHITE);
  display.drawLine(x-radius, y, x+radius, y, SSD1306_WHITE); 
  display.drawLine(x, y-radius, x, y+radius, SSD1306_WHITE); 

  float center_x=x+radius*cos(angle);
  float center_y=y-radius*sin(angle);
  display.fillCircle(center_x, center_y, 2, SSD1306_WHITE);
  //display.drawLine(center_x, center_y, x, y, SSD1306_WHITE); 
}

void getWheelVel(float &l_velocity, float &r_velocity)
{
  if(Serial1.available())
  {
    String s="";
    while(Serial1.available())
    {
      s+=char(Serial1.read());
    }
    
    int cnt=0;
    String l_vel_s="";
    String r_vel_s="";

    while(s[cnt]!=',')
    {
      l_vel_s+=s[cnt++];
    }
    cnt++;
    while(cnt<s.length())
    {
      r_vel_s+=s[cnt++];
    }

    l_velocity=l_vel_s.toFloat();
    r_velocity=r_vel_s.toFloat();
  }
}

void OLEDInit()
{
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }  
  Serial.println(F("SSD1306 allocation succeed")); 
  
  display.clearDisplay(); // 清空屏幕
  display.drawBitmap(27, 0, NERV, 128, 64, WHITE);
  display.display();
}

void IMUInit()
{
  int status;

  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error"); Serial.println(status);
    while (1) {}
  }
  else {
    Serial.println("Accel Initialization Succeed");
  }

  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error"); Serial.println(status);
    while (1) {}
  }
  else {
    Serial.println("Gyro Initialization Succeed");
  }
}

void IMUUpdate()
{
  /* read the accel */
  accel.readSensor();
  /* read the gyro */
  gyro.readSensor();

  axFilter.input(accel.getAccelX_mss());
  ayFilter.input(accel.getAccelY_mss());
  azFilter.input(accel.getAccelZ_mss());

  gxFilter.input(gyro.getGyroX_rads());
  gyFilter.input(gyro.getGyroY_rads());
  gzFilter.input(gyro.getGyroZ_rads());

  if ((micros() - IMU_timer >= (1e6 * 1.f) / sample_freq) || micros() < IMU_timer)
  {
    delta = micros() - IMU_timer ;
    Serial.println(delta);
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
    balance_controller.updateParam(B_KP, param_value);
  }
  else if(param_name == "vd")
  {
    balance_controller.updateParam(B_KD, param_value);
  }
  else if(param_name == "pp")
  {
    balance_controller.updateParam(V_KP, param_value);
  }
  else if(param_name == "pi")
  {
    balance_controller.updateParam(V_KI, param_value);
  }
}
