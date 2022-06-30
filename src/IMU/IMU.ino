#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <IMU.h>
#include <Filter.h>
// #define DEBUG

/* I2C cfgs */
#define I2C0_SDA    (17)
#define I2C0_SCL    (18)
#define I2C0Freq    (400000UL)
TwoWire I2Cone = TwoWire(0);

/* Serial cfgs */
#define Serial0BaudRate (230400UL)

#define Serial1_TX      (42)
#define Serial1_RX      (41)
#define Serial1BaudRate (921600UL)

#define IMUFreq (500.f)
#define Serial0Rate (100.f)
#define Serial1Rate (500.f)

/* mpu */
Adafruit_MPU6050 mpu;

/* imu vars */
float sample_freq = IMUFreq;
IMU::IMU_6DOF imu(sample_freq, MADGWICK);

unsigned long IMU_timer;
unsigned long Serial0_timer; 
unsigned long Serial1_timer;

float ax = 0.f, ay = 0.f, az = 0.f;
float gx = 0.f, gy = 0.f, gz = 0.f;

Filter::MeanFilter axFilter, ayFilter, azFilter;
Filter::FOLPF gxFilter, gyFilter, gzFilter;

void IMUInit()
{
  if(!mpu.begin(0x68, &I2Cone))
  {
    Serial.println("Failed to find MPU6050 chip");
    while(true);
    delay(10);
  }
  else  
  {
    Serial.println("MPU6050 Init Success");
    // break;
  }
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
    float dt = (micros() - IMU_timer)*1.f/1e6;
    // Serial.println(delta);
    IMU_timer = micros(); 
    
    /* imu get data */
    ax = axFilter.output(); axFilter.reset();
    ay = ayFilter.output(); ayFilter.reset();
    az = azFilter.output(); azFilter.reset();

    gx = gxFilter.output();
    gy = gyFilter.output();
    gz = gzFilter.output();

    imu.getData(ax, ay, az, gx, gy, gz);

    /* imu update */
    imu.updateState(dt);
    imu.pitch+=5.3f/180;
  }
}

void setup() {
  /* USB Serial to print data */
  Serial.begin(Serial0BaudRate);
  Serial1.begin(Serial1BaudRate, SERIAL_8N1, Serial1_TX, Serial1_RX);

  I2Cone.begin(I2C0_SDA,I2C0_SCL, I2C0Freq);   //SDA0,SCL0

  /* maximize cpu frequency */
  setCpuFrequencyMhz(240);
  Serial.println("CPU Frequency: " + String(getCpuFrequencyMhz()));

  IMUInit();
  
  /* Init Timer */
  IMU_timer = micros();
  Serial0_timer=micros();
  Serial1_timer=micros();
}

void loop() {
  IMUUpdate();

  #ifdef DEBUG
  if((micros()-Serial0_timer)>1.f*1e6/Serial0Rate&&micros()>Serial0_timer)
  {
    Serial0_timer=micros();
    Serial.println("Roll:"+String(imu.roll*RAD2ANG,8)+"Pitch:"+String(imu.pitch*RAD2ANG,8));
  }
  #endif 
  if((micros()-Serial1_timer)>1.f*1e6/Serial1Rate&&micros()>Serial1_timer)
  {
    Serial1_timer=micros();
    Serial1.println("Roll:"+String(imu.roll*RAD2ANG,8)+"Pitch:"+String(imu.pitch*RAD2ANG,8));
  }
}
