#ifndef CONFIG_H
#define CONFIG_H

/* Macro Defs */
// #define DEBUG
#define VoltageControl
#define ParallelPID

/* Robot State */
#define NORMAL (0)
#define FALL (1)

#define FALL_ANG (60.f)

#define MAX_SUM  (1.f*1e4)
#define MIN_SUM  (-1.f*1e4)

/* Serial cfgs */
#define Serial0BaudRate (1000000UL)

#define Serial1_TX      (17)
#define Serial1_RX      (16)
#define Serial1BaudRate (921600UL)

/* I2C cfgs */
#define I2C0_SDA    (19)
#define I2C0_SCL    (18)
#define I2C0Freq    (400000UL)

#define I2C1_SDA    (23)
#define I2C1_SCL    (5)
#define I2C1Freq    (400000UL)

/* motor cfgs */
#define VoltagePowerSupply (12)

/* Rates */
#define FSMFreq (500.f)

#endif
