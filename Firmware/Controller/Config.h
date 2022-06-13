#ifndef CONFIG_H
#define CONFIG_H

/* Robot State */
#define NORMAL (0)
#define FALL (1)

/* Serial cfgs */
#define SERIAL0_TX (44)
#define SERIAL0_RX (43)
#define SERIAL1_TX (42)
#define SERIAL1_RX (41)

/* SPI cfgs */
#define HSPI_MISO (39)
#define HSPI_MOSI (17)
#define HSPI_SCLK (18)
#define HSPI_CS0  (2)   // accel chip
#define HSPI_CS1  (38)  // gyro chip

/* OLED cfgs */
#define SCREEN_WIDTH  (128) // OLED display width, in pixels
#define SCREEN_HEIGHT (64) // OLED display height, in pixels
// Declaration for SSD1306 display connected using software SPI (default case):
#define OLED_MOSI     (21)
#define OLED_CLK      (20)
#define OLED_DC       (33)
#define OLED_CS       (34)
#define OLED_RESET    (48)

/* RC Circle */
#define RC_X_COOR       (16)
#define RC_Y_COOR       (48)
#define RC_RADIUS       (14.f)

/* RC cfgs */
#define RC_X_RANGE      (1.f)
#define RC_Y_RANGE      (1.f)

/* Task Frequency */
#define IMU_FREQ        (1000.f)
#define CONTROLLER_FREQ (1000.f)
#define OLED_FPS        (5.f)

#endif
