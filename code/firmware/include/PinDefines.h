// All ESP32 GPIOs can be used as inputs, except GPIOs 6 to 11 (connected to the integrated SPI flash).
// All GPIOs can be used as outputs except GPIOs 6 to 11 (connected to the integrated SPI flash) and GPIOs 34, 35, 36 and 39 (input only GPIOs);

#ifndef PIN_DEFINES_H_
#define PIN_DEFINES_H_

#define PIN_WHEEL_SERVO 5
#define PIN_CAMERA_SERVO 4
#define PIN_ALARM_LED 17
#define PIN_LAMP_LED 16
#define PIN_BCK 2
#define PIN_WS 15
#define PIN_DATA_IN 13
#define PIN_ULTRASONIC_ECHO 34
#define PIN_ULTRASONIC_TRIG 18

#endif
