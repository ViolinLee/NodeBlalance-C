#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
// #include <HTTPClient.h>
#include <Wire.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>
#include "driver/i2s.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
// for balancing
#include <MPU6050.h>
#include <Control.h>
#include <Motors.h>
#include <HCSR04.h>
#include <iostream>
#include <sstream>
#include "PinDefines.h"

// Definitions
#define RAD2GRAD 57.2957795             // aux definitions
#define GRAD2RAD 0.01745329251994329576923690768489
#define SERVO_AUX_NEUTRO 307
#define SERVO_AUX_MIN 192 // 102
#define SERVO_AUX_MAX 422 // 512
#define SERVO_CAM_AUX_NEUTRO 307
#define SERVO_CAM_AUX_MIN 122 // 102
#define SERVO_CAM_AUX_MAX 328 // 348
#define MTU 1440
#define MAX_TARGET_ANGLE 14
#define MAX_CONTROL_OUTPUT  300 //500

// Constant Declarations
const int SAMPLE_SIZE = 16384;
const int SAMPLE_RATE = 16000;
const byte triggerPin = 13;
const byte echoPin = 12;
const char* ssid     = "***";
const char* password = "***";
const int udpPort = 3333;
const int queueSize = SAMPLE_RATE * 2;
const int PWM_WHEEL_FREQ = 50;          // PWM Pulse_Width calculation: 
const int PWM_CAMERA_FREQ = 50;         // (500~2500us / 20000us)*4096
const int PWM_WHEEL_RESOLUTION = 12;    // (degree*(2500-500)/(180-0)+500)/20000*4096
const int PWM_CAMERA_RESOLUTION = 12;   // (degree*11+500)/20000*4096 <=> duty_cycle * 4096
const int PWM_WHEEL_CHANNEL = 0;
const int PWM_CAMERA_CHANNEL = 3;
const TickType_t xDelay50ms = pdMS_TO_TICKS( 50 );
const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
const i2s_port_t I2S_PORT = I2S_NUM_0;

// Struct Declarations
i2s_config_t i2sMemsConfigLeftChannel = {                // i2s config for reading from left channel of I2S
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 32,
    .dma_buf_len = 64,
    .use_apll = true,};

i2s_pin_config_t i2sPins = {                             // i2s pins
    .bck_io_num = PIN_BCK,
    .ws_io_num = PIN_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = PIN_DATA_IN};


// Variable Declarations
int8_t loop_cnt = 0;
int16_t sample16 = 0;
int32_t sample = 0;
int16_t element = 0;
int sample_counter = 0;

unsigned long currentTime;            // frame timer variables
unsigned long previousTime;
float dt;

int newCommand = 0;
int CarState = 1;                 // 0-differential 1-balance
int CarCmdMode = 1;               // 0-gui 1-voice 2-vision
int CarSpeed;
int carCmdMovement=4;
float speed=10;
float servoAngle=50;  // 900~1900
float throttle;
float steering;
float steering2throttle_factor = 3;
int16_t actual_robot_speed;

float angle_adjusted_Old;
float angle_adjusted;
float angle_adjusted_filtered;
float angle_offset;
float estimated_speed_filtered;
boolean positionControlMode = false;
int16_t motor1_control;
int16_t motor2_control;
int32_t target_steps1;
int32_t target_steps2;
float Kp_position;
float Kd_position;
float target_angle;
float Kp = 0.35;
float Kd = 0.05;
float Kp_thr = 0.080;
float Ki_thr = 0.1;
float control_output;
int16_t motor1;
int16_t motor2;

char * udpAddress = "192.168.0.113";

// Object Declarations
WiFiUDP udp;
AsyncWebServer server(80);
AsyncWebSocket wsCarCommand("/CarInput");
TaskHandle_t SampleAudioTaskHandle;
TaskHandle_t SendAudioTaskHandle;
TaskHandle_t DetectDistanceTaskHandle;
QueueHandle_t queueSample;
UltraSonicDistanceSensor distanceSensor(PIN_ULTRASONIC_TRIG, PIN_ULTRASONIC_ECHO);

// Function Declaration
void initMPU6050();
void setupI2SMic();
void processCommand();
void sampleAudioTask( void * parameter );
void sendAudioTask( void * parameter );
void detectDistanceTask( void * parameter );
void moveCar(float throttleVal, float steeringVal);
void rotateWheel(int command);
void setLandingGear(float position, int gear_speed);
void rotateCamera(int command);
void onCarCommandWebSocketEvent(AsyncWebSocket *server,
                                AsyncWebSocketClient *client,
                                AwsEventType type,
                                void *arg,
                                uint8_t *data,
                                size_t len);

// Initialization Routine
void setup() {
    // Serial COMM
    Serial.begin(115200);         // set up serial monitor at 115200 bps
    Serial.setDebugOutput(true);
    Serial.println("*Serial Initialization*");
    Serial.println("--------------------------------------------------------");

    // Stepper Motor
    pinMode(PIN_ENABLE_MOTORS, OUTPUT);
    digitalWrite(PIN_ENABLE_MOTORS, HIGH);
    pinMode(PIN_MOTOR_LEFT_DIR, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_STEP, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_DIR, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_STEP, OUTPUT);
    pinMode(PIN_MOTOR_GEAR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_GEAR_STEP, OUTPUT);
    pinMode(PIN_WHEEL_SERVO, OUTPUT);
    pinMode(PIN_CAMERA_SERVO, OUTPUT);

    // Lamp & LED
    // pinMode(PIN_ALARM_LED, OUTPUT);
    // digitalWrite(PIN_ALARM_LED, LOW);
    pinMode(PIN_LAMP_LED, OUTPUT);
    digitalWrite(PIN_LAMP_LED, LOW);

    // HC-SR04 Ultrasonic Sensor
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);
    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);

    // Servo Motor
    ledcSetup(PWM_WHEEL_CHANNEL, PWM_WHEEL_FREQ, PWM_WHEEL_RESOLUTION);    // mg90s: 500~2500 of 0~180° -> 102~512 pulse_width
    ledcAttachPin(PIN_WHEEL_SERVO, PWM_WHEEL_CHANNEL);
    ledcWrite(PWM_WHEEL_CHANNEL, SERVO_AUX_NEUTRO);

    ledcSetup(PWM_CAMERA_CHANNEL, PWM_CAMERA_FREQ, PWM_CAMERA_RESOLUTION);    // adrc1.5g: 900~2100us of 0~9mm -> 102~348 pulse_width
    ledcAttachPin(PIN_CAMERA_SERVO, PWM_CAMERA_CHANNEL);
    ledcWrite(PWM_CAMERA_CHANNEL, SERVO_AUX_NEUTRO);

    // Gyro & Accelerator
    Wire.begin();
    initMPU6050();

    // WIFI (STA)
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the WiFi network");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    queueSample = xQueueCreate( queueSize, sizeof( int16_t ) );
    if(queueSample == NULL){
        Serial.println("Error creating the queue");
    }

    // HTTP Server and WebSockets
    // server.on("/", HTTP_GET, handleRoot);
    // server.onNotFound(handleNotFound);
    wsCarCommand.onEvent(onCarCommandWebSocketEvent);
    server.addHandler(&wsCarCommand);
    server.begin();
    Serial.println("HTTP server started");

    /*
    HTTP Client (for audio data transmission)
    wifiClientI2S = new WiFiClient();
    httpClientI2S = new HTTPClient();
    Direct i2s input from INMP441 or the SPH0645
    i2sSampler = new I2SMEMSSampler(I2S_NUM_0, i2sPins, i2sMemsConfigLeftChannel, false);
    i2sSampler->start();
    xTaskCreatePinnedToCore(i2sMemsWriterTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsWriterTaskHandle, 1);
    */

    initTimers();

    setupI2SMic();
    xTaskCreatePinnedToCore(sampleAudioTask, "SampleAudioTask",  10000,  NULL,  1, &SampleAudioTaskHandle, 1);
    xTaskCreatePinnedToCore(sendAudioTask, "SendAudioTask",  10000,  NULL,  1, &SendAudioTaskHandle, 1);
    xTaskCreatePinnedToCore(detectDistanceTask, "DetectDistanceTask",  4000,  NULL,  1, &DetectDistanceTaskHandle, 1);

    //xTaskCreate(sampleAudioTask, "SampleAudio", 10000, NULL, 1, NULL);

    Serial.println("Finish Setuped.");
}

// Main Program
void loop() {
  
  //Serial.println("In loop");
  if (newCommand) {
      newCommand = 0;
      processCommand();
  }
  
  // Serial.println(throttle);

  //Serial.println(distanceSensor.lastest_distanceCm);
  if (distanceSensor.lastest_distanceCm < 8) {
    throttle = -50;
    // 亮红灯
    // Serial.println("Red LED Light!");
  } else {
    if (throttle == -50) {
      throttle = 0;
    }
  }
  //Serial.println("After loop");
  //Serial.println("************");
  Serial.println(throttle);
  //Serial.println(steering);
  //Serial.println("************");

  // uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
  if (CarState == 0)
  {
    // Tricycle mode
    moveCar(throttle, steering);
  } else {
    // Balancing mode
    currentTime = micros();
    if (MPU6050_newData()) {
      MPU6050_read_3axis();

      dt = (currentTime - previousTime) * 0.000001; // dt in seconds
      //Serial.println(dt);
      previousTime = currentTime;

      angle_adjusted_Old = angle_adjusted;
      // Get new orientation angle from IMU (MPU6050)
      float MPU_sensor_angle = MPU6050_getAngle(dt);
      angle_adjusted = MPU_sensor_angle + angle_offset;
      // Serial.println(angle_adjusted);

      if ((MPU_sensor_angle > -15) && (MPU_sensor_angle < 15))
        angle_adjusted_filtered = angle_adjusted_filtered * 0.99 + MPU_sensor_angle * 0.01;

      actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward

      int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units
      int16_t estimated_speed = -actual_robot_speed + angular_velocity;
      estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float) estimated_speed * 0.1; // low pass filter on estimated speed

      /*
      if (positionControlMode) {
      // POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
      motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
      motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

      // Convert from motor position control to throttle / steering commands
      throttle = (motor1_control + motor2_control) / 2;
      throttle = constrain(throttle, -190, 190);
      steering = motor2_control - motor1_control;
      steering = constrain(steering, -50, 50);
      }
      */

      // ROBOT SPEED CONTROL: This is a PI controller.
      //    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
      if (loop_cnt == 2) {
        target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
        target_angle = constrain(target_angle, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE); // limited output

        loop_cnt = 0;
      }
      // Serial.println(target_angle);

      // Stability control (100Hz loop): This is a PD controller.
      //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
      //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
      control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
      control_output = constrain(control_output, -MAX_CONTROL_OUTPUT,  MAX_CONTROL_OUTPUT); // Limit max output from control

      // The steering part from the user is injected directly to the output
      motor1 = control_output + steering;
      motor2 = control_output - steering;

      // Limit max speed (control output)
      motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
      motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
      // Serial.println("______________________________________________");

      int angle_ready = 80;
      if ((angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready)) // Is robot ready (upright?)
          {
        // NORMAL MODE
        digitalWrite(PIN_ENABLE_MOTORS, LOW);  // Motors enable
        // NOW we send the commands to the motors
        setMotorSpeedM1(motor1);
        setMotorSpeedM2(motor2);
      } else   // Robot not ready (flat), angle > angle_ready => ROBOT OFF
      {
        digitalWrite(PIN_ENABLE_MOTORS, HIGH);  // Disable motors
        setMotorSpeedM1(0);
        setMotorSpeedM2(0);
        Serial.println("Robot not ready");
      }

      loop_cnt += 1;
    }
  }
  
  /*
  digitalWrite(PIN_ENABLE_MOTORS, LOW); 
   
  setMotorSpeedM1(10);
  setMotorSpeedM2(10);
  setMotorSpeedM3(10);
  delay(200);

  float MPU_sensor_angle = MPU6050_getAngle(dt);
  //Serial.println(MPU_sensor_angle);

  digitalWrite(PIN_LAMP_LED, HIGH);
  delay(800);
  digitalWrite(PIN_LAMP_LED, LOW);

  rotateWheel(2);
  delay(200);
  rotateWheel(0);

  rotateCamera(200);
  delay(200);
  */
}


void initMPU6050() {
  MPU6050_setup();
  delay(500);
  MPU6050_calibrate();
}

void processCommand() {
  // Only used as command parser
  Serial.println("In processcmd");
  switch (carCmdMovement) {
    case 0:
      throttle = speed * steering2throttle_factor;
      steering = 0;
      break;
    case 1:
      throttle = -speed * steering2throttle_factor;
      steering = 0;
      break;
    case 2:
      throttle = 0;
      steering = -speed;
      break;
    case 3:
      throttle = 0;
      steering = speed;
      break;
    case 4:
      throttle = 0;
      steering = 0;
      break;
  }
  newCommand = 0;
}

void onCarCommandWebSocketEvent(AsyncWebSocket *server,
                                AsyncWebSocketClient *client,
                                AwsEventType type,
                                void *arg,
                                uint8_t *data,
                                size_t len)
{
  switch (type)
  {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      AwsFrameInfo *info;
      info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
      {
        std::string myData = "";
        myData.assign((char *)data, len);
        std::istringstream ss(myData);
        std::string key, value;
        std::getline(ss, key, ',');
        std::getline(ss, value, ',');
        Serial.printf("Key [%s] Value[%s]\n", key.c_str(), value.c_str());
        // int valueInt = atoi(value.c_str());

        if (key == "GuiIP")
        {
          strcpy (udpAddress, value.c_str());
          Serial.println("GuiIP");
        }
        if (key == "MoveCar")
        {
          carCmdMovement = atoi(value.c_str());
          rotateWheel(carCmdMovement);
          Serial.println("MoveCar");
        }
        else if (key == "Lamp")
        {
          digitalWrite(PIN_LAMP_LED, atoi(value.c_str()));
          Serial.println("Lamp");
        }
        else if (key == "Mode")
        {
          CarCmdMode = atoi(value.c_str());
          Serial.println("Mode");
        }
        else if (key == "States")
        {
          if (CarState == 1) {  // pid -> diff
            setLandingGear(0.95, 10);  // down
            CarState = 0;
            // xTaskNotify(SampleAudioTaskHandle, 1, eSetBits);
          }
          else if (CarState == 0) {  // diff -> pid
            setLandingGear(0.8, 50);  // up
            CarState = 1 - CarState;
            // xTaskNotify(SampleAudioTaskHandle, 1, eSetBits);
            setLandingGear(0, 50);
          }
        }
        else if (key == "SetSpeed")
        {
          speed = atoi(value.c_str());
          Serial.println("SetSpeed");
        }
        else if (key == "SetAngle")
        {
          servoAngle = atoi(value.c_str());
          Serial.println("SetAngle");
          Serial.println(servoAngle);
          int16_t camAngle = map(servoAngle, 0, 100, SERVO_CAM_AUX_MIN, SERVO_CAM_AUX_MAX);
          rotateCamera(camAngle);
        }

        newCommand = 1;
      }
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
    default:
      break;
  }
}


void moveCar(float throttleVal, float steeringVal) {
  //Serial.println("in move car");
}

void rotateWheel(int command) {
  //Serial.println("in rotate wheel");
  switch (command) {
    case 2: ledcWrite(PWM_WHEEL_CHANNEL, SERVO_AUX_MIN); break;
    case 3: ledcWrite(PWM_WHEEL_CHANNEL, SERVO_AUX_MAX); break;
    default: ledcWrite(PWM_WHEEL_CHANNEL, SERVO_AUX_NEUTRO); break;;
  }
}

void rotateCamera(int command) {
  //Serial.println("in rotate Camera");
  ledcWrite(PWM_CAMERA_CHANNEL, command);
}

void setLandingGear(float position, int gear_speed) {
  //Serial.println("in Landing gear");
}

void sampleAudioTask( void * parameter ) {     
  while ( true ){
    if (CarCmdMode == 1) {
      i2s_pop_sample(I2S_PORT, (char *)&sample, 10);
      sample>>=14;
      sample16 = (int16_t)sample;
      xQueueSend(queueSample, &sample16, 10);
    }
  }
  vTaskDelete(SampleAudioTaskHandle);
}

void sendAudioTask( void * parameter ) {
  while (true){
    if (CarCmdMode == 1) {
      udp.beginPacket(udpAddress, udpPort);
      for(sample_counter=0; sample_counter<MTU/2; sample_counter++){
        xQueueReceive(queueSample, &element, portMAX_DELAY);
        udp.write((byte*)&element,2);
      }
      udp.endPacket();
    }
  }
  vTaskDelete( SendAudioTaskHandle );
}

void detectDistanceTask( void * parameter ) {
   while (true) {
      distanceSensor.measureDistanceCm();
      vTaskDelay( xDelay50ms );
   }
}


void  setupI2SMic() {
  Serial.println("Configuring I2S...");
  // The I2S config as per the example
  esp_err_t err;
  err = i2s_driver_install(I2S_PORT, &i2sMemsConfigLeftChannel, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2S_PORT, &i2sPins);
  if (err != ESP_OK) {
    Serial.printf("Failed setting pin: %d\n", err);
    while (true);
  }  
  Serial.println("I2S driver installed.");
};

/*
// send data to a remote address
void sendData(WiFiClient *wifiClient, HTTPClient *httpClient, const char *url, uint8_t *bytes, size_t count)
{
  // send them off to the server
  digitalWrite(2, HIGH);
  httpClient->begin(*wifiClient, url);
  httpClient->addHeader("content-type", "application/octet-stream");
  httpClient->POST(bytes, count);
  httpClient->end();
  digitalWrite(2, LOW);
}


// Task to write samples to our server
void i2sMemsWriterTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *) param;
  int16_t *samples = (int16_t *)malloc(sizeof(uint16_t) * SAMPLE_SIZE);
  if (!samples)
  {
    Serial.println("Failed to allocate memory for samples");
    return;
  }
  while (true)
  {
    if (CarCmdMode == 1) {
        if (i2s_server_url[0]) {
            int samples_read = sampler->read(samples, SAMPLE_SIZE);
            sendData(wifiClientI2S, httpClientI2S, i2s_server_url, (uint8_t *)samples, samples_read * sizeof(uint16_t));
        } else {
            Serial.println("I2S Server URL is not configured correctly");
        }
    }
  }
}
*/
