#include <Arduino.h>
#include <AccelStepper.h>
#include <micro_ros_platformio.h>
#include <EEPROM.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/image.h>
#include <esp_camera.h>
#include <WiFiUdp.h>

// Stepper Motor and LED Pins
#define DIR_PIN 14
#define STEP_PIN 15
#define MOTOR_INTERFACE_TYPE 1
#define LED_PIN 4

// Motion Sensor & Button Pins
#define MOTION_SENSOR_PIN 13
#define BUTTON_PIN 12

// EEPROM
#define EEPROM_SIZE 10
#define EEPROM_ADDRESS 0

// ROS 2 Network Config
const char* ROS2_IP = "192.168.1.1";
const int ROS2_PORT = 15005;
int portionSize = 400; // Default portion size
unsigned long button_timer = 0;
char* WiFi_SSID = "SSID";
char* WiFi_PASSWORD = "Password";
IPAddress AgentIP(192, 168, 1, 1);
const int AgentPort = 8888;

// Camera Pins (ESP32-CAM AI Thinker)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

WiFiUDP udp;
AccelStepper stepper = AccelStepper(MOTOR_INTERFACE_TYPE, STEP_PIN, DIR_PIN);

// ROS 2 Communication
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
bool feeding = false;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

volatile bool motionDetected = false;
unsigned long motionStartTime = 0;

void FeedPet();
void startCamera();
void sendImage();

void error_loop() {
  while (1) {
    Serial.print("Error!");
    delay(100);
  }
}

// Interrupt for Motion Sensor
void motionISR() {
  motionDetected = true;
  motionStartTime = millis();
}

// Interrupt for Manual Button Press
void buttonISR() {
  if (millis() - button_timer > 250){
    button_timer = millis();
    FeedPet();
  }
}

// ROS 2 Subscriber Callback (Receives Portion Size & Feed Commands)
void feed_callback(const void* msg_in) {
  const std_msgs__msg__Int32* incoming_msg = (const std_msgs__msg__Int32*) msg_in;
  int command = incoming_msg->data;

  if (command > 0) {
    portionSize = command; // Update portion size
    EEPROM.write(EEPROM_ADDRESS, portionSize);
    EEPROM.commit();
    Serial.print("New Portion Size Saved: ");
    Serial.println(portionSize);
  } else if (command == -1) {
    FeedPet(); // Trigger feeding
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");

  EEPROM.begin(EEPROM_SIZE);
  portionSize = EEPROM.read(EEPROM_ADDRESS);
  if (portionSize == 255) {
    portionSize = 400;  // Set default value if EEPROM is empty
    EEPROM.write(EEPROM_ADDRESS, portionSize);
    EEPROM.commit();
  }

  stepper.setMaxSpeed(800);
  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  set_microros_wifi_transports(WiFi_SSID, WiFi_PASSWORD, AgentIP, AgentPort);
  Serial.println("Connected to ROS 2!");

  allocator = rcl_get_default_allocator();

  // Initialize ROS 2 node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "SmartFeederOutput"));

  // Create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "SmartFeederInput"));

  // Setup executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &feed_callback, ON_NEW_DATA));

  // Configure motion sensor & button
  pinMode(MOTION_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), motionISR, RISING);
  

  udp.begin(ROS2_PORT);
  startCamera();
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  if (motionDetected && millis() - motionStartTime < 60000) {
    sendImage();
    delay(500);
  } else {
    motionDetected = false;
  }
  // if (digitalRead(BUTTON_PIN) == LOW && millis() - button_timer > 250) {
  //   FeedPet();
  //   button_timer = millis();
  // }
}

void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
      config.frame_size = FRAMESIZE_UXGA;
      config.jpeg_quality = 10;
      config.fb_count = 2;
  } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.jpeg_quality = 12;
      config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
  }
}

void sendImage() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
      Serial.println("Camera capture failed");
      return;
  }

  unsigned int packetSize = 1024;
  int totalPackets = (fb->len + packetSize - 1) / packetSize;
  for (int i = 0; i < totalPackets; i++) {
      int chunkSize = min(packetSize, fb->len - (i * packetSize));
      udp.beginPacket(ROS2_IP, ROS2_PORT);
      udp.write(fb->buf + (i * packetSize), chunkSize);
      udp.endPacket();
      delay(5);
  }

  esp_camera_fb_return(fb);
}

void FeedPet() {
  //digitalWrite(LED_PIN, HIGH);
  Serial.println("Feeding Pet...");
  if (feeding) {
    Serial.println("Already feeding...");
    return;
  }
  feeding = true;
  stepper.setCurrentPosition(0);
  while (stepper.currentPosition() != portionSize) {
    stepper.setSpeed(60);
    stepper.runSpeed();
  }
  feeding = false;
  //digitalWrite(LED_PIN, LOW);
  Serial.println("Feeding Done.");
}
