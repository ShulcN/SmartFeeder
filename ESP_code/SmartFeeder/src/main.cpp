#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_camera.h>
#include <esp_task_wdt.h>

// Default Hardware Pins (configurable via web interface)
struct Config {
  // Stepper Motor Pins
  int dirPin = 14;
  int stepPin = 15;
  int ledPin = 4;
  
  // Sensor Pins
  int motionSensorPin = 13;
  int buttonPin = 12;
  
  // Motor Parameters
  int portionSize = 400;
  int motorSpeed = 200;
  int forwardSteps = 19;
  int backwardSteps = 12;
  
  // Network Settings
  char wifiSSID[32] = "ShulcHome_2";
  char wifiPassword[32] = "Rfrfirf5";
  char mqttServer[64] = "192.168.2.36";
  int mqttPort = 1883;
  char mqttUser[32] = "";
  char mqttPassword[32] = "";
  char deviceName[32] = "SmartFeeder";
  
  // Camera Settings
  int cameraFrameSize = 6; // FRAMESIZE_SVGA
  int cameraJpegQuality = 12;
  int videoStreamPort = 5005;
  
  // Topics
  char topicCommands[64] = "feeder/commands";
  char topicResponses[64] = "feeder/responses";
  
  // Add a magic number to validate EEPROM data
  uint32_t magic = 0xDEADBEEF;
};

// EEPROM addresses
#define EEPROM_SIZE 1024
#define CONFIG_ADDRESS 0

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

// Global variables
Config config;
AccelStepper stepper;
WebServer server(80);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WiFiUDP udp;

// State variables
bool feeding = false;
bool motionDetected = false;
bool mqttConnectedOnce = false;
bool apMode = false;
unsigned long motionStartTime = 0;
unsigned long button_timer = 0;
unsigned long lastMqttAttempt = 0;
unsigned long lastWifiAttempt = 0;
unsigned long lastVideoFrame = 0;

const unsigned long MOTION_STREAM_DURATION = 60000; // 1 минута

bool cameraStreamActive = false;
WiFiClient streamClient;


// Watchdog timer
#define WDT_TIMEOUT 30

// Function declarations
void loadConfig();
void saveConfig();
void setupWiFi();
void setupMQTT();
void setupWebServer();
void setupCamera();
void startVideoStream();
void feedPet(bool sendMqttResponse = false);
void handleMqttMessage(char* topic, byte* payload, unsigned int length);
void sendMqttResponse(const char* message);
void motionISR();
void buttonISR();
void stopVideoStream();
void handleMotionDetection();
//void setupWebServerVideoRoutes();
void setupCameraServer();

void setup() {
  Serial.begin(115200);
  // Add delay to allow serial monitor to connect
  delay(2000);
  Serial.println("\n\n=== Smart Pet Feeder Starting ===");
  Serial.println("Firmware Version: 1.0");
  
  // Initialize EEPROM and load configuration
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to initialize EEPROM");
    ESP.restart();
  }
  loadConfig();
  
  // Print current configuration
  Serial.println("Current Configuration:");
  Serial.printf("WiFi SSID: %s\n", config.wifiSSID);
  Serial.printf("MQTT Server: %s:%d\n", config.mqttServer, config.mqttPort);
  Serial.printf("Device Name: %s\n", config.deviceName);
  
  // Initialize stepper motor (only if pins are available)
  if (config.stepPin != 0 && config.dirPin != 0) {
    stepper = AccelStepper(AccelStepper::DRIVER, config.stepPin, config.dirPin);
    stepper.setMaxSpeed(800);
    Serial.println("Stepper motor initialized");
  }
  
  // Initialize pins (check availability for ESP32-CAM)
  // if (config.buttonPin != 0 && config.buttonPin != 1) { // Avoid TX/RX pins
  //   pinMode(config.buttonPin, INPUT_PULLUP);
  //   attachInterrupt(digitalPinToInterrupt(config.buttonPin), buttonISR, FALLING);
  //   Serial.printf("Button pin %d initialized\n", config.buttonPin);
  // }
  
  if (config.motionSensorPin != 0 && config.motionSensorPin != 1) {
    pinMode(config.motionSensorPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(config.motionSensorPin), motionISR, RISING);
    Serial.printf("Motion sensor pin %d initialized\n", config.motionSensorPin);
  }
  
  if (config.ledPin != 0 && config.ledPin != 1) {
    pinMode(config.ledPin, OUTPUT);
    digitalWrite(config.ledPin, LOW);
    Serial.printf("LED pin %d initialized\n", config.ledPin);
  }
  
  // Setup WiFi first
  Serial.println("Setting up WiFi...");
  setupWiFi();
  
  // Setup web server
  Serial.println("Setting up web server...");
  setupWebServer();
  
  // Setup MQTT only if WiFi is connected
  if (WiFi.status() == WL_CONNECTED && !apMode) {
    Serial.println("Setting up MQTT...");
    setupMQTT();
  }
  
  // Setup camera
  Serial.println("Setting up camera...");
  setupCamera();
  
  // Start video stream
  setupCameraServer();
  
  Serial.println("=== Setup completed! ===");
  if (apMode) {
    Serial.println("Device is in AP mode - connect to 'SmartFeeder_Config' WiFi");
    Serial.print("Configuration page: http://");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.print("Device IP: ");
    Serial.println(WiFi.localIP());
  }
}

void loop() {
  // Handle web server
  server.handleClient();
  
  // Handle WiFi connection
  if (WiFi.status() != WL_CONNECTED && !apMode) {
    if (millis() - lastWifiAttempt > 30000) { // Try reconnect every 30 seconds
      Serial.println("WiFi disconnected, attempting reconnection...");
      setupWiFi();
      lastWifiAttempt = millis();
    }
  }
  
  // Handle MQTT connection
  if (WiFi.status() == WL_CONNECTED && !apMode) {
    if (!mqttClient.connected()) {
      if (millis() - lastMqttAttempt > 5000) { // Try reconnect every 5 seconds
        setupMQTT();
        lastMqttAttempt = millis();
      }
    } else {
      mqttClient.loop();
    }
  }
  
  // Handle motion detection
  // if (motionDetected && millis() - motionStartTime < 60000) {
  //   // Motion detected, video stream is already running
  //   motionDetected = false; // Reset flag
  // }
  handleMotionDetection();

  if (digitalRead(config.buttonPin) == LOW && millis() - button_timer > 250) {
    feedPet(false);
    button_timer = millis();
  }
  
  // Feed watchdog if MQTT was connected at least once
  if (mqttConnectedOnce) {
    esp_task_wdt_reset();
  }
  
  delay(10);
}

void loadConfig() {
  Config tempConfig;
  EEPROM.get(CONFIG_ADDRESS, tempConfig);
  
  // Check if EEPROM contains valid data using magic number
  if (tempConfig.magic == 0xDEADBEEF && 
      tempConfig.portionSize > 0 && tempConfig.portionSize <= 10000 &&
      strlen(tempConfig.wifiSSID) > 0) {
    config = tempConfig;
    Serial.println("Configuration loaded from EEPROM");
  } else {
    Serial.println("EEPROM empty or corrupted, using defaults");
    // Keep default values and save them
    saveConfig();
  }
}

void saveConfig() {
  config.magic = 0xDEADBEEF; // Set magic number
  EEPROM.put(CONFIG_ADDRESS, config);
  if (EEPROM.commit()) {
    Serial.println("Configuration saved to EEPROM");
  } else {
    Serial.println("Failed to save configuration to EEPROM");
  }
}

void setupWiFi() {
  // Disconnect any previous connection
  WiFi.disconnect(true);
  delay(1000);
  
  Serial.printf("Attempting to connect to WiFi: %s\n", config.wifiSSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(config.wifiSSID, config.wifiPassword);
  
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) { // Increase attempts
    delay(1000);
    Serial.print(".");
    attempts++;
    
    // Print WiFi status for debugging
    if (attempts % 10 == 0) {
      wl_status_t status = WiFi.status();
      Serial.printf("\nWiFi Status: %d ", status);
      switch(status) {
        case WL_NO_SSID_AVAIL: Serial.print("(SSID not found)"); break;
        case WL_CONNECT_FAILED: Serial.print("(Connection failed)"); break;
        case WL_CONNECTION_LOST: Serial.print("(Connection lost)"); break;
        case WL_DISCONNECTED: Serial.print("(Disconnected)"); break;
        default: Serial.print("(Unknown)"); break;
      }
      Serial.print("\nContinuing");
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected successfully!");
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("DNS: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("Signal strength: %d dBm\n", WiFi.RSSI());
    apMode = false;
  } else {
    Serial.println();
    Serial.println("Failed to connect to WiFi, starting AP mode");
    
    // Stop STA mode before starting AP
    WiFi.mode(WIFI_OFF);
    delay(1000);
    
    WiFi.mode(WIFI_AP);
    if (WiFi.softAP("SmartFeeder_Config", "12345678")) {
      Serial.println("AP mode started successfully");
      Serial.printf("AP IP address: %s\n", WiFi.softAPIP().toString().c_str());
      Serial.println("Connect to 'SmartFeeder_Config' WiFi with password '12345678'");
      apMode = true;
    } else {
      Serial.println("Failed to start AP mode");
    }
  }
}

void setupMQTT() {
  if (strlen(config.mqttServer) == 0) {
    Serial.println("MQTT server not configured, skipping MQTT setup");
    return;
  }
  
  mqttClient.setServer(config.mqttServer, config.mqttPort);
  mqttClient.setCallback(handleMqttMessage);
  
  Serial.printf("Connecting to MQTT server: %s:%d\n", config.mqttServer, config.mqttPort);
  
  String clientId = String(config.deviceName) + "_" + String(random(0xffff), HEX);
  
  bool connected = false;
  if (strlen(config.mqttUser) > 0) {
    connected = mqttClient.connect(clientId.c_str(), config.mqttUser, config.mqttPassword);
  } else {
    connected = mqttClient.connect(clientId.c_str());
  }
  
  if (connected) {
    Serial.println("MQTT connected successfully");
    mqttClient.subscribe(config.topicCommands);
    Serial.printf("Subscribed to topic: %s\n", config.topicCommands);
    
    // Send IP address on connection
    String ipMessage = "{\"ip\":\"" + WiFi.localIP().toString() + "\",\"device\":\"" + config.deviceName + "\"}";
    mqttClient.publish(config.topicResponses, ipMessage.c_str());
    
    if (!mqttConnectedOnce) {
      mqttConnectedOnce = true;
      // Initialize watchdog timer
      esp_task_wdt_init(WDT_TIMEOUT, true);
      esp_task_wdt_add(NULL);
      Serial.println("Watchdog timer initialized");
    }
  } else {
    Serial.printf("MQTT connection failed, rc=%d\n", mqttClient.state());
    switch(mqttClient.state()) {
      case -4: Serial.println("Connection timeout"); break;
      case -3: Serial.println("Connection lost"); break;
      case -2: Serial.println("Connect failed"); break;
      case -1: Serial.println("Disconnected"); break;
      case 1: Serial.println("Bad protocol version"); break;
      case 2: Serial.println("Bad client ID"); break;
      case 3: Serial.println("Unavailable"); break;
      case 4: Serial.println("Bad credentials"); break;
      case 5: Serial.println("Unauthorized"); break;
    }
  }
}

void setupWebServer() {
  // Main configuration page
  server.on("/config", []() {
    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Smart Feeder Configuration</title>
    <meta charset="UTF-8">
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
        .section { margin: 20px 0; padding: 15px; border: 1px solid #ccc; border-radius: 5px; }
        input, select { width: 200px; padding: 8px; margin: 5px; border: 1px solid #ccc; border-radius: 3px; }
        button { padding: 10px 20px; margin: 10px 5px; background: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; }
        button:hover { background: #0056b3; }
        .status { background: #d4edda; border: 1px solid #c3e6cb; color: #155724; padding: 10px; border-radius: 5px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Smart Pet Feeder Configuration</h1>
        
        <div class="status">
            <strong>Device Status:</strong><br>
            IP: )" + (apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + R"(<br>
            Mode: )" + (apMode ? "Access Point" : "WiFi Connected") + R"(<br>
            Free Heap: )" + String(ESP.getFreeHeap()) + R"( bytes<br>
            Uptime: )" + String(millis() / 1000) + R"( seconds
        </div>
    
        <div class="section">
            <h3>WiFi Settings</h3>
            <form action="/save" method="POST">
                <input type="hidden" name="section" value="wifi">
                SSID: <input type="text" name="ssid" value=")" + String(config.wifiSSID) + R"(" required><br>
                Password: <input type="password" name="password" value=")" + String(config.wifiPassword) + R"("><br>
                <button type="submit">Save WiFi Settings</button>
            </form>
        </div>
        
        <div class="section">
            <h3>MQTT Settings</h3>
            <form action="/save" method="POST">
                <input type="hidden" name="section" value="mqtt">
                Server: <input type="text" name="server" value=")" + String(config.mqttServer) + R"("><br>
                Port: <input type="number" name="port" value=")" + String(config.mqttPort) + R"(" min="1" max="65535"><br>
                Username: <input type="text" name="user" value=")" + String(config.mqttUser) + R"("><br>
                Password: <input type="password" name="pass" value=")" + String(config.mqttPassword) + R"("><br>
                Device Name: <input type="text" name="device" value=")" + String(config.deviceName) + R"(" required><br>
                Commands Topic: <input type="text" name="cmd_topic" value=")" + String(config.topicCommands) + R"(" required><br>
                Responses Topic: <input type="text" name="resp_topic" value=")" + String(config.topicResponses) + R"(" required><br>
                <button type="submit">Save MQTT Settings</button>
            </form>
        </div>
        
        <div class="section">
            <h3>Hardware Settings</h3>
            <form action="/save" method="POST">
                <input type="hidden" name="section" value="hardware">
                Step Pin: <input type="number" name="step_pin" value=")" + String(config.stepPin) + R"(" min="0" max="39"><br>
                Dir Pin: <input type="number" name="dir_pin" value=")" + String(config.dirPin) + R"(" min="0" max="39"><br>
                LED Pin: <input type="number" name="led_pin" value=")" + String(config.ledPin) + R"(" min="0" max="39"><br>
                Motion Sensor Pin: <input type="number" name="motion_pin" value=")" + String(config.motionSensorPin) + R"(" min="0" max="39"><br>
                Button Pin: <input type="number" name="button_pin" value=")" + String(config.buttonPin) + R"(" min="0" max="39"><br>
                <button type="submit">Save Hardware Settings</button>
            </form>
        </div>
        
        <div class="section">
            <h3>Motor Settings</h3>
            <form action="/save" method="POST">
                <input type="hidden" name="section" value="motor">
                Portion Size (steps): <input type="number" name="portion" value=")" + String(config.portionSize) + R"(" min="1" max="10000" required><br>
                Motor Speed: <input type="number" name="speed" value=")" + String(config.motorSpeed) + R"(" min="50" max="1000"><br>
                Forward Steps: <input type="number" name="forward" value=")" + String(config.forwardSteps) + R"(" min="1" max="100"><br>
                Backward Steps: <input type="number" name="backward" value=")" + String(config.backwardSteps) + R"(" min="0" max="100"><br>
                <button type="submit">Save Motor Settings</button>
            </form>
        </div>
        
        <div class="section">
            <h3>Controls</h3>
            <button onclick="feedPet()"+R"(>Feed Pet</button>
            <button onclick="rebootDevice()"+R"(>Reboot Device</button>
            <button onclick="testWiFi()"+R"(>Test WiFi Connection</button>
            <p><strong>Video Stream:</strong> <a href="http://)" + WiFi.localIP().toString() + ":" + String(config.videoStreamPort) + R"(" target="_blank">Open Stream</a></p>
        </div>
    </div>
    
    <script>
        function feedPet() {
            fetch('/feed').then(response => response.text()).then(data => alert(data));
        }
        
        function rebootDevice() {
            if (confirm('Are you sure you want to reboot the device?')) {
                fetch('/reboot').then(() => alert('Device is rebooting...'));
            }
        }
        
        function testWiFi() {
            fetch('/test_wifi').then(response => response.text()).then(data => alert(data));
        }
    </script>
</body>
</html>
)";
    server.send(200, "text/html", html);
  });
  //setupWebServerVideoRoutes();
  setupCameraServer();
  // Save configuration
  server.on("/save", HTTP_POST, []() {
    String section = server.arg("section");
    bool needsReboot = false;
    
    if (section == "wifi") {
      if (server.arg("ssid") != String(config.wifiSSID) || 
          server.arg("password") != String(config.wifiPassword)) {
        needsReboot = true;
      }
      strncpy(config.wifiSSID, server.arg("ssid").c_str(), sizeof(config.wifiSSID) - 1);
      strncpy(config.wifiPassword, server.arg("password").c_str(), sizeof(config.wifiPassword) - 1);
      config.wifiSSID[sizeof(config.wifiSSID) - 1] = '\0';
      config.wifiPassword[sizeof(config.wifiPassword) - 1] = '\0';
    }
    else if (section == "mqtt") {
      strncpy(config.mqttServer, server.arg("server").c_str(), sizeof(config.mqttServer) - 1);
      config.mqttPort = server.arg("port").toInt();
      strncpy(config.mqttUser, server.arg("user").c_str(), sizeof(config.mqttUser) - 1);
      strncpy(config.mqttPassword, server.arg("pass").c_str(), sizeof(config.mqttPassword) - 1);
      strncpy(config.deviceName, server.arg("device").c_str(), sizeof(config.deviceName) - 1);
      strncpy(config.topicCommands, server.arg("cmd_topic").c_str(), sizeof(config.topicCommands) - 1);
      strncpy(config.topicResponses, server.arg("resp_topic").c_str(), sizeof(config.topicResponses) - 1);
      
      // Ensure null termination
      config.mqttServer[sizeof(config.mqttServer) - 1] = '\0';
      config.mqttUser[sizeof(config.mqttUser) - 1] = '\0';
      config.mqttPassword[sizeof(config.mqttPassword) - 1] = '\0';
      config.deviceName[sizeof(config.deviceName) - 1] = '\0';
      config.topicCommands[sizeof(config.topicCommands) - 1] = '\0';
      config.topicResponses[sizeof(config.topicResponses) - 1] = '\0';
    }
    else if (section == "hardware") {
      config.stepPin = server.arg("step_pin").toInt();
      config.dirPin = server.arg("dir_pin").toInt();
      config.ledPin = server.arg("led_pin").toInt();
      config.motionSensorPin = server.arg("motion_pin").toInt();
      config.buttonPin = server.arg("button_pin").toInt();
      needsReboot = true; // Hardware changes need reboot
    }
    else if (section == "motor") {
      config.portionSize = server.arg("portion").toInt();
      config.motorSpeed = server.arg("speed").toInt();
      config.forwardSteps = server.arg("forward").toInt();
      config.backwardSteps = server.arg("backward").toInt();
    }
    
    saveConfig();
    
    String response = "Configuration saved successfully!";
    if (needsReboot) {
      response += " Device will reboot in 3 seconds to apply changes.";
      server.send(200, "text/plain", response);
      delay(3000);
      ESP.restart();
    } else {
      server.send(200, "text/plain", response);
    }
  });
  
  // Manual feed
  server.on("/feed", []() {
    feedPet();
    server.send(200, "text/plain", "Feeding initiated successfully");
  });
  
  // Test WiFi
  server.on("/test_wifi", []() {
    String response = "WiFi Status: ";
    response += WiFi.status();
    response += "\nSSID: " + String(config.wifiSSID);
    response += "\nSignal: " + String(WiFi.RSSI()) + " dBm";
    response += "\nIP: " + WiFi.localIP().toString();
    server.send(200, "text/plain", response);
  });
  
  // Reboot
  server.on("/reboot", []() {
    server.send(200, "text/plain", "Device is rebooting...");
    delay(1000);
    ESP.restart();
  });
  
  server.begin();
  Serial.println("Web server started on port 80");
}

void setupCamera() {
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Проверка PSRAM и настройка качества
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
      Serial.println("PSRAM found - using high quality settings");
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
      Serial.println("No PSRAM - using standard settings");
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
    config.fb_count = 2;
  }

  // Инициализация камеры
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  // Получение сенсора для настройки
  sensor_t * s = esp_camera_sensor_get();
  
  // Настройки для OV3660
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  
  // Установка начального размера кадра
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

  // Настройки для AI-Thinker
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);

  Serial.println("Camera initialized successfully");
}



void setupCameraServer() {
  // Главная страница камеры
  server.on("/", HTTP_GET, []() {
    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Smart Feeder Camera</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 0; background: #263238; color: white; text-align: center; }
        .container { max-width: 1200px; margin: 0 auto; padding: 20px; }
        .stream-container { margin: 20px 0; }
        img { max-width: 100%; height: auto; border: 2px solid #37474f; border-radius: 8px; }
        .controls { margin: 20px 0; }
        button { 
            padding: 10px 20px; margin: 5px; background: #ff5722; color: white; 
            border: none; border-radius: 5px; cursor: pointer; font-size: 16px;
        }
        button:hover { background: #e64a19; }
        .status { 
            background: rgba(76, 175, 80, 0.1); border: 1px solid #4caf50; 
            color: #4caf50; padding: 10px; border-radius: 5px; margin: 10px 0;
        }
        .settings { text-align: left; background: rgba(255,255,255,0.1); padding: 15px; border-radius: 8px; margin: 20px 0; }
        select, input { padding: 5px; margin: 5px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🐾 Smart Pet Feeder Camera</h1>
        
        <div class="status">
            <strong>Device Status:</strong> Online | 
            IP: )" + WiFi.localIP().toString() + R"( | 
            Free Memory: )" + String(ESP.getFreeHeap()) + R"( bytes
        </div>
        
        <div class="stream-container">
            <h3>📹 Live Stream</h3>
            <img id="stream" src="/stream" style="display: none;">
            <div id="stream-status">Stream stopped</div>
        </div>
        
        <div class="controls">
            <button onclick='startStream()')"+R"(>▶️ Start Stream</button>
            <button onclick='stopStream()')"+R"(>⏹️ Stop Stream</button>
            <button onclick='capturePhoto()')"+R"(>📸 Capture</button>
            <button onclick='feedPet()')"+R"(>🍽️ Feed Pet</button>
        </div>
        
        <div class="settings">
            <h4>⚙️ Camera Settings</h4>
            <div>
                Resolution: 
                <select id="framesize" onchange='setFramesize()')"+R"(>
                    <option value="10">QVGA (320x240)</option>
                    <option value="9">CIF (352x288)</option>
                    <option value="8">VGA (640x480)</option>
                    <option value="7" selected>SVGA (800x600)</option>
                    <option value="6">XGA (1024x768)</option>
                    <option value="5">SXGA (1280x1024)</option>
                    <option value="13">UXGA (1600x1200)</option>
                </select>
            </div>
            <div>
                Quality: 
                <input type="range" id="quality" min="4" max="63" value="12" onchange='setQuality()')"+R"(>
                <span id="qualityValue">12</span>
            </div>
        </div>
        
        <div style="margin-top: 30px;">
            <a href="/config" style="color: #4caf50;">🔧 Configuration</a>
        </div>
    </div>
    
    <script>
        const streamImg = document.getElementById('stream');
        const streamStatus = document.getElementById('stream-status');
        let streamActive = false;
        
        function startStream() {
            streamImg.src = '/stream?' + Date.now();
            streamImg.style.display = 'block';
            streamStatus.textContent = 'Stream active';
            streamStatus.style.color = '#4caf50';
            streamActive = true;
        }
        
        function stopStream() {
            streamImg.src = '';
            streamImg.style.display = 'none';
            streamStatus.textContent = 'Stream stopped';
            streamStatus.style.color = '#ff5722';
            streamActive = false;
        }
        
        function capturePhoto() {
            window.open('/capture', '_blank');
        }
        
        function feedPet() {
            fetch('/feed').then(response => response.text()).then(data => alert(data));
        }
        
        function setFramesize() {
            const framesize = document.getElementById('framesize').value;
            fetch('/control?var=framesize&val=' + framesize)
                .then(() => { if(streamActive) setTimeout(startStream, 500); });
        }
        
        function setQuality() {
            const quality = document.getElementById('quality').value;
            document.getElementById('qualityValue').textContent = quality;
            fetch('/control?var=quality&val=' + quality);
        }
        
        // Обработка ошибок загрузки стрима
        streamImg.onerror = function() {
            streamStatus.textContent = 'Stream error - check connection';
            streamStatus.style.color = '#ff5722';
        };
        
        streamImg.onload = function() {
            if (streamActive) {
                streamStatus.textContent = 'Stream active';
                streamStatus.style.color = '#4caf50';
            }
        };
    </script>
</body>
</html>
)";
    server.send(200, "text/html", html);
  });

  // MJPEG стрим
  server.on("/stream", HTTP_GET, []() {
    WiFiClient client = server.client();
    String response = "HTTP/1.1 200 OK\r\n";
    response += "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n\r\n";
    client.print(response);
    
    static const char* _STREAM_BOUNDARY = "\r\n--123456789000000000000987654321\r\n";
    static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
    
    while (client.connected()) {
      camera_fb_t * fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        break;
      }
      
      if (fb->width > 400) {
        client.print(_STREAM_BOUNDARY);
        client.printf(_STREAM_PART, fb->len);
        client.write(fb->buf, fb->len);
      }
      esp_camera_fb_return(fb);
      
      if (!client.connected()) break;
    }
    
    Serial.println("Stream client disconnected");
  });

  // Захват одного кадра
  server.on("/capture", HTTP_GET, []() {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      server.send(500, "text/plain", "Camera capture failed");
      return;
    }
    
    server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
    server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
  });

  // Управление настройками камеры
  server.on("/control", HTTP_GET, []() {
    String var = server.arg("var");
    String val = server.arg("val");
    
    sensor_t * s = esp_camera_sensor_get();
    int res = 0;
    
    if (var == "framesize") {
      res = s->set_framesize(s, (framesize_t)val.toInt());
    } else if (var == "quality") {
      res = s->set_quality(s, val.toInt());
    } else if (var == "contrast") {
      res = s->set_contrast(s, val.toInt());
    } else if (var == "brightness") {
      res = s->set_brightness(s, val.toInt());
    } else if (var == "saturation") {
      res = s->set_saturation(s, val.toInt());
    } else if (var == "hmirror") {
      res = s->set_hmirror(s, val.toInt());
    } else if (var == "vflip") {
      res = s->set_vflip(s, val.toInt());
    }
    
    if (res == 0) {
      server.send(200, "text/plain", "OK");
    } else {
      server.send(500, "text/plain", "ERROR");
    }
  });

  // Перенаправить старую конфигурацию на новый адрес
  server.on("/config", HTTP_GET, []() {
    // Здесь поместить старый HTML конфигурации, но заменить главную страницу
    // на более простую без видео контролов
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
}



void feedPet(bool sendMqttResp) {
  if (feeding) {
    Serial.println("Already feeding, ignoring request");
    return;
  }
  
  Serial.println("Starting feeding sequence...");
  feeding = true;
  
  // Turn on LED
  if (config.ledPin != 0) {
    digitalWrite(config.ledPin, HIGH);
  }
  
  // Only run motor if pins are configured
  if (config.stepPin != 0 && config.dirPin != 0) {
    stepper.setCurrentPosition(0);
    long targetPosition = config.portionSize;
    long currentPos = 0;
    
    while (currentPos < targetPosition) {
      long nextForwardPos = currentPos + config.forwardSteps;
      long nextBackwardPos = nextForwardPos - config.backwardSteps;
      
      // Forward movement
      stepper.moveTo(nextForwardPos);
      while (stepper.distanceToGo() != 0) {
        stepper.setSpeed(config.motorSpeed);
        stepper.runSpeedToPosition();
      }
      
      // Backward movement
      stepper.moveTo(nextBackwardPos);
      while (stepper.distanceToGo() != 0) {
        stepper.setSpeed(-config.motorSpeed);
        stepper.runSpeedToPosition();
      }
      
      currentPos = nextBackwardPos;
    }
  } else {
    Serial.println("Motor pins not configured, simulating feed");
    delay(2000); // Simulate feeding time
  }
  
  feeding = false;
  
  // Turn off LED
  if (config.ledPin != 0) {
    digitalWrite(config.ledPin, LOW);
  }
  
  Serial.println("Feeding sequence completed");
  
  if (sendMqttResp && mqttClient.connected()) {
    String statusMsg = "{\"status\":\"feeding_complete\",\"portion_size\":" + String(config.portionSize) + "}";
    sendMqttResponse(statusMsg.c_str());
  }
}

void handleMqttMessage(char* topic, byte* payload, unsigned int length) {
  // Ensure payload is null-terminated
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  
  Serial.printf("MQTT message received on topic '%s': %s\n", topic, message);
  
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.printf("Failed to parse JSON: %s\n", error.c_str());
    return;
  }
  
  String command = doc["command"].as<String>();
  
  if (command == "feed") {
    feedPet(true);
  }
  else if (command == "reboot") {
    sendMqttResponse("{\"status\":\"rebooting\"}");
    delay(1000);
    ESP.restart();
  }
  else if (command == "set_portion") {
    int newPortion = doc["value"].as<int>();
    if (newPortion > 0 && newPortion <= 10000) {
      config.portionSize = newPortion;
      saveConfig();
      String msg = "{\"status\":\"portion_updated\",\"new_portion\":" + String(newPortion) + "}";
      sendMqttResponse(msg.c_str());
    } else {
      sendMqttResponse("{\"status\":\"error\",\"message\":\"Invalid portion size\"}");
    }
  }
  else if (command == "get_ip") {
    String ipMessage = "{\"ip\":\"" + WiFi.localIP().toString() + "\",\"device\":\"" + config.deviceName + "\"}";
    sendMqttResponse(ipMessage.c_str());
  }
  else if (command == "status") {
    String statusMessage = "{\"status\":\"online\",\"ip\":\"" + WiFi.localIP().toString() + 
                          "\",\"portion_size\":" + String(config.portionSize) + 
                          ",\"feeding\":" + (feeding ? "true" : "false") + 
                          ",\"free_heap\":" + String(ESP.getFreeHeap()) + 
                          ",\"uptime\":" + String(millis() / 1000) + "}";
    sendMqttResponse(statusMessage.c_str());
  }
  else {
    Serial.printf("Unknown command: %s\n", command.c_str());
    sendMqttResponse("{\"status\":\"error\",\"message\":\"Unknown command\"}");
  }
}

void sendMqttResponse(const char* message) {
  if (mqttClient.connected()) {
    if (mqttClient.publish(config.topicResponses, message)) {
      Serial.printf("MQTT response sent: %s\n", message);
    } else {
      Serial.println("Failed to send MQTT response");
    }
  } else {
    Serial.println("MQTT not connected, cannot send response");
  }
}


// Обновленная функция обработки motion sensor
void IRAM_ATTR motionISR() {
  static unsigned long lastTrigger = 0;
  unsigned long now = millis();
  // Debounce - ignore triggers within 2000ms (2 секунды)
  if (now - lastTrigger > 2000) {
    motionDetected = true;
    motionStartTime = now;
    lastTrigger = now;
  }
}

// Функция для отправки MQTT сообщения о движении
void sendMotionDetectedMqtt() {
  if (mqttClient.connected()) {
    String streamUrl = "http://" + WiFi.localIP().toString() + "/stream";
    String motionMessage = "{\"event\":\"motion_detected\",\"timestamp\":" + String(millis()) + 
                          ",\"stream_url\":\"" + streamUrl + "\",\"device\":\"" + config.deviceName + 
                          "\",\"duration\":" + String(MOTION_STREAM_DURATION / 1000) + "}";
    
    if (mqttClient.publish(config.topicResponses, motionMessage.c_str())) {
      Serial.printf("Motion detection MQTT sent: %s\n", motionMessage.c_str());
    } else {
      Serial.println("Failed to send motion detection MQTT");
    }
  }
}

// Обновленная часть функции loop() для обработки движения
void handleMotionDetection() {
  if (motionDetected) {
    Serial.println("Motion detected! Sending MQTT notification...");
    
    // Отправляем MQTT сообщение с URL стрима
    if (mqttClient.connected()) {
      String streamUrl = "http://" + WiFi.localIP().toString() + "/stream";
      String motionMessage = "{\"event\":\"motion_detected\",\"timestamp\":" + String(millis()) + 
                            ",\"stream_url\":\"" + streamUrl + "\",\"device\":\"" + config.deviceName + 
                            "\",\"camera_url\":\"http://" + WiFi.localIP().toString() + "/\"}";
      
      if (mqttClient.publish(config.topicResponses, motionMessage.c_str())) {
        Serial.printf("Motion detection MQTT sent: %s\n", motionMessage.c_str());
      } else {
        Serial.println("Failed to send motion detection MQTT");
      }
    }
    
    motionDetected = false; // Reset flag
  }
}



// // Обновите HTML в функции setupWebServer() - добавьте эту секцию в контейнер
// String getVideoControlsHTML() {
//   String s = R"(
//         <div class="section">
//             <h3>Video Stream Controls</h3>
//             <button onclick="startVideoStream()"+R"(>Start Video Stream</button>
//             <button onclick="stopVideoStream()"+R"(>Stop Video Stream</button>
//             <button onclick="checkVideoStatus()"+R"(>Check Status</button>
//             <div id="videoStatus" style="margin-top: 10px; padding: 10px; background: #f8f9fa; border-radius: 5px;"></div>
//             <div style="margin-top: 15px;">
//                 <strong>Stream URL:</strong> 
//                 <a href="http://)" + WiFi.localIP().toString() + R"(/stream" target="_blank" id="streamLink">
//                     http://)" + WiFi.localIP().toString() + R"(/stream
//                 </a>
//             </div>
//             <div style="margin-top: 10px;">
//                 <video id="videoPlayer" width="640" height="480" style="display: none; border: 1px solid #ccc;">
//                     <source src="http://)" + WiFi.localIP().toString() + R"(/stream" type="video/mp4">
//                     Your browser does not support the video tag.
//                 </video>
//                 <br>
//                 <button onclick="toggleVideoPlayer()"+R"(>Toggle Video Preview</button>
//             </div>
//         </div>
// )";

//   return s;
// }

// JavaScript функции для добавления в HTML
String getVideoControlsJS() {
  return R"(
        function startVideoStream() {
            fetch('/video/start')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('videoStatus').innerHTML = 
                        '<span style="color: green;">✓ ' + data.message + '</span>';
                    console.log('Stream URL:', data.stream_url);
                })
                .catch(error => {
                    document.getElementById('videoStatus').innerHTML = 
                        '<span style="color: red;">✗ Error starting stream</span>';
                });
        }
        
        function stopVideoStream() {
            fetch('/video/stop')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('videoStatus').innerHTML = 
                        '<span style="color: orange;">⊖ ' + data.message + '</span>';
                })
                .catch(error => {
                    document.getElementById('videoStatus').innerHTML = 
                        '<span style="color: red;">✗ Error stopping stream</span>';
                });
        }
        
        function checkVideoStatus() {
            fetch('/video/status')
                .then(response => response.json())
                .then(data => {
                    let status = data.active ? 'Active' : 'Inactive';
                    let motionStatus = data.motion_active ? ' (Motion Triggered)' : '';
                    document.getElementById('videoStatus').innerHTML = 
                        'Stream Status: <strong>' + status + motionStatus + '</strong>';
                })
                .catch(error => {
                    document.getElementById('videoStatus').innerHTML = 
                        '<span style="color: red;">✗ Error checking status</span>';
                });
        }
        
        function toggleVideoPlayer() {
            const video = document.getElementById('videoPlayer');
            if (video.style.display === 'none') {
                video.style.display = 'block';
                video.src = 'http://)" + WiFi.localIP().toString() + R"(/stream?' + new Date().getTime();
            } else {
                video.style.display = 'none';
                video.src = '';
            }
        }
        
        // Автоматическая проверка статуса каждые 5 секунд
        setInterval(checkVideoStatus, 5000);
)";
}



void IRAM_ATTR buttonISR() {
  static unsigned long lastPress = 0;
  unsigned long now = millis();
  
  // Debounce - ignore presses within 500ms
  if (now - lastPress > 500) {
    lastPress = now;
    // Schedule feeding (don't do it in ISR)
    feedPet(false);
    button_timer = now;
  }
}