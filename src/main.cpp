// ESP32 GeoFence Monitor

#include <WiFi.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>

const String VERSION = "V1.0";

hw_timer_t *timer = NULL;
volatile bool timeoutFlag = false;

Preferences prefs;

// wifi
WiFiClient wifiClient;
PubSubClient mqttServer(wifiClient);
String ssid = "";
String password = "";

char serverIP[16];
const int port = 8080;

#define WHEEL_DEBOUNCE_DELAY 10
#define BUTTON_DEBOUNCE_DELAY 100
#define DEBOUNCE_DELAY_HELD 3000
#define MS_DELAY 10

#define CASE_PAIR 10
#define CASE_MEASURE_WHEEL 20
#define CASE_HOME 2

constexpr const char *IOT_TYPE_VEHICLE = "Vehicle";
constexpr const char *IOT_TYPE_MOBILE_MACHINE = "Mobile Machine";
constexpr const char *IOT_TYPE_STATIONARY_MACHINE = "Stationary Machine";
constexpr const char *IOT_TYPE_WHEEL = "Distance Wheel";

// Flash Settings
constexpr const char *SETTING_TICKS_PER_M = "ticksPerM";
constexpr const char *SETTING_IOT_TYPE = "iotType";

char settingsIotType[32] = {0};
int32_t settingsTicksPerMeter = 0;

int wifiCasePtr = 0;
int mainCasePtr = 0;
bool isBluetoothConnected = false;
bool isWifiConnected = false;
bool isMqttServiceConnected = false;
bool isRPiConnectingBusy = false;
bool isRPiConnected = false;
bool gotWifiCredentials = false;
int wheelTicksCount = 0;

// Commands
const String CMD_SHARED_WIFI_CREDENTIALS = "wificred:"; // Format: wificred:ssid#password

// Button Debounce
volatile bool debPairFallEdge = false;
volatile bool debPairPressed = false;
volatile bool debPairHeld = false;
volatile int debPairLastTime = 0;
bool btnPairPressed = false;

// Wheel 1 sensor Debounce
// UH81046
volatile bool debWheel1FallEdge = false;
volatile bool debWheel1RiseEdge = false;
volatile bool debWheel1High = false;
volatile bool debWheel1Low = false;
volatile int debWheel1LastTime = 0;
bool wheelSensor1Low = false;

// Wheel 2 sensor Debounce
volatile bool debWheel2FallEdge = false;
volatile bool debWheel2RiseEdge = false;
volatile bool debWheel2High = false;
volatile bool debWheel2Low = false;
volatile int debWheel2LastTime = 0;
bool wheelSensor2Low = false;

// MQTT Command
constexpr size_t JSON_MAX_CMD = 32; // MAX Len
const String MQTT_CMD_REQ_MONITOR = "#REQ_MONITOR";
const String MQTT_CMD_FOUND_MONITOR = "#FOUND_MONITOR";
const String MQTT_CMD_CONNECT_MONITOR = "#CONNECT_MONITOR";
const String MQTT_CMD_DISCONNECT_MONITOR = "#DISCONNECT_MONITOR";
const String MQTT_CMD_DEVICE_ID = "#DEVICE_ID";
const String MQTT_CMD_ACK = "#ACK";
const String MQTT_CMD_MONITOR_DATA = "#MONITOR_DATA";
const String MQTT_CMD_MEASURE_DATA = "#MEASURE_DATA";

// MQTT Topics
constexpr size_t JSON_MAX_TOPIC = 32; // MAX Len
const String MQTT_TOPIC_FROM_IOT = "mqtt/from/iot";
const String MQTT_TOPIC_TO_IOT = "mqtt/to/iot";
String MQTT_TOPIC_TO_IOT_PRIVATE = "";
const String MQTT_TOPIC_CRED = "mqtt/credentials";

// MQTT Device ID
constexpr size_t JSON_MAX_DEVICE_ID = 32;

// MQTT JSON
const String MQTT_JSON_FROM_DEVICE_ID = "from";
const String MQTT_JSON_TO_DEVICE_ID = "to";
const String MQTT_JSON_TOPIC = "topic";
const String MQTT_JSON_PAYLOAD = "payload";
const String MQTT_JSON_CMD = "cmd";
const String MQTT_JSON_WHEEL_DISTANCE = "wheel_distance";
const String MQTT_JSON_MEASUREMENTS = "measurements";
const String MQTT_JSON_DATA = "data";

const String JSON_SETTING_OPERATOR = "operator";
const String JSON_SETTING_FOREMAN = "foreman";
const String JSON_SETTING_DISTANCE = "distance";
const String JSON_SETTING_LINES = "lines";  

// FLASH Data Structure
struct Measure {
  char nameOperator[17];     // max 16 chars + null
  char nameForeman[17];     // max 16 chars + null
  float distance;
  int lines;
};

Measure thisMeasure;    
uint8_t measureCount = 0;
uint8_t measureIndex = 0;
const char* FLASH_MEASURE_KEY = "measures";
const char* FLASH_MEASURE_DATA = "data";

#define MQTT_PAYLOAD_MAX 255
using MqttJsonDoc = StaticJsonDocument<
  JSON_OBJECT_SIZE(5) +               // root
  JSON_OBJECT_SIZE(2) +               // payload
  JSON_ARRAY_SIZE(10) +               // measurements[]
  (JSON_OBJECT_SIZE(4) * 10)  +   // Measure structs
  JSON_MAX_DEVICE_ID +
  JSON_MAX_CMD +
  JSON_MAX_TOPIC +
  80                                  // JSON key strings
>;

// IO
#define BUT_PAIR 20           // BT isPairing button
#define SENSOR_WHEEL_1 19     // Wheel distance sensor1
#define SENSOR_WHEEL_2 5      // Wheel distance sensor2
#define LED_BLUETOOTH 27      // Blue
#define LED_WIFI_CONNECTED 12 // Red
#define LED_MQTT_CONNECTED 14 // Green
#define BUZZER 23             // Buzzer

#define SDA_PIN 21
#define SCL_PIN 22

#define KEY_COL1 2
#define KEY_COL2 0
#define KEY_COL3 4 
#define KEY_COL4 16
#define KEY_ROW1 26
#define KEY_ROW2 25
#define KEY_ROW3 33
#define KEY_ROW4 15

// LCD
LiquidCrystal_I2C lcd_i2c(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows

// Keypad
const int ROWS = 4;
const int COLS = 4;

int rowPins[ROWS] = {KEY_ROW1, KEY_ROW2, KEY_ROW3, KEY_ROW4};
int colPins[COLS] = {KEY_COL1, KEY_COL2, KEY_COL3, KEY_COL4};

// Connector front: R1,R2,R3,R3,C4,C3,C2,C1
// Pinout:          26,25,33,15,16,04,00,02

const char* keys[ROWS][COLS] = {
  {"1","2","3","Up"},
  {"4","5","6","Down"},
  {"7","8","9","Back"},
  {"Start","0","Stop","Enter"}
};

#define KEY_0 "0"
#define KEY_1 "1"
#define KEY_2 "2"
#define KEY_3 "3"
#define KEY_4 "4"
#define KEY_5 "5"
#define KEY_6 "6"
#define KEY_7 "7"
#define KEY_8 "8"
#define KEY_9 "9"
#define KEY_START "Start"
#define KEY_STOP "Stop"
#define KEY_ENTER "Enter"
#define KEY_BACK "Back"
#define KEY_UP "Up"
#define KEY_DOWN "Down"
#define KEY_PAIR "Stop"

char key[10];
String prevKey = "";
int pairModePressCount = 0;
#define PAIR_MODE_PRESS_COUNT 5

// Task handle
TaskHandle_t GeneralTaskHandle = NULL;
TaskHandle_t ConnectWiFiTaskHandle = NULL;
TaskHandle_t IotTaskHandle = NULL;

// Bluetooth
#define PAIR_TIMEOUT 20 // seconds
#define DISPLAY_TIMEOUT 3 // seconds
#define SERVICE_UUID "f3a1c2d0-6b4e-4e9a-9f3e-8d2f1c9b7a1e"
#define CHAR_UUID "c7b2e3f4-1a5d-4c3b-8e2f-9a6b1d8c2f3a"
const String NAME_PREFIX = "iOT"; // Bluetooth Prefix
NimBLECharacteristic *pChar = nullptr;
NimBLEServer *pServer = nullptr;
NimBLEService *pService = nullptr;

int runningTime = 0; // seconds
bool time_1sec_Flag = false;
String connectedDeviceId;
String myDeviceId;
String fromDeviceId;
String toDeviceId;
bool newIotDataAvailable = false;
bool newMeasurementsPushed = false;
double wheelDistance = 0;
double oldDistance = 0;
char nrOfLanesToCut[3];
uint8_t laneIndex = 0;

bool startKeyPressed = false;
bool stopKeyPressed = false;
bool enterKeyPressed = false;
bool backKeyPressed = false;
bool upKeyPressed = false;
bool downKeyPressed = false;
bool newNumKeyPressed = false;
bool isPairing = false;
bool btConnected = false;
bool wifiConnected = false;
bool isAdvertising = false;
bool androidConnected = false;
bool dataACK = false;
bool isRunning = false;

// Battery
int batteryLevel = 81; // Percentage

// Debug
const bool PRINT_ERRORS = true;
const bool PRINT_GENERAL_DEBUG = true;
const bool PRINT_CREDENTIALS_DEBUG = true;
const bool PRINT_WIFI_DEBUG = true;
const bool PRINT_BT_DEBUG = true;
const bool PRINT_DEBOUNCE_DEBUG = true;


// Declare functions
void DebouncePairBtn();
void SendHttpMsg(String msg);
void buttonISR();
void wheelSensor1ISR();
void wheelSensor2ISR();
void DebounceWheelSensor1();
void DebounceWheelSensor2();
void bt_StopServer();
void bt_StartServer();
void bt_StartAdvertising();
void mqttTX(const JsonDocument &msg, const String &topic);
void mqttRx(char *topic, byte *payload, unsigned int length);
void PrintDebug(String, bool);
void writeLCD(String line1, String line2);
void writeLCD(String line1, String line2, bool cnt);
bool getKeypad(char *out, size_t outSize);
void saveDistancesToNVM(int value);
void saveWifiCredentials(String ssid, String password, String ip);
void removeDistancesFromNVM();
void loadDistancesFromNVM(std::vector<int> &nums);
bool loadMeasureFromFlash(const char* key, Measure &out);
void loadMeasureCountFromFlash();
void deleteMeasuresFromFlash();
void mqttReportMyID();
bool mqttReportMeasurement(int);
void getAllMeasurements(std::vector<Measure> &data);
bool wifiCredentialsExist();

//------------------------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------------------------

// Timer
void IRAM_ATTR onTimerOneSec()
{
  if (runningTime == 0)
  {
    timeoutFlag = true;
    timerAlarmDisable(timer);
  }
  else
  {
    runningTime--;
  }
}
void startTimout(int seconds)
{
  runningTime = seconds;
  timeoutFlag = false;
  pairModePressCount = 0;
  timerAlarmEnable(timer);
}

// Interrupts
void IRAM_ATTR buttonISR()
{
  debPairFallEdge = true;
}
void IRAM_ATTR wheelSensor1ISR()
{
  debWheel1FallEdge = true;
}
void IRAM_ATTR wheelSensor2ISR()
{
  debWheel2FallEdge = true;
}

// Tasks
void GeneralTask(void *parameter)
{
  int delay = 0;
  int step = 0;
  bool startup = true;
  int cntAdvertising = 0;

  bool ledState = LOW;
  bool ledWifiState = LOW;
  bool ledMqttState = LOW;
  bool ledAdvertisingState = LOW;
  bool showStack = true;

  for (;;)
  {
    // Blink LED
    if (startup){
      if (delay++ >= 300)
      {
        delay = 0;

        switch (step)
        {
        case 0:
        case 4:
        case 8:
          digitalWrite(LED_BLUETOOTH, HIGH);
          digitalWrite(LED_WIFI_CONNECTED, LOW);
          digitalWrite(LED_MQTT_CONNECTED, LOW);
          break;

        case 1:
        case 5:
        case 9:
          digitalWrite(LED_BLUETOOTH, HIGH);
          digitalWrite(LED_WIFI_CONNECTED, HIGH);
          digitalWrite(LED_MQTT_CONNECTED, LOW);
          break;

        case 2:
        case 6:
        case 10:
          digitalWrite(LED_BLUETOOTH, HIGH);
          digitalWrite(LED_WIFI_CONNECTED, HIGH);
          digitalWrite(LED_MQTT_CONNECTED, HIGH);
          break;

        case 3:
        case 7:
        case 11:
          digitalWrite(LED_BLUETOOTH, LOW);
          digitalWrite(LED_WIFI_CONNECTED, LOW);
          digitalWrite(LED_MQTT_CONNECTED, LOW);
          break;
        }

        step++;

        if (step > 11)
        {
          startup = false;
        }
      }
    }
    else
    {

      if (isBluetoothConnected)
        digitalWrite(LED_BLUETOOTH, HIGH); // Blue
      else
        digitalWrite(LED_BLUETOOTH, LOW);

      if (isWifiConnected)
        digitalWrite(LED_WIFI_CONNECTED, HIGH); // Red
      else
        digitalWrite(LED_WIFI_CONNECTED, LOW);

      if (isAdvertising)
      {
        if (delay++ >= 700)
        {
          delay = 0;
          ledMqttState = !ledMqttState;

          if (ledMqttState)
            digitalWrite(LED_MQTT_CONNECTED, HIGH); // Green
          else
            digitalWrite(LED_MQTT_CONNECTED, LOW);
        }
      }
    }

    // Keypad
    if(getKeypad(key, sizeof(key))){
      
      if(showStack){
        showStack = false;
        Serial.printf("%s Stack free: %u bytes\n", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(GeneralTaskHandle));
     }
      
      PrintDebug(String("Key Pressed: ") + key, PRINT_GENERAL_DEBUG);

      // Control
      if (strcmp(key, KEY_START) == 0)  startKeyPressed = true;
      else if (strcmp(key, KEY_ENTER) == 0)  enterKeyPressed = true;
      else if (strcmp(key, KEY_BACK) == 0)  backKeyPressed = true;
      else if (strcmp(key, KEY_UP) == 0)  upKeyPressed = true;
      else if (strcmp(key, KEY_DOWN) == 0)  downKeyPressed = true;
      
      // Stop / Pair
      else if (strcmp(key, KEY_STOP) == 0){
        if (!isPairing && !isRunning){
          if (pairModePressCount == 0){
            startTimout(5); // 5 seconds to enter isPairing mode
          }

          if (++pairModePressCount >= PAIR_MODE_PRESS_COUNT){
            pairModePressCount = 0;
            PrintDebug("Pairing MODE", PRINT_GENERAL_DEBUG);
            isPairing = true;
          }
        }
      }
      // Number key
      else newNumKeyPressed = true; 

    }
    
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
void wifiConnectTask(void *parameter){
  int retry = 0;
  bool printed = false;
  bool showMsgOnce = true;

  for (;;){
    if (!printed)
    {
      PrintDebug("WIFI Task Running", PRINT_WIFI_DEBUG);
      printed = true;
    }

    switch (wifiCasePtr)
    {
      // Start Bluetooth
    case 0:{
      PrintDebug("Starting Bluetooth Connection", PRINT_WIFI_DEBUG);

      isBluetoothConnected = false;
      isWifiConnected = false;
      isMqttServiceConnected = false;
      gotWifiCredentials = false;
      showMsgOnce = true;

      bt_StartServer();
      bt_StartAdvertising();
      wifiCasePtr++;
    }
    break;

    // Wait for bluetooth Connection
    case 1:
    {
      if (isBluetoothConnected)
      {
        digitalWrite(LED_BLUETOOTH, HIGH); // Solid ON
        PrintDebug("Waiting for Wifi Credentials ... ", PRINT_WIFI_DEBUG);
        wifiCasePtr++;
      }
    }
    break;

    // Wait for Wifi Credentials via Bluetooth
    case 2:
    {
      if (gotWifiCredentials)
      {
        // Start WiFi Connection
        isWifiConnected = false;
        isMqttServiceConnected = false;

        saveWifiCredentials(ssid, password, serverIP);
        wifiCasePtr++;
      }
    }
    break;

    // Connecting Wifi
    case 3:
    {
      WiFi.begin(ssid, password);
      wifiCasePtr++;
    }
    break;

    // Wifi Connected
    case 4:
    {
      if (WiFi.status() == WL_CONNECTED)
      {
        String ip = WiFi.localIP().toString().c_str();

        PrintDebug(String("Wifi Connected: ") + WiFi.SSID(), PRINT_WIFI_DEBUG);
        PrintDebug("IP Address: " + ip , PRINT_WIFI_DEBUG);
        
        isWifiConnected = true;
        digitalWrite(LED_WIFI_CONNECTED, HIGH); // Red

        PrintDebug(String("Start MQTT ... ") + serverIP + ":1883", PRINT_WIFI_DEBUG);    
        mqttServer.setServer(serverIP, 1883);
        mqttServer.setCallback(mqttRx);

        wifiCasePtr++;
      }
    }
    break;

    // Connecting MQTT
    case 5:
    {
      if (mqttServer.connect(myDeviceId.c_str()))
      {
        PrintDebug("MQTT Connected", PRINT_WIFI_DEBUG);

        isMqttServiceConnected = true;
        mqttServer.loop();
        digitalWrite(LED_MQTT_CONNECTED, HIGH); // Green
        mqttReportMyID();
      
        wifiCasePtr++;
      }
      else
      {
        PrintDebug("MQTT FAILED: " + String(mqttServer.state()), PRINT_WIFI_DEBUG);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
      }
    }
    break;

    // Subscribe MQTT Topics
    case 6:
    {
      // Broadcast Subscribe
      mqttServer.subscribe(MQTT_TOPIC_TO_IOT.c_str());
      PrintDebug((String("MQTT Subscribe ...") + MQTT_TOPIC_TO_IOT.c_str()), PRINT_WIFI_DEBUG);

      // Private Subscribe
      MQTT_TOPIC_TO_IOT_PRIVATE = MQTT_TOPIC_TO_IOT + '/' + myDeviceId;
      mqttServer.subscribe(MQTT_TOPIC_TO_IOT_PRIVATE.c_str());
      PrintDebug((String("MQTT Subscribe ...") + MQTT_TOPIC_TO_IOT_PRIVATE.c_str()), PRINT_WIFI_DEBUG);

      Serial.printf("%s Stack free: %u bytes\n", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(ConnectWiFiTaskHandle));
      retry = 3;  // Push Data Retry in next
      wifiCasePtr++;
    }
    break;

    // Keep MQTT alive - Check Connection
    case 7:
    {
      mqttServer.loop(); // Keep MQTT Alive

      if (WiFi.status() != WL_CONNECTED)
      {
        PrintDebug("Lost WIFI Connection", PRINT_WIFI_DEBUG);
        wifiCasePtr = 0;    // Restart connection
      }

      // Push measurements
      if(measureCount > 0 && retry != 0) {
        measureIndex = measureCount-1;
        wifiCasePtr = 10;
      }

      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    break;

    // Push Measurements
    case 10:{
      mqttServer.loop(); // Keep MQTT Alive

      if(measureIndex > 0){
        PrintDebug(String("Pushing Measurement: ") + String(measureIndex), PRINT_WIFI_DEBUG);
        mqttReportMeasurement(measureIndex);
        startTimout(5);
        wifiCasePtr++;
      }
      else{
        deleteMeasuresFromFlash();
        newMeasurementsPushed = true;
        wifiCasePtr = 7;
      }
    }
    break;

    case 11:{
      // Timeout
      if(timeoutFlag){
        timeoutFlag = false;
        PrintDebug(String("TIMEOUT"), PRINT_WIFI_DEBUG);
        retry--;
        wifiCasePtr = 7;
      }

      // Base ACK
      if(dataACK){
        dataACK = false;
        measureIndex--;
        wifiCasePtr = 10;
      }
    }

    break;
    default:
      break;
    }
  }
}
void IotTask(void *parameter){
  bool printed = false;
  bool showStack = true;
  int casePtr = 0;

  for (;;)
  {
    if (!printed)
    {
      PrintDebug("Iot Task Running", PRINT_GENERAL_DEBUG);
      printed = true;
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);

    // Distance Wheel
    if (strcmp((const char *)settingsIotType, IOT_TYPE_WHEEL) == 0)
    {
      DebounceWheelSensor1();
      DebounceWheelSensor2();

      if(showStack){
        showStack = false;
        Serial.printf("%s Stack free: %u bytes\n", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(IotTaskHandle));
     }

      // Get Wheel Distance
      switch (casePtr) {
        case 0:{
          // Forward
          if (wheelSensor1Low)
            casePtr = 10;

          // Reverse
          if (wheelSensor2Low)
            casePtr = 20;
        }
        break;

        // Forward
        case 10:{
          // Forward
          if (!wheelSensor1Low)
            casePtr++;

          // Reverse
          if (wheelSensor2Low)
            casePtr = 0;
        }
        break;

        case 11:{
          // Forward
          if (!wheelSensor2Low)
            casePtr++;

          // Reverse
          if (wheelSensor1Low)
            casePtr = 0;
        }
        break;

        case 12:
          if (!wheelSensor2Low)
          {
            wheelTicksCount++;
            wheelDistance = (double)wheelTicksCount / settingsTicksPerMeter;
            PrintDebug("Forward:" + String(wheelDistance), PRINT_GENERAL_DEBUG);
            newIotDataAvailable = true;
            casePtr = 0;
          }
          break;

        // Reverse
        case 20:{
          // Reverse
          if (!wheelSensor2Low)
            casePtr++;

          // Forward
          if (wheelSensor1Low)
            casePtr = 0;
        }
        break;

        case 21:{
          // Reverse
          if (wheelSensor1Low)
            casePtr++;

          // Forward
          if (wheelSensor2Low)
            casePtr = 0;
        }
        break;

        case 22:{
          if (!wheelSensor1Low)
          {
            if (wheelTicksCount > 0) {
              wheelTicksCount--;
            }

            wheelDistance = (double)wheelTicksCount / settingsTicksPerMeter;
            PrintDebug("Reverse:" + String(wheelDistance), PRINT_GENERAL_DEBUG);
            newIotDataAvailable = true;
            casePtr = 0;
          }
        }
        break;

        default:
          casePtr = 0;
          break;
      }
    }
  }
}

// Debounce
void DebouncePairBtn(){
  // if (debPairFallEdge)
  // {
  //   debPairLastTime = millis();
  //   debPairFallEdge = false;
  // }

  // if (!debPairPressed && !debPairHeld)
  // {
  //   if (debPairLastTime > 0)
  //   {
  //     if (digitalRead(BUT_PAIR))
  //     {
  //       debPairLastTime = 0;
  //       return;
  //     }

  //     // Button Pressed
  //     uint32_t currentTime = millis();
  //     if (currentTime - debPairLastTime > BUTTON_DEBOUNCE_DELAY)
  //     {
  //       debPairPressed = true;
  //       btnPairPressed = true;
  //       PrintDebug("Button Pressed", PRINT_DEBOUNCE_DEBUG);
  //     }
  //   }
  // }
  // else if (debPairPressed && !debPairHeld)
  // {
  //   uint32_t currentTime = millis();

  //   // Start Bluetooth Pairing
  //   // Button HELD
  //   if (currentTime - debPairLastTime > DEBOUNCE_DELAY_HELD)
  //   {
  //     debPairHeld = true;
  //     PrintDebug("Button HELD", PRINT_DEBOUNCE_DEBUG);
  //   }

  //   // Wait for release before HELD
  //   if (digitalRead(BUT_PAIR) == HIGH)
  //   {
  //     debPairPressed = false;
  //     debPairHeld = false;
  //     debPairLastTime = 0;
  //     PrintDebug("Button Released", PRINT_DEBOUNCE_DEBUG);
  //   }
  // }
  // else
  // {
  //   // Button HELD (Wait for release)
  //   if (digitalRead(BUT_PAIR) == HIGH)
  //   {
  //     debPairPressed = false;
  //     debPairHeld = false;
  //     debPairLastTime = 0;
  //     PrintDebug("Button Released", PRINT_DEBOUNCE_DEBUG);
  //   }
  // }
}
void DebounceWheelSensor1()
{
  // if (debWheel1FallEdge)
  // {
  //   debWheel1LastTime = millis();
  //   debWheel1FallEdge = false;
  // }

  if (!debWheel1Low)
  {
    // if (debWheel1LastTime > 0)
    //{
    if (digitalRead(SENSOR_WHEEL_1))
    {
      // debWheel1LastTime = 0;
      return;
    }

    // Sensor Falling Edge
    // uint32_t currentTime = millis();
    // if (currentTime - debWheel1LastTime > WHEEL_DEBOUNCE_DELAY)
    //{
    debWheel1Low = true;
    wheelSensor1Low = true;
    PrintDebug("Sensor1 Low", PRINT_DEBOUNCE_DEBUG);
    //}
    //}
  }
  else
  {
    // Sensor Low
    // uint32_t currentTime = millis();
    // if (currentTime - debWheel1LastTime > WHEEL_DEBOUNCE_DELAY)
    //{
    // Sensor Rising Edge
    if (digitalRead(SENSOR_WHEEL_1) == HIGH)
    {
      debWheel1Low = false;
      // debWheel1LastTime = 0;
      wheelSensor1Low = false;
      PrintDebug("Sensor1 Hi", PRINT_DEBOUNCE_DEBUG);
    }
    //}
  }
}
void DebounceWheelSensor2()
{
  // if (debWheel2FallEdge)
  // {
  //   debWheel2LastTime = millis();
  //   debWheel2FallEdge = false;
  // }

  if (!debWheel2Low)
  {
    // if (debWheel2LastTime > 0)
    // {
    //   // High
    if (digitalRead(SENSOR_WHEEL_2))
    {
      // debWheel2LastTime = 0;
      return;
    }

    //   // Falling Edge
    //   uint32_t currentTime = millis();
    //   if (currentTime - debWheel2LastTime > WHEEL_DEBOUNCE_DELAY)
    //   {
    debWheel2Low = true;
    wheelSensor2Low = true;
    PrintDebug("Sensor2 Low", PRINT_DEBOUNCE_DEBUG);
    //   }
    // }
  }
  else
  {
    // Low

    // uint32_t currentTime = millis();
    // if (currentTime - debWheel2LastTime > WHEEL_DEBOUNCE_DELAY)
    // {
    //   // Rising Edge
    if (digitalRead(SENSOR_WHEEL_2) == HIGH)
    {
      debWheel2Low = false;
      debWheel2LastTime = 0;
      wheelSensor2Low = false;
      PrintDebug("Sensor2 Hi", PRINT_DEBOUNCE_DEBUG);
    }
    // }
  }
}

// General
void PrintDebug(String msg, bool print){
  if (print)
  {
    Serial.printf("%s\n", msg.c_str());
  }
}
String getDeviceName(){
  // Read raw 48-bit efuse MAC (never modified)
  uint64_t mac = ESP.getEfuseMac();

  // Extract 6 bytes manually (correct order, no bit masking)
  uint8_t macBytes[6];
  for (int i = 0; i < 6; i++)
  {
    macBytes[i] = (mac >> (8 * (5 - i))) & 0xFF;
  }

  // Format as 12-char hex string (uppercase)
  char macStr[13]; // 12 hex chars + null
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X",
          macBytes[5], macBytes[4], macBytes[3],
          macBytes[2], macBytes[1], macBytes[0]);

  // Build device name
  String deviceName = NAME_PREFIX + "_";
  deviceName += macStr;

  return deviceName;
}
String GetMacAddress(){
  uint64_t mac = ESP.getEfuseMac();
  uint8_t macArr[6];
  for (int i = 0; i < 6; i++)
  {
    macArr[i] = (mac >> (8 * i)) & 0xFF;
  }

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           macArr[5], macArr[4], macArr[3], macArr[2], macArr[1], macArr[0]);

  Serial.println(macStr);
  return macStr;
}
void saveWifiCredentials(String ssid, String password, String ip){
  prefs.begin("wifi", false); // namespace "wifi"
  prefs.putString("ssid", ssid);
  prefs.putString("pw", password);
  prefs.putString("ip", ip);
  prefs.end();
}
String loadSSID(){
  prefs.begin("wifi", true);
  String ssid = prefs.getString("ssid", "");
  prefs.end();
  return ssid;
}
String loadPassword(){
  prefs.begin("wifi", true);
  String pw = prefs.getString("pw", "");
  prefs.end();
  return pw;
}
void writeLCD(String line1, String line2, bool showCnt){
  lcd_i2c.clear();
  lcd_i2c.setCursor(0, 0);
  
  // Insert Measurement Counts
  if(showCnt){
    String cnt = String(" (") + String(measureCount) + ")";
  
    if (line1.length() > 16 - cnt.length())
    {
      line1 = line1.substring(0, 16 - cnt.length());
    }
    else
    {
      while (line1.length() < 16 - cnt.length())
      {
        line1 += " ";
      }
    }
    line1 += cnt;
  }
  
  lcd_i2c.print(line1);

  // Insert Percent
  String percent = String(" ") + String(batteryLevel) + "%";
  if (line2.length() > 16 - percent.length())
  {
    line2 = line2.substring(0, 16 - percent.length());
  }
  else
  {
    while (line2.length() < 16 - percent.length())
    {
      line2 += " ";
    }
  }
  line2 += percent;

  lcd_i2c.setCursor(0, 1);
  lcd_i2c.print(line2);
}
void writeLCD(String line1, String line2){
  lcd_i2c.clear();
  lcd_i2c.setCursor(0, 0); 
  lcd_i2c.print(line1);

  // Insert Percent
  String percent = String(" ") + String(batteryLevel) + "%";
  if (line2.length() > 16 - percent.length())
  {
    line2 = line2.substring(0, 16 - percent.length());
  }
  else
  {
    while (line2.length() < 16 - percent.length())
    {
      line2 += " ";
    }
  }
  line2 += percent;

  lcd_i2c.setCursor(0, 1);
  lcd_i2c.print(line2);
}
bool getKeypad(char *out, size_t outSize) {
  for (int r = 0; r < ROWS; r++) {
    digitalWrite(rowPins[r], LOW);

    for (int c = 0; c < COLS; c++) {
      if (digitalRead(colPins[c]) == LOW) {
        delay(20);
        while (digitalRead(colPins[c]) == LOW);
        digitalWrite(rowPins[r], HIGH);

        const char *src = keys[r][c];
        strncpy(out, src, outSize - 1);
        out[outSize - 1] = '\0';  // always terminate

        if(strcmp(src, "7") == 0) return false; // REMOVE - FAULTY KEYPAD

        return true;
      }
    }
    digitalWrite(rowPins[r], HIGH);
  }
  return false;
}

// MQTT
void mqttRx(char *topic, byte *payload, unsigned int length){

  Serial.print("MQTT RX: ");
  Serial.write(payload, length);
  Serial.println();

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error)
  {
    Serial.print("JSON parse failed: ");
    Serial.println(error.c_str());
    return;
  }

  String _topic = doc[MQTT_JSON_TOPIC];
  String _cmd = doc[MQTT_JSON_CMD];
  toDeviceId = doc[MQTT_JSON_TO_DEVICE_ID].as<String>();
  fromDeviceId = doc[MQTT_JSON_FROM_DEVICE_ID].as<String>();
  JsonVariant _payloadVar = doc[MQTT_JSON_PAYLOAD];
  JsonObject _payloadJson = _payloadVar.as<JsonObject>();

  // Broadcast RX
  if (_topic == MQTT_TOPIC_TO_IOT)
  {
    // isPairing - Device ID Request
    if (_cmd == MQTT_CMD_REQ_MONITOR)
    {
      if (isAdvertising)
      {

        MqttJsonDoc mqttPacket;
        
        mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
        mqttPacket[MQTT_JSON_TO_DEVICE_ID] = fromDeviceId;
        mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
        mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_REQ_MONITOR;

        mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
      }
    }
  }

  // Private RX
  if (_topic == MQTT_TOPIC_TO_IOT + '/' + myDeviceId)
  {
    // Connection Requested
    if (_cmd == MQTT_CMD_CONNECT_MONITOR)
    {
      androidConnected = true;
      connectedDeviceId = fromDeviceId;
      strlcpy((char*)settingsIotType, _payloadJson[SETTING_IOT_TYPE].as<const char *>(), sizeof(settingsIotType));

      // Wheel Data
      if (settingsIotType == IOT_TYPE_WHEEL)
      {
        settingsTicksPerMeter = _payloadJson[SETTING_TICKS_PER_M].as<int32_t>();
        wheelTicksCount = 0;
      }

      // Send Confirmation MQTT
      MqttJsonDoc mqttPacket;

      mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
      mqttPacket[MQTT_JSON_TO_DEVICE_ID] = fromDeviceId;
      mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
      mqttPacket[MQTT_JSON_PAYLOAD] = "";
      mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_CONNECT_MONITOR;

      mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
    }

    // DisConnection Requested
    if (_cmd == MQTT_CMD_DISCONNECT_MONITOR)
    {
      androidConnected = false;
      connectedDeviceId = fromDeviceId;

      // Send Confirmation MQTT
      MqttJsonDoc mqttPacket;
      
      mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
      mqttPacket[MQTT_JSON_TO_DEVICE_ID] = fromDeviceId;
      mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
      mqttPacket[MQTT_JSON_PAYLOAD] = "";
      mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_DISCONNECT_MONITOR;

      mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
    }

    // Monitor Found (ACK)
    if (_cmd == MQTT_CMD_FOUND_MONITOR)
    {
      MqttJsonDoc mqttPacket;

      mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
      mqttPacket[MQTT_JSON_TO_DEVICE_ID] = fromDeviceId;
      mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
      mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_FOUND_MONITOR;

      mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
    }

    // Measure Data (ACK)
    if (_cmd == MQTT_CMD_ACK)
    {
      dataACK = true;
    }
  }
}
void mqttTX(const JsonDocument &msg, const String &topic)
{
  if (!isWifiConnected)
  {
    Serial.println("WiFi not connected");
    return;
  }

  if (!mqttServer.connected())
  {
    Serial.println("MQTT not connected");
    return;
  }

  size_t payloadSize = measureJson(msg);
  char payload[MQTT_PAYLOAD_MAX];
  
  if (payloadSize >= sizeof(payload))
  {
    Serial.printf("MQTT payload too large %i>%i", payloadSize, sizeof(payload));
    return;
  }

  size_t len = serializeJson(msg, payload);
  payload[len] = '\0'; // Null-terminate the string in case buffer overflows
  bool ok = mqttServer.publish(topic.c_str(), payload, len);

  Serial.print("MQTT TX: ");
  Serial.println(payload); // Safe, prints the JSON string
  Serial.printf("MQTT payload: %i/%i \n", payloadSize, sizeof(payload));
    
  if (ok)
  {
    Serial.println("OK");
  }
  else
  {
    Serial.println("MQTT publish failed");
    }
}
void mqttReportIotData(){
  MqttJsonDoc mqttPacket;

  mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
  mqttPacket[MQTT_JSON_TO_DEVICE_ID] = connectedDeviceId;
  mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
  mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_MONITOR_DATA;

  // Payload Json
  JsonObject payload = mqttPacket.createNestedObject(MQTT_JSON_PAYLOAD);
  payload[MQTT_JSON_WHEEL_DISTANCE] = wheelDistance;

  mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
}
void mqttReportMyID(){
  // Allows base station to autosubscribe to private CH
  MqttJsonDoc  mqttPacket;

  mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
  mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
  mqttPacket[MQTT_JSON_PAYLOAD] = "";
  mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_DEVICE_ID;

  mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
}
bool mqttReportMeasurement(int index){
  std::vector<Measure> measures;
  getAllMeasurements(measures);

  //for (int i = 0; i < measures.size(); i++) {
    PrintDebug(
      (String("Push ") + FLASH_MEASURE_DATA + String(index) + ": " + 
      String(JSON_SETTING_OPERATOR) + String(" ") + String(measures[index].nameOperator) + "," +
      String(JSON_SETTING_FOREMAN) + String(" ") + String(measures[index].nameForeman) + "," +
      String(JSON_SETTING_DISTANCE) + String(" ") + String(measures[index].distance) + "," +
      String(JSON_SETTING_LINES) + String(" ") + String(measures[index].lines)) ,
      PRINT_GENERAL_DEBUG);
  //}
    
  MqttJsonDoc mqttPacket;

  mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
  mqttPacket[MQTT_JSON_TO_DEVICE_ID] = connectedDeviceId;
  mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
  mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_MEASURE_DATA;

  // Payload Json
   JsonObject payload = mqttPacket.createNestedObject(MQTT_JSON_PAYLOAD);
   
    payload[JSON_SETTING_OPERATOR] = measures[index].nameOperator;
    payload[JSON_SETTING_FOREMAN]  = measures[index].nameForeman;
    payload[JSON_SETTING_DISTANCE] = measures[index].distance;
    payload[JSON_SETTING_LINES]    = measures[index].lines;
  
  mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
  return true;
}

// Wifi
void wifiScanNetwork(){
  int n = WiFi.scanNetworks();
  Serial.printf("Scan done %i", n);
  for (int i = 0; i < n; i++)
  {
    Serial.println(WiFi.SSID(i));
  }
}
void SendHttpMsg(String msg)
{
  if (wifiClient.connect(serverIP, port))
  {
    wifiClient.print(
        String("GET ") + msg +
        " HTTP/1.1\r\n" +
        "Host: " + serverIP + "\r\n" +
        "Connection: close\r\n" +
        "\r\n");

    Serial.println("MQTT TX: " + msg);
  }
  else
  {
    Serial.println("MQTT TX: Failed");
  }
}
bool wifiCredentialsExist()
{
  prefs.begin("wifi", true);
  bool ok = prefs.isKey("ssid") && prefs.isKey("pw");
  prefs.end();
  return ok;
}

// Bluetooth
class BT_ServerCallbacks : public NimBLEServerCallbacks{
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo)
  {
    PrintDebug("BT Client connected",PRINT_BT_DEBUG);
    isBluetoothConnected = true;
  }

  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason)
  {
    Serial.printf("BT Client disconnected: %s \n", connInfo.getAddress().toString().c_str());
    NimBLEDevice::startAdvertising();
    isBluetoothConnected = false;
  }
};
class BT_HandshakeCallbacks : public NimBLECharacteristicCallbacks{
 void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
  {
    String rxData = pCharacteristic->getValue();
    PrintDebug("BT Rx " + String(rxData), PRINT_BT_DEBUG);

    // Recieved WiFi Credentials
    if (rxData.startsWith(CMD_SHARED_WIFI_CREDENTIALS))
    {
      rxData.remove(0, CMD_SHARED_WIFI_CREDENTIALS.length()); // remove "wificred:"

      int i1 = rxData.indexOf(">");
      int i2 = rxData.indexOf(">", i1 + 1);

      if (i1 != -1 && i2 != -1)
      {
        ssid = rxData.substring(0, i1);
        password = rxData.substring(i1 + 1, i2);

        // Set Global IP address part
        String ip = rxData.substring(i2 + 1);
        ip.toCharArray(serverIP, sizeof(serverIP));

        PrintDebug("SSID: " + ssid, true);
        PrintDebug("Password: " + password, PRINT_CREDENTIALS_DEBUG);

        gotWifiCredentials = true;
      }
      else
      {
        PrintDebug("Invalid WiFi credentials format: " + rxData, PRINT_ERRORS);
        Serial.println();
      }
    }
  }
  void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override
  {
    // Raspberry PI Subscribed to notifications
    Serial.println("\n");
    PrintDebug("BT Client subscribed to notifications", PRINT_BT_DEBUG);
  }
};
void bt_StopServer(){
  if (pServer != nullptr)
  {
    PrintDebug("Stopping existing BLE server...", PRINT_BT_DEBUG);
    NimBLEDevice::stopAdvertising();
    pServer->removeService(pService);
    pService = nullptr;
    pChar = nullptr;
    NimBLEDevice::deinit(true); // fully shuts down BLE
    pServer = nullptr;
    delay(100);
  }
}
void bt_StartServer(){
  bt_StopServer();
  
  NimBLEDevice::init(myDeviceId.c_str());
  NimBLEDevice::setSecurityAuth(false, false, true);

  // Create Server
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new BT_ServerCallbacks());

  // Create Comms Service
  pService = pServer->createService(SERVICE_UUID);
  pChar = pService->createCharacteristic(
      CHAR_UUID,
      NIMBLE_PROPERTY::READ |
          NIMBLE_PROPERTY::WRITE |
          NIMBLE_PROPERTY::NOTIFY);
  pChar->setCallbacks(new BT_HandshakeCallbacks());
  pChar->setValue("IDLE");
  pService->start();

  PrintDebug((String("BT Server Started: ") + myDeviceId), PRINT_BT_DEBUG);
}
void bt_StartAdvertising(){
  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;

  advData.setFlags(0x06); // general discoverable + BR/EDR not supported
  advData.addServiceUUID(SERVICE_UUID);
  pAdv->setAdvertisementData(advData);

  NimBLEAdvertisementData scanResp;
  scanResp.setName(myDeviceId.c_str()); // full name delivered in scan response
  pAdv->setScanResponseData(scanResp);

  NimBLEDevice::startAdvertising();
  PrintDebug((String("BT Advertising Started:  ") + myDeviceId), PRINT_BT_DEBUG);
}

// Flash Read / Write
void saveSettingsToFlash(){
  noInterrupts();

  prefs.begin("iot", false); // read-write
  prefs.putString(SETTING_IOT_TYPE, (char *)settingsIotType);
  prefs.putInt(SETTING_TICKS_PER_M, settingsTicksPerMeter);
  prefs.end();

  interrupts();
}
void loadSettingsFromFlash(){
  noInterrupts();
  prefs.begin("iot", true); // read-only

  String iot = prefs.getString(SETTING_IOT_TYPE, IOT_TYPE_WHEEL);
  strlcpy((char *)settingsIotType, iot.c_str(), sizeof(settingsIotType));

  settingsTicksPerMeter = prefs.getInt(SETTING_TICKS_PER_M, 20);
  PrintDebug(String("From FLASH - IOT Type: ") + settingsIotType, PRINT_GENERAL_DEBUG);

  prefs.end();
  interrupts();
}
void saveMeasureToFlash(Measure &measure){
  const String data = FLASH_MEASURE_DATA + String(measureCount++);

  PrintDebug(String("TO FLASH: ") + 
    FLASH_MEASURE_DATA + String(measureCount) + ","  +
    measure.nameForeman + "," +  
    measure.nameOperator + "," + 
    String(measure.distance) + ","  +  
    String(measure.lines) , 
    PRINT_GENERAL_DEBUG);

  prefs.begin(FLASH_MEASURE_KEY, false);
  prefs.putBytes(data.c_str(), &measure, sizeof(measure));
  prefs.putUInt("count", measureCount);
  prefs.end();
}
bool loadMeasureFromFlash(const char* key, Measure &out){
  prefs.begin(FLASH_MEASURE_KEY, true);

  if (!prefs.isKey(key)) {
    prefs.end();
    return false;
  }

  size_t len = prefs.getBytes(key, &out, sizeof(out));
  if (len != sizeof(out)) {
    prefs.end();
    return false;
  }

  prefs.end();
  return true;
}
void getAllMeasurements(std::vector<Measure> &data){
  data.clear();
  data.reserve(measureCount);

  for(int i = 0; i < measureCount; i++){
    String dataPtr = FLASH_MEASURE_DATA + String(i);
    Measure m;
    
    if(loadMeasureFromFlash(dataPtr.c_str(),m)){
      data.push_back(m);
    }
  }
}
void deleteMeasuresFromFlash(){
  prefs.begin(FLASH_MEASURE_KEY, false);
  prefs.clear();
  prefs.end();
}
void loadMeasureCountFromFlash(){
  prefs.begin(FLASH_MEASURE_KEY, true);

  size_t count = prefs.getUInt("count", 0);
  measureCount = count;
  PrintDebug(String("From FLASH - Measure Count: ") + measureCount, PRINT_GENERAL_DEBUG);
  prefs.end();
}

void saveDistancesToNVM(int value){
  prefs.begin("distances", false);

  uint count = prefs.getUInt("count", 0);

  String key = "n" + String(count);
  prefs.putInt(key.c_str(), value);
  prefs.putUInt("count", count + 1);
  prefs.end();
}
void removeDistancesFromNVM(){
  prefs.begin("distances", false);
  prefs.clear();
  prefs.end();
}
void loadDistancesFromNVM(std::vector<int> &nums){
  prefs.begin("distances", true); // read-only

  size_t count = prefs.getUInt("count", 0);
  nums.clear();

  for (size_t i = 0; i < count; i++)
  {
    String key = "n" + String(i);
    nums.push_back(prefs.getInt(key.c_str(), 0));
  }

  prefs.end();
}

void setup(){
  Serial.begin(9600);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(LED_BLUETOOTH, OUTPUT);
  pinMode(LED_WIFI_CONNECTED, OUTPUT);
  pinMode(LED_MQTT_CONNECTED, OUTPUT);
  pinMode(BUT_PAIR, INPUT_PULLUP);
  pinMode(SENSOR_WHEEL_1, INPUT_PULLUP);
  pinMode(SENSOR_WHEEL_2, INPUT_PULLUP);
  pinMode(KEY_ROW1, OUTPUT);
  pinMode(KEY_ROW2, OUTPUT);
  pinMode(KEY_ROW3, OUTPUT);
  pinMode(KEY_ROW4, OUTPUT);
  pinMode(KEY_COL1, INPUT_PULLUP);
  pinMode(KEY_COL2, INPUT_PULLUP);
  pinMode(KEY_COL3, INPUT_PULLUP);
  pinMode(KEY_COL4, INPUT_PULLUP);

  myDeviceId = getDeviceName();

  // timer 0, prescaler 80 → 1 tick = 1 µs
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimerOneSec, true);

  // 1,000,000 µs = 1 second
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmDisable(timer);

  // Button Interrupt
  attachInterrupt(
      digitalPinToInterrupt(BUT_PAIR),
      buttonISR,
      FALLING);

  // Wheel Sensor 1 Interrupt
  attachInterrupt(
      digitalPinToInterrupt(SENSOR_WHEEL_1),
      wheelSensor1ISR,
      FALLING);

  // Wheel Sensor 2 Interrupt
  attachInterrupt(
      digitalPinToInterrupt(SENSOR_WHEEL_2),
      wheelSensor2ISR,
      FALLING);

  // Button Interrupt
  attachInterrupt(
      digitalPinToInterrupt(BUT_PAIR),
      buttonISR,
      FALLING);

  xTaskCreatePinnedToCore(
      GeneralTask,        // Task function
      "GeneralTask",      // Task name
      3000,              // Stack size (bytes)
      NULL,               // Parameters
      1,                  // Priority
      &GeneralTaskHandle, // Task handle
      1                   // Core 1
  );

  xTaskCreatePinnedToCore(
      wifiConnectTask,        // Task function
      "ConnectWiFiTask",      // Task name
      8000,                   // Stack size (bytes)
      NULL,                   // Parameters
      1,                      // Priority
      &ConnectWiFiTaskHandle, // Task handle
      1                       // Core 1
  );

  xTaskCreatePinnedToCore(
      IotTask,        // Task function
      "IotTask",      // Task name
      8000,          // Stack size (bytes)
      NULL,           // Parameters
      1,              // Priority
      &IotTaskHandle, // Task handle
      0               // Core 1
  );

  // vTaskSuspend(ConnectWiFiTaskHandle);
  // vTaskResume(GeneralTaskHandle);
  // vTaskResume(IotTaskHandle);

  memset(&thisMeasure, 0, sizeof(thisMeasure));

  loadSettingsFromFlash();
  loadMeasureCountFromFlash();

  wifiCasePtr = 0;
  mainCasePtr = 0;

  lcd_i2c.init(); // initialize the lcd
  lcd_i2c.backlight();
}

void loop(){
  vTaskDelay(1 / portTICK_PERIOD_MS);

  switch (mainCasePtr){

  // Splash
  case 0: {
    writeLCD("Wheel " + VERSION, "Connecting...");
    startTimout(5);
    mainCasePtr++;
  }
  break;

  // Splash Screen timout
  case 1:
  {
    if (timeoutFlag)
    {
      timeoutFlag = false;
      mainCasePtr++;
    }
  }
  break;

  // Ready
  case 2:
  {
    writeLCD("Ready", "Press Start", true);
    mainCasePtr++;
  }
  break;

  // Home:
  // - Wait for isPairing Button Press
  // - Wait for Connection request
  case 3: {
    // Keep MQTT Alive
    mqttServer.loop();

    // isPairing
    if (isPairing)
    {
      mainCasePtr = CASE_PAIR;
      break;
    }

    // TX Live Data
    if (androidConnected) {
      if (newIotDataAvailable) {
        newIotDataAvailable = false;
        mqttReportIotData();
      }
    }
        
    // startKeyPressed
    if (startKeyPressed)
    {
      startKeyPressed = false;
      wheelTicksCount = 0;
      wheelDistance = 0;
      mainCasePtr = 20;
      PrintDebug("Start Wheel", PRINT_GENERAL_DEBUG);
    }

    if(upKeyPressed){
      upKeyPressed = false;
    }

    // New Measurement
    if(newMeasurementsPushed){
      newMeasurementsPushed = false;
       writeLCD("Data Pushed", "");
       startTimout(DISPLAY_TIMEOUT);
       mainCasePtr++;
    }

    // Wifi Status
    if (WiFi.status() == WL_CONNECTED){
      isWifiConnected = true;
    }
    else{
      isWifiConnected = false;
    }
  }
  break;

  case 4: {
    // Display Delay - Return Home
    if (timeoutFlag)
    {
      timeoutFlag = false;
      mainCasePtr = 2;
    }
  }
  break;
  
  // (isPairing) Start 
  case 10: {
    // Keep MQTT Alive
    mqttServer.loop();

    if (!isWifiConnected) {
        writeLCD("Pairing ...", "No WiFi");
        startTimout(DISPLAY_TIMEOUT);
        isPairing = false;
        mainCasePtr = 12;
        break;
      }
     
    if (!mqttServer.connected()){
      writeLCD("Pairing ...", "No MQTT");
      startTimout(DISPLAY_TIMEOUT);
      isPairing = false;
      mainCasePtr = 12;
      break;
    }

    writeLCD("Pairing...", "");

    isAdvertising = true;
    androidConnected = false;
    startTimout(PAIR_TIMEOUT);  
    mainCasePtr++;  
  }
  break;

  // (isPairing) Wait:
  // - Monitor Found
  // - Cancel isPairing button
  // - Timeout
  case 11:{
  // Monitor Found (From Android)
    if (androidConnected)
    {
      timerAlarmDisable(timer);
      digitalWrite(LED_MQTT_CONNECTED, HIGH);

      isAdvertising = false;
      isPairing = false;

      PrintDebug("Monitor Found", PRINT_GENERAL_DEBUG);
      writeLCD("Connected!", "");
      startTimout(DISPLAY_TIMEOUT);
      mainCasePtr++;
      break;
    }

    // User Cancelled
    if (btnPairPressed)
    {
      timerAlarmDisable(timer);
      digitalWrite(LED_MQTT_CONNECTED, HIGH);

      btnPairPressed = false;
      isAdvertising = false;
      isPairing = false;
      androidConnected = false;

      PrintDebug("User Cancelled Advertising", PRINT_GENERAL_DEBUG);
      writeLCD("Cancelled", "");
      startTimout(3);
      mainCasePtr++;
      break;
    }

    if (timeoutFlag)
    {
      timeoutFlag = false;
      isAdvertising = false;
      isPairing = false;
      digitalWrite(LED_MQTT_CONNECTED, HIGH);

      writeLCD("Timeout", "");
      startTimout(3);
      mainCasePtr ++;
      break;
    }
  }
  break;

  // (isPairing) Message Delay
  case 12:
  {
    if (timeoutFlag)
    {
      timeoutFlag = false;
      mainCasePtr = CASE_HOME;
    }
  }
  break;
  
  // (Wheel) Start - Present Tag
  case 20:
  {
    isRunning = true;
    nrOfLanesToCut[0] = '\0';
    laneIndex = 0;
    writeLCD("Operator Tag", "");
    mainCasePtr++;
  }
  break;

  // (Wheel) Operator Tag
  case 21:
  {
    if (startKeyPressed)
    {
      strncpy(thisMeasure.nameOperator, "Sipho Khumalo", sizeof(thisMeasure.nameOperator) - 1);
      startKeyPressed = false;
      mainCasePtr++;
    }
  }
  break;

  // (Wheel) Show Operator Name
  case 22:
  {
    writeLCD(thisMeasure.nameOperator, "");
    startTimout(3);
    mainCasePtr++;
  }
  break;

    // (Wheel) Go
  case 23:
  {
    if (timeoutFlag)
    {
      timeoutFlag = false;
      writeLCD("Distance: " + String(wheelDistance) + "m", "");
      mainCasePtr++;
    }
  }
  break;

  // (Wheel) Measure Distance
  case 24:
  {
    if (oldDistance != wheelDistance)
    {
      oldDistance = wheelDistance;
      writeLCD("Distance: " + String(wheelDistance) + "m", "");
    }

    // Stop
    if (stopKeyPressed){
      stopKeyPressed = false;
      writeLCD("Distance: " + String(wheelDistance) + "m", "Stop?");
      mainCasePtr++;
    }
  }
  break;

  // (Wheel) Confirm Cancel or Proceed
  case 25:
  { 
    // Back - Cancel
    if (backKeyPressed){
      backKeyPressed = false;
      mainCasePtr = 23;
    }

    // Enter - Proceed
    else if (enterKeyPressed) {
      enterKeyPressed = false;
      strcpy(nrOfLanesToCut, "");
      writeLCD("Total: " + String(wheelDistance) + "m", "Lanes: ");
      mainCasePtr++;
    }
  }
  break;

  // (Wheel) Enter Lanes to count
  case 26:
  {
    // Back
    if (backKeyPressed){
      backKeyPressed = false;

      if(laneIndex > 0) {
        nrOfLanesToCut[--laneIndex] = '\0';

        writeLCD(
          "Total: " + String(wheelDistance) + "m",
          "Lanes: " + String(nrOfLanesToCut)
        );
      }
      else {
        mainCasePtr = 23;
      }
    }

    // Enter
    else if (enterKeyPressed) {
      enterKeyPressed = false;
      thisMeasure.distance = wheelDistance;  
      thisMeasure.lines = strtol(nrOfLanesToCut, nullptr, 10);
      writeLCD("Foreman Tag", "");
      mainCasePtr++;
    }
    
    // Numbers
    else if ( newNumKeyPressed && laneIndex < sizeof(nrOfLanesToCut) - 1) {
      newNumKeyPressed = false;
      
      if(!(laneIndex == 0 && strcmp(key, KEY_0) == 0)) {
      
        if (laneIndex < sizeof(nrOfLanesToCut) - 1) {
          nrOfLanesToCut[laneIndex++] = key[0];
          nrOfLanesToCut[laneIndex] = '\0';
        }
      }
      
      writeLCD(
        "Total: " + String(wheelDistance) + "m",
        "Lanes: " + String(nrOfLanesToCut)
      );

      key[0] = '\0';  // clear key
    }
  }
  break;

  // (Wheel) Foreman tag
  case 27:
  {
    if (startKeyPressed)
    {
      startKeyPressed = false;
   
      strncpy(thisMeasure.nameForeman, "Marius Radyn", sizeof(thisMeasure.nameForeman) - 1);
      writeLCD(thisMeasure.nameForeman, "");
      startTimout(3);
      mainCasePtr++;
    }
  }
  break;

  // (Wheel) Save
  case 28:
  {
    if(timeoutFlag) {
      timeoutFlag = false;
      
      saveMeasureToFlash(thisMeasure);
      writeLCD(String(wheelDistance) + "m X " + String(nrOfLanesToCut), "SAVED");
      startTimout(3);
      mainCasePtr++;
    }
  }
  break;

  // (Wheel) Saved
  case 29:
  {
    if (timeoutFlag)
    {
      mainCasePtr = 2;
    }
  }
  break;

  default:
    break;
  }
}