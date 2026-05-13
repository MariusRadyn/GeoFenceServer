// ESP32 GeoFence Monitor

#include "FS.h"
#include "pgmspace.h"
#include "time.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <LittleFS.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <OneWire.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <ctime>

const String VERSION = "V1.0";

hw_timer_t *timer = NULL;
volatile bool timeoutFlag = false;
time_t EPOCH_START_OF_TIME = 1700000000; // 2023-11-10 00:00:00
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
#define CASE_LIVE_WHEEL_DATA 40
#define CASE_TAG_PRESENTED 50
#define CASE_DISPLAY_RETURN_HOME 4
#define CASE_HOME 2

#define SUB_CASE_UKNOWN_TAG 0
#define SUB_CASE_WRONG_USER_ACCESS 2
#define SUB_CASE_NONE 255

constexpr const char *IOT_TYPE_VEHICLE = "Vehicle";
constexpr const char *IOT_TYPE_MOBILE_MACHINE = "Mobile Machine";
constexpr const char *IOT_TYPE_STATIONARY_MACHINE = "Stationary Machine";
constexpr const char *IOT_TYPE_WHEEL = "Distance Wheel";

// Flash Settings
constexpr const char *FLASH_SET_TICKS_PER_M = "ticksPerM";
constexpr const char *FLASH_SET_IOT_NAME = "iotName";
constexpr const char *FLASH_SET_IOT_TYPE = "iotType";
constexpr const char *FLASH_SET_USER_DOC_ID = "userDocId";
constexpr const char *FLASH_SET_MONITOR_DOC_ID = "monDocId";
constexpr const char *FLASH_WIFI_SSID = "ssid";
constexpr const char *FLASH_WIFI_PASSWORD = "pw";
constexpr const char *FLASH_WIFI_IP = "ip";

char settingsIotType[32] = {0};
char settingsIotName[32] = {0};
char settingsUserDocId[29] = {0};
char settingsMonDocId[29] = {0};
int32_t settingsTicksPerMeter = 0;

int wifiCasePtr = 0;
int mainCasePtr = 0;
int subCasePtr = 0;
bool isBluetoothConnected = false;
bool isWifiConnected = false;
bool isMqttServiceConnected = false;
bool isRPiConnectingBusy = false;
bool isRPiConnected = false;
bool gotWifiCredentials = false;
bool gotPing = false;
bool gotSync = false;
int wheelTicksCount = 0;

// Commands
const String CMD_SHARED_WIFI_CREDENTIALS =
    "wificred:"; // Format: wificred:ssid#password

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
const String MQTT_CMD_DISCOVER = "#REQ_MONITOR";
const String MQTT_CMD_FOUND_MONITOR = "#FOUND_MONITOR";
const String MQTT_CMD_CONNECT_MONITOR = "#CONNECT_MONITOR";
const String MQTT_CMD_DISCONNECT_MONITOR = "#DISCONNECT_MONITOR";
const String MQTT_CMD_DEVICE_ID = "#DEVICE_ID";
const String MQTT_CMD_ACK = "#ACK";
const String MQTT_CMD_PING = "#PING";
const String MQTT_CMD_TAG_DATA = "#TAG_DATA";
const String MQTT_CMD_LIVE_MONITOR_DATA = "#MONITOR_DATA";
const String MQTT_CMD_IOT_DATA = "#IOT_DATA";
const String MQTT_CMD_SETTINGS = "#REQ_SETTINGS";
const String MQTT_CMD_SYNC = "#SYNC";
const String MQTT_CMD_OPERATORS = "#OPERATORS";

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
const String MQTT_JSON_TAG_DATA = "tag_data";

const String JSON_SET_OPERATOR = "operator";
const String JSON_SET_SUPERVISOR = "supervisor";
const String JSON_SET_TICKS_PER_M = "ticksPerM";
const String JSON_MEASURE_DISTANCE = "distance";
const String JSON_MEASURE_LINES = "lines";
const String JSON_MEASURE_SESSION = "session";
const String JSON_TIMESTAMP = "timestamp";
const String JSON_USER_DOC_ID = "userDocId";
const String JSON_MON_DOC_ID = "monDocId";
const String JSON_IOT_TYPE = "iotType";
const String JSON_IOT_NAME = "iotName";
const String JSON_OPERATORS_VERSION = "operatorsVer";
const String JSON_OPERATORS_LIST = "operatorsList";
const String JSON_OPERATOR_NAME = "name";
const String JSON_OPERATOR_SURNAME = "surname";
const String JSON_OPERATOR_TAG_ID = "tagId";
const String JSON_OPERATOR_ACCESS_LEVEL = "accessLevel";

const char * ACCESS_LEVEL_SUPERVISOR = "supervisor";
const char * ACCESS_LEVEL_OPERATOR = "operator";

const char *FLASH_READINGS_KEY = "readings";
const char *FLASH_READINGS_DATA = "data";
const char *FLASH_SETTINGS = "settings";
const char *FLASH_WIFI_CRED = "wifi";

#define MQTT_PAYLOAD_MAX 512
using MqttJsonDoc =
    StaticJsonDocument<JSON_OBJECT_SIZE(5) +        // root
                       JSON_OBJECT_SIZE(2) +        // payload
                       JSON_ARRAY_SIZE(10) +        // measurements[]
                       (JSON_OBJECT_SIZE(4) * 10) + // Readings structs
                       JSON_MAX_DEVICE_ID + JSON_MAX_CMD + JSON_MAX_TOPIC +
                       80 // JSON key strings
                       >;

// IO
#define BATTERY_VIN 36        // Battery Analog Voltage
#define TAG_READER 13         // Tag Reader
#define TAG_LED 33            // Tag Reader LED
#define BUZZER 32             // Buzzer
#define SENSOR_WHEEL_1 25     // Wheel distance sensor1
#define SENSOR_WHEEL_2 26     // Wheel distance sensor2
#define LED_BLUETOOTH 27      // Blue
#define LED_WIFI_CONNECTED 14 // Red
#define LED_MQTT_CONNECTED 12 // Green

#define SDA_PIN 21
#define SCL_PIN 22

#define KEY_COL1 2
#define KEY_COL2 0
#define KEY_COL3 4
#define KEY_COL4 16
#define KEY_ROW1 17
#define KEY_ROW2 5
#define KEY_ROW3 18
#define KEY_ROW4 19

#define ADC_BATTERY 36

// IOT Readings
#define MAX_MEASURE 100
struct IotData_Wheel {
  char operatorName[17];   // 16 chars + \0
  char supervisorName[17];
  char userDocId[29]; // 28 + \0 
  char monDocId[29]; 
  char gpsCoord[21];  
  float distance;
  int lines;
  char timestamp[15]; // 14 chars + \0
};

IotData_Wheel iotDataWheel [MAX_MEASURE];
IotData_Wheel currentIotDataWheel;
int iotDataCount = 0;
int8_t iotDataIndex = 0;
int session = 0;
bool sessionOpen = false;


// User Tags
struct User {
  uint8_t tag[8];
  char name[30];
  char accessLevel[30];
};

#define MAX_USERS 100
OneWire dsTag(TAG_READER);
byte tagCode[8];
byte lastTagCode[8];
bool tagPresented = false;
bool tagCRCError = false;
User users[MAX_USERS];
uint16_t userCount = 0;
bool maxUsersReached = false;

// LCD
LiquidCrystal_I2C lcd_i2c(0x27, 16,
                          2); // I2C address 0x27, 16 column and 2 rows
char lcdLine1[17] = {0};
char lcdLine2[17] = {0};

// Keypad
const int ROWS = 4;
const int COLS = 4;

int rowPins[ROWS] = {KEY_ROW1, KEY_ROW2, KEY_ROW3, KEY_ROW4};
int colPins[COLS] = {KEY_COL1, KEY_COL2, KEY_COL3, KEY_COL4};

// Connector front: R1,R2,R3,R3,C4,C3,C2,C1
// Pinout:          26,25,33,15,16,04,00,02

const char *keys[ROWS][COLS] = {{"1", "2", "3", "Enter"},
                                {"4", "5", "6", "Back"},
                                {"7", "8", "9", "Up"},
                                {"0", "Start", "Stop", "Down"}};

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
#define PAIR_TIMEOUT 20   // seconds
#define DISPLAY_TIMEOUT 2 // seconds
#define SERVICE_UUID "f3a1c2d0-6b4e-4e9a-9f3e-8d2f1c9b7a1e"
#define CHAR_UUID "c7b2e3f4-1a5d-4c3b-8e2f-9a6b1d8c2f3a"
const String NAME_PREFIX = "iOT_"; // Bluetooth Prefix
NimBLECharacteristic *pChar = nullptr;
NimBLEServer *pServer = nullptr;
NimBLEService *pService = nullptr;

int runningTime = 0; // seconds
bool time_1sec_Flag = false;
String connectedDeviceId;
String myDeviceId;
String fromDeviceId;
String toDeviceId;
bool newIotDataPushed = false;
bool isPushingIotData = false;
bool isPushingIotDataTimeout = false;
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
bool startPairing = false;
bool btConnected = false;
// bool isWifiConnected = false;
bool isPairing = false;
bool androidConnected = false;
bool androidPaired = false;
bool newSettingsRecieved = false;
bool dataACK = false;
bool isRunning = false;
bool iotTypeError = false;
bool getFirstBatReading = true;
bool newBatReadingAvailable = false;
bool buzzerTag = false;
bool showMqttWarning = true;

// Battery
int batteryLevel = 0; // Percentage

// Debug
const bool PRINT_ERRORS = true;
const bool PRINT_GENERAL_DEBUG = true;
const bool PRINT_FLASH_DEBUG = true;
const bool PRINT_CREDENTIALS_DEBUG = true;
const bool PRINT_WIFI_DEBUG = true;
const bool PRINT_BT_DEBUG = true;
const bool PRINT_DEBOUNCE_DEBUG = true;
bool simulateWheelDistance = false;

// LittleFS Filenames
const String OPERATOR_VER_FILE = "/operators_version.bin";
const String OPERATORS_FILE = "/operators.bin";
const String IOTDATA_FILE = "/iotdata.bin";
const String SESSION_NR_FILE = "/session_nr.bin";  // only used when time is not set

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
void lcdWrite(const char *line1, const char *line2);
void lcdWrite(const char *line1, const char *line2, bool showMeasureCnt);
void lcdRefresh();
bool getKeypad(char *out, size_t outSize);
void saveDistancesToNVM(int value);
void saveWifiCredToFlash(String ssid, String password, String ip);
void mqttSendPing();
void readWifiCredFromFlash();
String readOperatorVerFile();
void wrtieOperatorVerToFile(const char *ver);
void writeOperatorsToFile(const char *operators);
void readOperatorsFromFile();
void removeDistancesFromNVM();
void loadDistancesFromNVM(std::vector<int> &nums);
bool readReadingsFromFlash(const char *key, IotData_Wheel &out);
void mqttReportMyID();
bool mqttPushIotData(int);
void getAllReadings(std::vector<IotData_Wheel> &data);
bool wifiCredentialsExist();
String tagToString(byte *addr);
void mqttSendSync();
String GetMacAddress();
String readSessionNrFromFile() ;
void writeSessionNrToFile(const char *session);
void deleteIotDataFile();
void deleteSessionNrFile();
void resetIotData();

//------------------------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------------------------

// Time and Date
void setTime() {
  struct tm timeinfo;
  time_t now;
  const char *ntpServer = "pool.ntp.org";
  const long gmtOffset_sec = 0; // adjust for your timezone
  const int daylightOffset_sec = 0;

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  setenv("TZ", "SAST-2", 1); // South Africa Standard Time
  tzset();

  if (getLocalTime(&timeinfo)) {
    time(&now);
    localtime_r(&now, &timeinfo);

    PrintDebug(String("Time synced: "), PRINT_GENERAL_DEBUG);
    Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
  } else {
    PrintDebug("Time FAILED synced", PRINT_GENERAL_DEBUG);
  }
}
String getTimeDateString() {
  time_t now;
  struct tm timeinfo;

  time(&now);
  localtime_r(&now, &timeinfo);

  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);

  return String(buffer);
}
time_t getTimeStamp() {
  time_t now;
  time(&now);
  
  PrintDebug("Timestamp: " + String(now), PRINT_GENERAL_DEBUG);
  if(now < 1700000000){
    now = EPOCH_START_OF_TIME; // 2023-11-10 00:00:00
  }
  return now;
}

// Timer
void IRAM_ATTR onTimerOneSec() {
  if (runningTime == 0) {
    timeoutFlag = true;
    if(simulateWheelDistance) {
      wheelDistance += 0.5; // Simulate 0.5 meter every second
    }else{
      timerAlarmDisable(timer);
    }
  } else {
    runningTime--;
  }
}
void startTimout(int seconds) {
  runningTime = seconds;
  timeoutFlag = false;
  pairModePressCount = 0;
  timerAlarmEnable(timer);
}

// Interrupts
void IRAM_ATTR wheelSensor1ISR() { 
  debWheel1FallEdge = true; 
}
void IRAM_ATTR wheelSensor2ISR() { 
  debWheel2FallEdge = true; 
}

// Tasks
void GeneralTask(void *parameter) {
  int delay = 0;
  int step = 0;
  bool startup = true;
  int cntAdvertising = 0;
  int tagReadDelay = 0;
  int tagLedDelay = 0;
  int batReadDelay = 0;

  bool ledTagState = LOW;
  bool ledState = LOW;
  bool ledWifiState = LOW;
  bool ledMqttState = LOW;
  bool ledAdvertisingState = LOW;
  bool showStack = true;

  for (;;) {
    // Blink LED
    if (startup) {
      if (delay++ >= 300) {
        delay = 0;

        switch (step) {
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

        if (step > 11) {
          startup = false;
        }
      }
    } else {

      // Blue - Bluetooth
      if (isBluetoothConnected)
        digitalWrite(LED_BLUETOOTH, HIGH); // Blue
      else
        digitalWrite(LED_BLUETOOTH, LOW);

      // Red - Wifi
      if (isWifiConnected)
        digitalWrite(LED_WIFI_CONNECTED, HIGH); // Red
      else
        digitalWrite(LED_WIFI_CONNECTED, LOW);

      // Green
      if (isPairing) {
        if (delay++ >= 700) {
          delay = 0;
          ledMqttState = !ledMqttState;

          if (ledMqttState)
            digitalWrite(LED_MQTT_CONNECTED, HIGH); // Green
          else
            digitalWrite(LED_MQTT_CONNECTED, LOW);
        }
      } else if (!isPairing && isMqttServiceConnected)
        digitalWrite(LED_MQTT_CONNECTED, HIGH);
      // else
      // digitalWrite(LED_MQTT_CONNECTED, LOW);
    }

    // Keypad
    if (getKeypad(key, sizeof(key))) {

      if (showStack) {
        showStack = false;
        Serial.printf("%s Stack free: %u bytes\n", pcTaskGetName(NULL),
                      uxTaskGetStackHighWaterMark(GeneralTaskHandle));
      }

      PrintDebug(String("Key Pressed: ") + key, PRINT_GENERAL_DEBUG);

      // Control
      if (strcmp(key, KEY_START) == 0)
        startKeyPressed = true;
      else if (strcmp(key, KEY_ENTER) == 0)
        enterKeyPressed = true;
      else if (strcmp(key, KEY_BACK) == 0)
        backKeyPressed = true;
      else if (strcmp(key, KEY_UP) == 0)
        upKeyPressed = true;
      else if (strcmp(key, KEY_DOWN) == 0)
        downKeyPressed = true;

      // Stop / Pair (Press STOP x 5)
      else if (strcmp(key, KEY_STOP) == 0) {
        stopKeyPressed = true;

        if (!isPairing && !isRunning) {
          if (pairModePressCount == 0) {
            startTimout(5); // 5 seconds to enter isPairing mode
          }

          if (pairModePressCount++ >= PAIR_MODE_PRESS_COUNT) {
            pairModePressCount = 0;
            PrintDebug("Pairing MODE", PRINT_GENERAL_DEBUG);
            startPairing = true;
            stopKeyPressed = false;
          }
        }
      }
      // Number key
      else
        newNumKeyPressed = true;
    }

    // Tag Reader
    if (tagReadDelay++ >= 250) {
      tagReadDelay = 0;

      if (!dsTag.search(tagCode)) {
        dsTag.reset_search();
      } else {

        // CRC check
        if (OneWire::crc8(tagCode, 7) != tagCode[7]) {
          tagCRCError = true;
          PrintDebug("CRC ERROR", PRINT_GENERAL_DEBUG);
          return;
        }

        // Verify
        if (!tagPresented || memcmp(lastTagCode, tagCode, 8) != 0) {
          tagPresented = true;
          buzzerTag = true;
          memcpy(lastTagCode, tagCode, 8);

          String tag = tagToString(tagCode);
          PrintDebug("Tag Code: " + tag, PRINT_GENERAL_DEBUG);
        }
      }
    }

    // Tag Buzzer
    if (buzzerTag) {
      digitalWrite(BUZZER, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      digitalWrite(BUZZER, LOW);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      digitalWrite(BUZZER, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      digitalWrite(BUZZER, LOW);
      buzzerTag = false;
    }

    // Tag LED Blink
    if (tagLedDelay == 0) {
      if (ledTagState) {
        // Off time
        tagLedDelay = 2000;
        ledTagState = LOW;
        digitalWrite(TAG_LED, LOW);
      } else {
        // On time
        tagLedDelay = 50;
        ledTagState = HIGH;
        digitalWrite(TAG_LED, HIGH);
      }
    } else {
      tagLedDelay--;
    }

    // Read Battery Voltage
    if (batReadDelay++ >= 30000 || getFirstBatReading) {
      // Every 30 seconds or first reading
      getFirstBatReading = false;
      batReadDelay = 0;
      
      int adcValue = analogRead(ADC_BATTERY);
      batteryLevel = map(adcValue, 0, 3800, 0, 100); // Assuming a 12-bit ADC (3800 Max count)
      if (batteryLevel > 100) batteryLevel = 100; // Cap at 100%
      newBatReadingAvailable = true;

      PrintDebug("Battery Level: " + String(adcValue) + " (" +
                     String(batteryLevel) + "%)",
                 PRINT_GENERAL_DEBUG);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
void wifiConnectTask(void *parameter) {
  int retry = 0;
  bool printed = false;
  bool showMsgOnce = true;

  for (;;) {
    if (!printed) {
      PrintDebug("WIFI Task Running", PRINT_WIFI_DEBUG);
      printed = true;
    }

    switch (wifiCasePtr) {
      // Start Bluetooth and Wifi
    case 0: {
      PrintDebug("Starting Bluetooth Connection", PRINT_WIFI_DEBUG);

      isBluetoothConnected = false;
      isWifiConnected = false;
      isMqttServiceConnected = false;
      gotWifiCredentials = false;
      showMsgOnce = true;

      readWifiCredFromFlash();

      bt_StartServer();
      bt_StartAdvertising();
      
      if (!password.isEmpty() && !ssid.isEmpty()) {
        // Use Wifi Connection
        WiFi.begin(ssid, password);
        PrintDebug("Got Credentials, Start Wifi ", PRINT_WIFI_DEBUG);
        wifiCasePtr = 3;
      }else{
        // Use Bluetooth Connection
        startTimout(5);
        wifiCasePtr++;  
      }
    } break;

    // Wait for bluetooth Connection
    case 1: {
      if (isBluetoothConnected) {
        digitalWrite(LED_BLUETOOTH, HIGH); // Solid ON
        PrintDebug("Waiting for Wifi Credentials ... ", PRINT_WIFI_DEBUG);
        wifiCasePtr++;
      }
      if (timeoutFlag) {
        if (WiFi.status() == WL_CONNECTED) {
          PrintDebug("No Bluetooth found. Check Wifi", PRINT_WIFI_DEBUG);
          wifiCasePtr = 3; // Skip Bluetooth if already connected to WiFi
        }
      }
    } break;

    // Wait for Wifi Credentials via Bluetooth
    case 2: {
      if (gotWifiCredentials) {
        // Start WiFi Connection
        isWifiConnected = false;
        isMqttServiceConnected = false;
        wifiCasePtr++;
      }
    } break;

    // Connecting Wifi
    case 3: {
      WiFi.begin(ssid, password);
      wifiCasePtr++;
    } break;

    // Wifi Connected
    case 4: {
      if (WiFi.status() == WL_CONNECTED) {
        String ip = WiFi.localIP().toString().c_str();

        PrintDebug(String("Wifi Connected: ") + WiFi.SSID(), PRINT_WIFI_DEBUG);
        PrintDebug("IP Address: " + ip, PRINT_WIFI_DEBUG);

        isWifiConnected = true;
        digitalWrite(LED_WIFI_CONNECTED, HIGH); // Red

        PrintDebug(String("Start MQTT ... ") + serverIP + ":1883",
                   PRINT_WIFI_DEBUG);
        mqttServer.setServer(serverIP, 1883);
        mqttServer.setCallback(mqttRx);

        setTime();
        wifiCasePtr++;
      }
    } break;

    // Connecting MQTT
    case 5: {
      mqttServer.setBufferSize(512);

      if (mqttServer.connect(myDeviceId.c_str())) {
        PrintDebug("MQTT Connected", PRINT_WIFI_DEBUG);

        mqttServer.loop();
        digitalWrite(LED_MQTT_CONNECTED, HIGH); // Green
        mqttReportMyID();
        showMqttWarning = true;

        wifiCasePtr++;
      } else {
        if(showMqttWarning){
          showMqttWarning = false;
          PrintDebug("MQTT FAILED: " + String(mqttServer.state()),
          PRINT_WIFI_DEBUG);
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
      }
    } break;

    // Subscribe MQTT Topics
    case 6: {
      // Broadcast Subscribe
      mqttServer.subscribe(MQTT_TOPIC_TO_IOT.c_str());
      PrintDebug((String("MQTT Subscribe ...") + MQTT_TOPIC_TO_IOT.c_str()),
                 PRINT_WIFI_DEBUG);

      // Private Subscribe
      MQTT_TOPIC_TO_IOT_PRIVATE = MQTT_TOPIC_TO_IOT + '/' + myDeviceId;
      mqttServer.subscribe(MQTT_TOPIC_TO_IOT_PRIVATE.c_str());
      PrintDebug(
          (String("MQTT Subscribe ...") + MQTT_TOPIC_TO_IOT_PRIVATE.c_str()),
          PRINT_WIFI_DEBUG);

      Serial.printf("%s Stack free: %u bytes\n", pcTaskGetName(NULL),
                    uxTaskGetStackHighWaterMark(ConnectWiFiTaskHandle));
      retry = 3; // Ping Retry
      wifiCasePtr++;
    } break;

    // Ping Base Station
    case 7: {
      mqttSendPing();
      startTimout(2);
      wifiCasePtr++;
    } break;

    // Wait Base Response
    case 8: {
      mqttServer.loop(); // Keep MQTT Alive

      if (gotPing) {
        PrintDebug("PING ACK", PRINT_WIFI_DEBUG);
        gotPing = false;
        isMqttServiceConnected = true;

        mqttSendSync();

        wifiCasePtr = 9; // Connection OK
      } else if (timeoutFlag) {
        PrintDebug("PING TIMEOUT", PRINT_WIFI_DEBUG);
        timeoutFlag = false;
        retry--;

        if (retry <= 0) {
          PrintDebug("Connection Failed after retries", PRINT_WIFI_DEBUG);
          wifiCasePtr = 0; // Restart connection
        } else {
          wifiCasePtr = 7; // Retry ping
        }
      }
      wifiCasePtr++;
    } break;

    // IDLE
    case 9: {
      mqttServer.loop(); // Keep MQTT Alive

      if (WiFi.status() != WL_CONNECTED) {
        PrintDebug("Lost WIFI Connection", PRINT_WIFI_DEBUG);
        wifiCasePtr = 0; // Restart connection
      }

      // Push measurements
      // if(readingsCount > 0 && retry != 0) {
      if (upKeyPressed) {
        upKeyPressed = false;
        //deleteReadingsFromFlash();

        if (iotDataCount > 0) {
          iotDataIndex = iotDataCount - 1;
          retry = 3;
          wifiCasePtr = 20;
        }
      }

      vTaskDelay(500 / portTICK_PERIOD_MS);
    } break;

    // -- Push Measurements ----------------------------------------------
    case 20: {
      mqttServer.loop(); // Keep MQTT Alive

      // Pushing Data
      if (iotDataIndex > -1) {
        PrintDebug(String("Pushing Measurement: ") + String(iotDataIndex), PRINT_WIFI_DEBUG);
        isPushingIotData = true;
        mqttPushIotData(iotDataIndex);
        startTimout(5);
        wifiCasePtr++;
      } 
      
      // Done Pushing Data
      else {
        PrintDebug(String("DELETE FLASH"), PRINT_WIFI_DEBUG);
        deleteIotDataFile();
        isPushingIotData = false;
        newIotDataPushed = true;
        wifiCasePtr = 9;
      }
    } break;

    case 21: {
      // Timeout (No Reply)
      if (timeoutFlag) {
        timeoutFlag = false;
     
        if (retry > 0) {
          // Retry
          retry--;
          wifiCasePtr = 20;
          PrintDebug(String("RETRY"), PRINT_WIFI_DEBUG);
        }
        else{
          // Timeout
          isPushingIotDataTimeout = true;
          PrintDebug(String("TIMEOUT"), PRINT_WIFI_DEBUG);
          wifiCasePtr = 9;  
        }
      }

      // Base ACK
      if (dataACK) {
        dataACK = false;
        iotDataIndex--;
        wifiCasePtr = 20;
      }
    }

    break;
    default:
      break;
    }
  }
}
void IotTask(void *parameter) {
  bool printed = false;
  bool showStack = true;
  int casePtr = 0;

  for (;;) {
    if (!printed) {
      PrintDebug("Iot Task Running", PRINT_GENERAL_DEBUG);
      printed = true;
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);

    // Distance Wheel
    // if (strcmp((const char *)settingsIotType, IOT_TYPE_WHEEL) == 0)
    //{
    DebounceWheelSensor1();
    DebounceWheelSensor2();

    if (showStack) {
      showStack = false;
      Serial.printf("%s Stack free: %u bytes\n", pcTaskGetName(NULL),
                    uxTaskGetStackHighWaterMark(IotTaskHandle));
    }

    // Get Wheel Distance
    switch (casePtr) {
    case 0: {
      // Forward
      if (wheelSensor1Low)
        casePtr = 10;

      // Reverse
      if (wheelSensor2Low)
        casePtr = 20;
    } break;

    // Forward
    case 10: {
      // Forward
      if (!wheelSensor1Low)
        casePtr++;

      // Reverse
      if (wheelSensor2Low)
        casePtr = 0;
    } break;

    case 11: {
      // Forward
      if (!wheelSensor2Low)
        casePtr++;

      // Reverse
      if (wheelSensor1Low)
        casePtr = 0;
    } break;

    case 12:
      if (!wheelSensor2Low) {
        wheelTicksCount++;
        wheelDistance = (double)wheelTicksCount / settingsTicksPerMeter;
        wheelDistance = roundf(wheelDistance * 100.0f) / 100.0f;
        PrintDebug("Forward:" + String(wheelDistance), PRINT_GENERAL_DEBUG);
        casePtr = 0;
      }
      break;

    // Reverse
    case 20: {
      // Reverse
      if (!wheelSensor2Low)
        casePtr++;

      // Forward
      if (wheelSensor1Low)
        casePtr = 0;
    } break;

    case 21: {
      // Reverse
      if (wheelSensor1Low)
        casePtr++;

      // Forward
      if (wheelSensor2Low)
        casePtr = 0;
    } break;

    case 22: {
      if (!wheelSensor1Low) {
        if (wheelTicksCount > 0) {
          wheelTicksCount--;
        }

        wheelDistance = (double)wheelTicksCount / settingsTicksPerMeter;
        PrintDebug("Reverse:" + String(wheelDistance), PRINT_GENERAL_DEBUG);
        casePtr = 0;
      }
    } break;

    default:
      casePtr = 0;
      break;
    }
    //}
  }
}

// Debounce
void DebouncePairBtn() {
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
void DebounceWheelSensor1() {
  // if (debWheel1FallEdge)
  // {
  //   debWheel1LastTime = millis();
  //   debWheel1FallEdge = false;
  // }

  if (!debWheel1Low) {
    // if (debWheel1LastTime > 0)
    //{
    if (digitalRead(SENSOR_WHEEL_1)) {
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
  } else {
    // Sensor Low
    // uint32_t currentTime = millis();
    // if (currentTime - debWheel1LastTime > WHEEL_DEBOUNCE_DELAY)
    //{
    // Sensor Rising Edge
    if (digitalRead(SENSOR_WHEEL_1) == HIGH) {
      debWheel1Low = false;
      // debWheel1LastTime = 0;
      wheelSensor1Low = false;
      PrintDebug("Sensor1 Hi", PRINT_DEBOUNCE_DEBUG);
    }
    //}
  }
}
void DebounceWheelSensor2() {
  // if (debWheel2FallEdge)
  // {
  //   debWheel2LastTime = millis();
  //   debWheel2FallEdge = false;
  // }

  if (!debWheel2Low) {
    // if (debWheel2LastTime > 0)
    // {
    //   // High
    if (digitalRead(SENSOR_WHEEL_2)) {
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
  } else {
    // Low

    // uint32_t currentTime = millis();
    // if (currentTime - debWheel2LastTime > WHEEL_DEBOUNCE_DELAY)
    // {
    //   // Rising Edge
    if (digitalRead(SENSOR_WHEEL_2) == HIGH) {
      debWheel2Low = false;
      debWheel2LastTime = 0;
      wheelSensor2Low = false;
      PrintDebug("Sensor2 Hi", PRINT_DEBOUNCE_DEBUG);
    }
    // }
  }
}

// General
void PrintDebug(String msg, bool print) {
  if (print) {
    Serial.printf("%s\n", msg.c_str());
  }
}
String tagToString(byte *addr) {
  String tag = "";

  for (int i = 0; i < 8; i++) {
    if (addr[i] < 16)
      tag += "0"; // leading zero
    tag += String(addr[i], HEX);
  }

  tag.toUpperCase();
  return tag;
}
bool isHexChar(char c) {
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
         (c >= 'a' && c <= 'f');
}
uint8_t hexCharValue(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return 10 + (c - 'A');
  return 10 + (c - 'a');
}
bool parseTagString(const char *tagStr, uint8_t tagOut[8]) {
  if (!tagStr)
    return false;

  uint8_t byteIndex = 0;
  bool highNibble = true;
  uint8_t current = 0;

  for (const char *p = tagStr; *p && byteIndex < 8; p++) {
    if (!isHexChar(*p))
      continue;

    uint8_t value = hexCharValue(*p);
    if (highNibble) {
      current = value << 4;
      highNibble = false;
    } else {
      current |= value;
      tagOut[byteIndex++] = current;
      highNibble = true;
    }
  }

  return (byteIndex == 8 && highNibble);
}
User getUserNameByTag(uint8_t searchTag[8]) {
    for (int i = 0; i < MAX_USERS; i++) {
        if (memcmp(users[i].tag, searchTag, 8) == 0) {
            return users[i];
        }
    }
    return {}; // not found
}
String getDeviceName() {
 
  String mac = GetMacAddress();
  String deviceName = NAME_PREFIX;
  deviceName += mac;

  return deviceName;
}
String GetMacAddress() {
  uint64_t mac = ESP.getEfuseMac();

  char macStr[13];

  snprintf(
    macStr, 
    sizeof(macStr),
    "%02X%02X%02X%02X%02X%02X",
      (uint8_t)(mac >> 40),
      (uint8_t)(mac >> 32),
      (uint8_t)(mac >> 24),
      (uint8_t)(mac >> 16),
      (uint8_t)(mac >> 8),
      (uint8_t)(mac)
    );

  PrintDebug(macStr, PRINT_GENERAL_DEBUG);
  return String(macStr);
}
String GetMacAddress2() {
  // Read raw 48-bit efuse MAC (never modified)
  uint64_t mac = ESP.getEfuseMac();

  // Extract 6 bytes manually (correct order, no bit masking)
  uint8_t macBytes[6];
  for (int i = 0; i < 6; i++) {
    macBytes[i] = (mac >> (8 * (5 - i))) & 0xFF;
  }

  // Format as 12-char hex string (uppercase)
  char macStr[13]; // 12 hex chars + null
  sprintf(
    macStr, 
    "%02X%02X%02X%02X%02X%02X", 
    macBytes[5], macBytes[4], macBytes[3], macBytes[2], macBytes[1], macBytes[0]);

  PrintDebug(macStr, PRINT_GENERAL_DEBUG);
  return macStr;
}
void lcdWrite(const char* line1, const char* line2, bool showMeasureCnt) {
  char buffer1[17];
  char buffer2[17];
  char percent[6];  // " 100%"
  char measureCount[7];  // " (999)"

  // Format Measurements Count safely
  snprintf(measureCount, sizeof(measureCount), " (%d)", iotDataCount);

  // Format percent safely
  snprintf(percent, sizeof(percent), " %d%%", batteryLevel);

  // ---- LINE 1 ----
  int textWidth = 16 - strlen(measureCount);

  if(showMeasureCnt){
    snprintf(
      buffer1, 
      sizeof(buffer1), 
      "%-*.*s%s", textWidth, textWidth, line1, measureCount
    );
  }else{
    snprintf(
      buffer1, 
      sizeof(buffer1), 
      "%-16.16s", line1
    );
  }
  
  // ---- LINE 2 ----
  textWidth = 16 - strlen(percent);

  snprintf(
    buffer2, 
    sizeof(buffer2),
    "%-*.*s%s", textWidth, textWidth, line2, percent
  );

  // ---- LCD UPDATE ----
  lcd_i2c.setCursor(0, 0);
  lcd_i2c.print(buffer1);

  lcd_i2c.setCursor(0, 1);
  lcd_i2c.print(buffer2);
}
void lcdWrite(const char* line1, const char* line2) {
  char buffer1[17];
  char buffer2[17];
  char percent[6];  // " 100%"
  
  // Format percent safely
  snprintf(percent, sizeof(percent), " %d%%", batteryLevel);

  // ---- LINE 1 ----
  snprintf(
    buffer1, 
    sizeof(buffer1), 
    "%-16.16s", line1
  );
    
  // ---- LINE 2 ----
  int textWidth = 16 - strlen(percent);

  snprintf(
    buffer2, 
    sizeof(buffer2),
    "%-*.*s%s", textWidth, textWidth, line2, percent
  );

  // ---- LCD UPDATE ----
  lcd_i2c.setCursor(0, 0);
  lcd_i2c.print(buffer1);

  lcd_i2c.setCursor(0, 1);
  lcd_i2c.print(buffer2);
}
void lcdWrite(String line1, String line2) {
  lcdWrite(line1.c_str(), line2.c_str());
}
void lcdWrite(String line1, String line2, bool showMeasureCnt) {
  lcdWrite(line1.c_str(), line2.c_str(), showMeasureCnt);
}
void lcdSetLanesPrompt() {
  char laneLine[17];
  snprintf(laneLine, sizeof(laneLine), "Set Lanes: %s", nrOfLanesToCut);
  lcdWrite(laneLine, "(Ent)(Back)");
}
void lcdRefresh() {
  if (lcdLine1[0] == '\0' || lcdLine2[0] == '\0') return;

  char line2[17];  // final output (16 chars + null)

  // Format: text left + percent right-aligned
  snprintf(
    line2, 
    sizeof(line2), 
    "%-11.11s %3d%%", lcdLine2, batteryLevel
  );

  lcd_i2c.setCursor(0, 0);
  lcd_i2c.print("                ");
  lcd_i2c.setCursor(0, 0);
  lcd_i2c.print(lcdLine1);

  lcd_i2c.setCursor(0, 1);
  lcd_i2c.print("                ");
  lcd_i2c.setCursor(0, 1);
  lcd_i2c.print(line2);
}
bool getKeypad(char *out, size_t outSize) {
  for (int r = 0; r < ROWS; r++) {
    digitalWrite(rowPins[r], LOW);

    for (int c = 0; c < COLS; c++) {
      if (digitalRead(colPins[c]) == LOW) {
        delay(20);
        while (digitalRead(colPins[c]) == LOW)
          ;
        digitalWrite(rowPins[r], HIGH);

        const char *src = keys[r][c];
        strncpy(out, src, outSize - 1);
        out[outSize - 1] = '\0'; // always terminate

        return true;
      }
    }
    digitalWrite(rowPins[r], HIGH);
  }
  return false;
}
bool compareString(const char *a, const char *b) {
  if (!a || !b) {
    return false;
  }
  while (*a && *b) {
    if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
      return false;
    }
    a++;
    b++;
  }
  return *a == '\0' && *b == '\0';
}

// MQTT
void mqttSendPing() {
  gotPing = false;
  MqttJsonDoc mqttPacket;

  mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
  mqttPacket[MQTT_JSON_TO_DEVICE_ID] = "";
  mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
  mqttPacket[MQTT_JSON_PAYLOAD] = "";
  mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_PING;

  mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
}
void mqttSendSync() {
  gotSync = false;
  MqttJsonDoc mqttPacket;

  mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
  mqttPacket[MQTT_JSON_TO_DEVICE_ID] = "";
  mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
  mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_SYNC;

  String iotType = settingsIotType[0] ? settingsIotType : "";
  
  if (iotType.isEmpty()) {
    PrintDebug("IOT Type is empty, cannot send SYNC", PRINT_ERRORS);
    return;
  }

  // Distance Wheel
  // Sync Operarators
  if (iotType == IOT_TYPE_WHEEL) {
    JsonObject payload = mqttPacket.createNestedObject(MQTT_JSON_PAYLOAD);
    String operateVer = readOperatorVerFile();
    
    payload[JSON_OPERATORS_VERSION] = operateVer;
    payload[JSON_IOT_TYPE] = settingsIotType;

    mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
    return;
  }
  else{  
    PrintDebug("Sync Error: Unknown IOT Type: " + iotType, PRINT_ERRORS);
  }
}
void mqttRx(char *topic, byte *payload, unsigned int length) {
  Serial.print("MQTT RX: ");
  Serial.write(payload, length);
  Serial.println();

  StaticJsonDocument<2048> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
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

  // -------------------------------------------
  // Broadcast RX
  // -------------------------------------------
  if (_topic == MQTT_TOPIC_TO_IOT) {
    // Pair - Device ID Request
    if (_cmd == MQTT_CMD_DISCOVER) {
      if (isPairing) {
        // IOT Type
        const char *iotType = _payloadJson[JSON_IOT_TYPE].as<const char *>();
        if (!iotType)
          return;

        if (strcmp(iotType, IOT_TYPE_WHEEL) != 0) {
          iotTypeError = true;
          PrintDebug(String("IotType (Distance Wheel) Mismatch: ") +
                         String(iotType),
                     PRINT_GENERAL_DEBUG);
          return;
        }

        MqttJsonDoc mqttPacket;

        mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
        mqttPacket[MQTT_JSON_TO_DEVICE_ID] = fromDeviceId;
        mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
        mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_DISCOVER;

        mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
      }
    }
  }

  // -------------------------------------------
  // Private RX
  // -------------------------------------------
  if (_topic == MQTT_TOPIC_TO_IOT + '/' + myDeviceId) {
    // Live Connection Requested
    if (_cmd == MQTT_CMD_CONNECT_MONITOR) {
      androidConnected = true;
      connectedDeviceId = fromDeviceId;
      wheelTicksCount = 0;

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
    if (_cmd == MQTT_CMD_DISCONNECT_MONITOR) {
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

    // Pair DONE (ACK) (Save Settings to FLASH)
    if (_cmd == MQTT_CMD_FOUND_MONITOR) {
      androidPaired = true;

      // iotType
      const char *iotType = _payloadJson[JSON_IOT_TYPE].as<const char *>();
      if (iotType)
        strlcpy(settingsIotType, iotType, sizeof(settingsIotType));
      else
        settingsIotType[0] = '\n';

      // Ticks per Meter
      settingsTicksPerMeter = _payloadJson[JSON_SET_TICKS_PER_M].as<int32_t>();

      // IOT Name
      const char *iotName = _payloadJson[JSON_IOT_NAME].as<const char *>();
      if (iotName)
        strlcpy(settingsIotName, iotName, sizeof(settingsIotName));
      else
        settingsIotName[0] = '\n';

      // IOT Monitor Doc ID
      const char *monId = _payloadJson[JSON_MON_DOC_ID];
      if (monId) {
        strncpy(settingsMonDocId, monId, sizeof(settingsMonDocId) - 1);
        settingsMonDocId[sizeof(settingsMonDocId) - 1] =
            '\0'; // ensure null termination
      }

      // User Doc ID
      const char *userId = _payloadJson[JSON_USER_DOC_ID];
      if (userId) {
        strncpy(settingsUserDocId, userId, sizeof(settingsUserDocId) - 1);
        settingsUserDocId[sizeof(settingsUserDocId) - 1] =
            '\0'; // ensure null termination
      }

      MqttJsonDoc mqttPacket;

      mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
      mqttPacket[MQTT_JSON_TO_DEVICE_ID] = fromDeviceId;
      mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
      mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_FOUND_MONITOR;

      mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
      newSettingsRecieved = true;
    }

    // Readings Data (ACK)
    if (_cmd == MQTT_CMD_ACK) {
      dataACK = true;
    }

    // PING
    if (_cmd == MQTT_CMD_PING) {
      gotPing = true;
    }

    // Settings (ADD LIST OF BASE STATIONS ALLOWED)
    if (_cmd == MQTT_CMD_SETTINGS) {
      // TODO: Add List of Base Stations allowed.
      // SAVE to flash
      // SEND ACK
    }

    // Operators
    if (_cmd == MQTT_CMD_OPERATORS) {

      const char *ver = _payloadJson[JSON_OPERATORS_VERSION].as<const char *>();
      JsonArray operatorsArray = _payloadJson[JSON_OPERATORS_LIST];
      String operatorsStr;
      
      if (!operatorsArray.isNull()) {
        serializeJson(operatorsArray, operatorsStr);
      }

      const char *operators = operatorsStr.c_str();
      
      // Operator Version
      if (ver) {
        wrtieOperatorVerToFile(ver);
      }
      
      // Operator List
      if (operators && strlen(operators) > 0) {
        writeOperatorsToFile(operators);
        readOperatorsFromFile();
      }
    }
  }
}
void mqttTX(const JsonDocument &msg, const String &topic) {
  if (!isWifiConnected) {
    Serial.println("WiFi not connected");
    return;
  }

  if (!mqttServer.connected()) {
    Serial.println("MQTT not connected");
    return;
  }

  size_t payloadSize = measureJson(msg);
  char payload[MQTT_PAYLOAD_MAX];

  if (payloadSize >= sizeof(payload)) {
    Serial.printf("MQTT payload too large %i>%i", payloadSize, sizeof(payload));
    return;
  }

  size_t len = serializeJson(msg, payload);
  payload[len] = '\0'; // Null-terminate the string in case buffer overflows
  bool ok = mqttServer.publish(topic.c_str(), payload, len);

  Serial.print("MQTT TX: ");
  Serial.println(payload); // Safe, prints the JSON string
  Serial.printf("MQTT payload: %i/%i \n", payloadSize, sizeof(payload));

  if (ok) {
    Serial.println("OK");
  } else {
    Serial.println("MQTT publish failed");
  }
}
void mqttReportLiveIotData() {
  MqttJsonDoc mqttPacket;

  mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
  mqttPacket[MQTT_JSON_TO_DEVICE_ID] = connectedDeviceId;
  mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
  mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_LIVE_MONITOR_DATA;

  // Payload Json
  JsonObject payload = mqttPacket.createNestedObject(MQTT_JSON_PAYLOAD);
  payload[MQTT_JSON_WHEEL_DISTANCE] = wheelDistance;

  mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
}
void mqttReportTag() {
  MqttJsonDoc mqttPacket;

  mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
  mqttPacket[MQTT_JSON_TO_DEVICE_ID] = connectedDeviceId;
  mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
  mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_TAG_DATA;

  // Payload Json
  JsonObject payload = mqttPacket.createNestedObject(MQTT_JSON_PAYLOAD);
  String tag = tagToString(tagCode);
  payload[MQTT_JSON_TAG_DATA] = tag;

  mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
}
void mqttReportMyID() {
  // Allows base station to autosubscribe to private CH
  MqttJsonDoc mqttPacket;

  mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
  mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
  mqttPacket[MQTT_JSON_PAYLOAD] = "";
  mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_DEVICE_ID;

  mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
}
bool mqttPushIotData(int index) {
  //std::vector<Measurement> measurements;
  //getAllReadings(readings);

  MqttJsonDoc mqttPacket;
  mqttPacket[MQTT_JSON_FROM_DEVICE_ID] = myDeviceId;
  mqttPacket[MQTT_JSON_TO_DEVICE_ID] = connectedDeviceId;
  mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
  mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_IOT_DATA;

  // Payload Json
  JsonObject payload = mqttPacket.createNestedObject(MQTT_JSON_PAYLOAD);

  payload[JSON_MEASURE_DISTANCE] = roundf(iotDataWheel[index].distance * 1000.0f) / 1000.0f;
  payload[JSON_SET_OPERATOR] = iotDataWheel[index].operatorName;
  payload[JSON_SET_SUPERVISOR] = iotDataWheel[index].supervisorName;
  payload[JSON_MEASURE_LINES] = iotDataWheel[index].lines;
  payload[JSON_TIMESTAMP] = iotDataWheel[index].timestamp;

  payload[JSON_USER_DOC_ID] = settingsUserDocId;
  payload[JSON_MON_DOC_ID] = settingsMonDocId;
  payload[JSON_IOT_TYPE] = settingsIotType;
  payload[JSON_IOT_NAME] = settingsIotName;
  
  mqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
  return true;
}

// Wifi
void wifiScanNetwork() {
  int n = WiFi.scanNetworks();
  Serial.printf("Scan done %i", n);
  for (int i = 0; i < n; i++) {
    Serial.println(WiFi.SSID(i));
  }
}
void SendHttpMsg(String msg) {
  if (wifiClient.connect(serverIP, port)) {
    wifiClient.print(String("GET ") + msg + " HTTP/1.1\r\n" + "Host: " +
                     serverIP + "\r\n" + "Connection: close\r\n" + "\r\n");

    Serial.println("MQTT TX: " + msg);
  } else {
    Serial.println("MQTT TX: Failed");
  }
}
bool wifiCredentialsExist() {
  prefs.begin("wifi", true);
  bool ok = prefs.isKey("ssid") && prefs.isKey("pw");
  prefs.end();
  return ok;
}

// Bluetooth
class BT_ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) {
    PrintDebug("BT Client connected", PRINT_BT_DEBUG);
    isBluetoothConnected = true;
  }

  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo,
                    int reason) {
    Serial.printf("BT Client disconnected: %s \n",
                  connInfo.getAddress().toString().c_str());
    NimBLEDevice::startAdvertising();
    isBluetoothConnected = false;
  }
};
class BT_HandshakeCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic,
               NimBLEConnInfo &connInfo) {
    String rxData = pCharacteristic->getValue();
    PrintDebug("BT Rx " + String(rxData), PRINT_BT_DEBUG);

    // Recieved WiFi Credentials
    if (rxData.startsWith(CMD_SHARED_WIFI_CREDENTIALS)) {
      rxData.remove(0,
                    CMD_SHARED_WIFI_CREDENTIALS.length()); // remove "wificred:"

      int i1 = rxData.indexOf(">");
      int i2 = rxData.indexOf(">", i1 + 1);

      if (i1 != -1 && i2 != -1) {
        String newSsid = rxData.substring(0, i1);
        String newPassword = rxData.substring(i1 + 1, i2);
        char newIP[16] = "";
        bool newIPMatch = false;

        // Set Global IP address part
        String ip = rxData.substring(i2 + 1);
        ip.toCharArray(newIP, sizeof(serverIP));
        strcmp(serverIP, newIP) == 0 ? newIPMatch = true : newIPMatch = false;

        if (password != newPassword || ssid != newSsid || !newIPMatch) {
          saveWifiCredToFlash(newSsid, newPassword, newIP);
          ssid = newSsid;
          password = newPassword;
          strlcpy(serverIP, newIP, sizeof(serverIP));
          PrintDebug("Save Wifi Credentials", PRINT_WIFI_DEBUG);
        } else {
          PrintDebug("WiFi Credentials did not change", PRINT_WIFI_DEBUG);
        }

        PrintDebug("SSID: " + ssid, true);
        // PrintDebug("Password: " + password, PRINT_CREDENTIALS_DEBUG);

        gotWifiCredentials = true;
      } else {
        PrintDebug("Invalid WiFi credentials format: " + rxData, PRINT_ERRORS);
        Serial.println();
      }
    }
  }
  void onSubscribe(NimBLECharacteristic *pCharacteristic,
                   NimBLEConnInfo &connInfo, uint16_t subValue) override {
    // Raspberry PI Subscribed to notifications
    Serial.println("\n");
    PrintDebug("BT Client subscribed to notifications", PRINT_BT_DEBUG);
  }
};
void bt_StopServer() {
  if (pServer != nullptr) {
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
void bt_StartServer() {
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
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pChar->setCallbacks(new BT_HandshakeCallbacks());
  pChar->setValue("IDLE");
  pService->start();

  PrintDebug((String("BT Server Started: ") + myDeviceId), PRINT_BT_DEBUG);
}
void bt_StartAdvertising() {
  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;

  advData.setFlags(0x06); // general discoverable + BR/EDR not supported
  advData.addServiceUUID(SERVICE_UUID);
  pAdv->setAdvertisementData(advData);

  NimBLEAdvertisementData scanResp;
  scanResp.setName(myDeviceId.c_str()); // full name delivered in scan response
  pAdv->setScanResponseData(scanResp);

  NimBLEDevice::startAdvertising();
  PrintDebug((String("BT Advertising Started:  ") + myDeviceId),
             PRINT_BT_DEBUG);
}

// Read/Write Prevs
void saveSettingsToFlash() {
  char _iotType[32];
  char _iotName[32];
  char _uid[32];
  char _monid[32];
  int _ticks = 0;

  prefs.begin(FLASH_SETTINGS, false); // read-write

  prefs.getString(FLASH_SET_IOT_NAME, _iotName, sizeof(settingsIotName));
  prefs.getString(FLASH_SET_IOT_TYPE, _iotType, sizeof(settingsIotType));
  prefs.getString(FLASH_SET_MONITOR_DOC_ID, _monid, sizeof(settingsMonDocId));
  prefs.getString(FLASH_SET_USER_DOC_ID, _uid, sizeof(settingsUserDocId));
  prefs.getInt(FLASH_SET_TICKS_PER_M, _ticks);

  // Save New
  if (strcmp(settingsMonDocId, _monid) != 0) {
    prefs.putString(FLASH_SET_MONITOR_DOC_ID, (char *)settingsMonDocId);
  }
  if (strcmp(settingsUserDocId, _uid) != 0) {
    prefs.putString(FLASH_SET_USER_DOC_ID, (char *)settingsUserDocId);
  }
  if (strcmp(settingsIotType, _iotType) != 0) {
    prefs.putString(FLASH_SET_IOT_TYPE, (char *)settingsIotType);
  }
  if (strcmp(settingsIotName, _iotName) != 0) {
    prefs.putString(FLASH_SET_IOT_NAME, (char *)settingsIotName);
  }
  if (settingsTicksPerMeter != _ticks) {
    prefs.putInt(FLASH_SET_TICKS_PER_M, settingsTicksPerMeter);
  }

  prefs.end();

  PrintDebug(String("FLASH: ") + FLASH_SET_IOT_NAME + String(": ") +
                 settingsIotName + ", " + FLASH_SET_IOT_TYPE + String(": ") +
                 settingsIotType + ", " + FLASH_SET_USER_DOC_ID + String(": ") +
                 settingsUserDocId + ", " + FLASH_SET_MONITOR_DOC_ID +
                 String(": ") + settingsMonDocId + ", " +
                 FLASH_SET_TICKS_PER_M + String(": ") + settingsTicksPerMeter,
             PRINT_FLASH_DEBUG);
}
void readSettingsFromFlash() {
  prefs.begin(FLASH_SETTINGS, true); // read-only

  prefs.getString(FLASH_SET_IOT_NAME, settingsIotName, sizeof(settingsIotName));
  prefs.getString(FLASH_SET_IOT_TYPE, settingsIotType, sizeof(settingsIotType));
  prefs.getString(FLASH_SET_MONITOR_DOC_ID, settingsMonDocId,
                  sizeof(settingsMonDocId));
  prefs.getString(FLASH_SET_USER_DOC_ID, settingsUserDocId,
                  sizeof(settingsUserDocId));
  settingsTicksPerMeter = prefs.getInt(FLASH_SET_TICKS_PER_M, 20);

  PrintDebug(String("From FLASH -  ") + FLASH_SET_IOT_NAME + String(": ") +
                 settingsIotName + ", " + FLASH_SET_IOT_TYPE + String(": ") +
                 settingsIotType + ", " + FLASH_SET_USER_DOC_ID + String(": ") +
                 settingsUserDocId + ", " + FLASH_SET_MONITOR_DOC_ID +
                 String(": ") + settingsMonDocId,
             PRINT_FLASH_DEBUG);

  prefs.end();
}
void saveWifiCredToFlash(String ssid, String password, String ip) {
  PrintDebug("Writing WIFI Cred", PRINT_FLASH_DEBUG);

  prefs.begin(FLASH_WIFI_CRED, false);
  prefs.putString(FLASH_WIFI_SSID, ssid.c_str());
  prefs.putString(FLASH_WIFI_PASSWORD, password.c_str());
  prefs.putString(FLASH_WIFI_IP, ip.c_str());
  prefs.end();

  PrintDebug(String("Save WIFI Cred to FLASH -  ") + FLASH_WIFI_SSID +
                 String(": ") + ssid + ", " + FLASH_WIFI_PASSWORD +
                 String(": ") + password + ", " + FLASH_WIFI_IP + String(": ") +
                 ip,
             PRINT_FLASH_DEBUG);
}
void readWifiCredFromFlash() {
  try {
    PrintDebug("Reading WIFI Cred", PRINT_FLASH_DEBUG);

    prefs.begin(FLASH_WIFI_CRED, true);
    String newSsid = prefs.getString(FLASH_WIFI_SSID, "");
    String newPassword = prefs.getString(FLASH_WIFI_PASSWORD, "");
    String ip = prefs.getString(FLASH_WIFI_IP, "");

    if (!newPassword.isEmpty()) {
      password = newPassword;
    }
    if (!newSsid.isEmpty()) {
      ssid = newSsid;
    }

    if (ip.length() > 0 && ip.length() < sizeof(serverIP)) {
      memset(serverIP, 0, sizeof(serverIP)); // clear old data
      ip.toCharArray(serverIP, sizeof(serverIP));
    }

    gotWifiCredentials = true;
    PrintDebug(
      String("WIFI Cred (FLASH) -  ") + FLASH_WIFI_SSID + String(": ") + newSsid + ", " + FLASH_WIFI_PASSWORD +
                   String(": ") + newPassword + ", " + FLASH_WIFI_IP +
                   String(": ") + ip,
      PRINT_FLASH_DEBUG);

    prefs.end();
  } catch (const std::exception &e) {
    PrintDebug(String("readWifiCredFromFlash(): ") + e.what(), PRINT_ERRORS);
    return;
  }
}

// Read/Write File (Session Number)
void writeSessionNrToFile(const char *session ) {
  File f = LittleFS.open(SESSION_NR_FILE, "w");
  f.print(session);
  f.close();
  PrintDebug("Saved Session Nr to file:" + String(session), PRINT_FLASH_DEBUG);
}
String readSessionNrFromFile() {
  if (LittleFS.exists(SESSION_NR_FILE)) {
    File f = LittleFS.open(SESSION_NR_FILE, "r");
    String session = "0";

    if (f.available()) {
      session = f.readString();
    }

    f.close();
    PrintDebug("Read Session Nr from file:" + String(session), PRINT_FLASH_DEBUG);
    return session;
  }

  return "0";
}
void deleteSessionNrFile() {
  if (LittleFS.exists(SESSION_NR_FILE)) {
    LittleFS.remove(SESSION_NR_FILE);
    PrintDebug("Deleted Session Nr file", PRINT_FLASH_DEBUG);
  }
}

// Read/Write File (Operators)
void writeOperatorsToFile(const char *operators) {

  if (operators && strlen(operators) > 0) {
    DynamicJsonDocument doc(strlen(operators) + 256);
    DeserializationError err = deserializeJson(doc, operators);
    if (err) {
      PrintDebug(String("Invalid operators JSON: ") + err.c_str(),
                 PRINT_ERRORS);
      return;
    }

    JsonArray operatorsArray = doc.as<JsonArray>();
    if (operatorsArray.isNull() && doc.is<JsonObject>()) {
      operatorsArray = doc[JSON_OPERATORS_LIST].as<JsonArray>();
    }
    if (operatorsArray.isNull()) {
      PrintDebug("Operators JSON is not an array", PRINT_ERRORS);
      return;
    }

    userCount = 0;
    for (JsonVariant v : operatorsArray) {
      if (userCount >= MAX_USERS) {
        maxUsersReached = true;
        break;
      }

      String s_fullName;
      String s_access;
      const char *tag = nullptr;

      if (v.is<JsonObject>()) {    
        JsonObject opObj = v.as<JsonObject>();

        const char *firstName = opObj[JSON_OPERATOR_NAME];
        const char *surname = opObj[JSON_OPERATOR_SURNAME];
        const char *access = opObj[JSON_OPERATOR_ACCESS_LEVEL];
        tag = opObj[JSON_OPERATOR_TAG_ID];
        
        if (firstName) {
          s_fullName = String(firstName);

          if (surname && strlen(surname) > 0) {
            s_fullName += " ";
            s_fullName += surname;
          }

          if(strlen(s_fullName.c_str()) > 29) {
            s_fullName = s_fullName.substring(0, 29);
          }
        }

        if (access) {
          s_access = String(access);

          if(s_access.length() > 29) {
            s_access = s_access.substring(0, 29);
          }
        }
      } 
      else if (v.is<const char *>()) {
        tag = v.as<const char *>();
      }

      if (!tag)
        continue;

      if (!parseTagString(tag, users[userCount].tag)) {
        PrintDebug(String("Invalid operator tag: ") + tag, PRINT_ERRORS);
        continue;
      }

      if (s_fullName.length() > 0) {
        strncpy(users[userCount].name, s_fullName.c_str(), sizeof(users[userCount].name) - 1);
        users[userCount].name[sizeof(users[userCount].name) - 1] = '\0';
      } 
      else {
        users[userCount].name[0] = '\0';
      }

      if (s_access.length() > 0) {
        strncpy(users[userCount].accessLevel, s_access.c_str(), sizeof(users[userCount].accessLevel) - 1);
        users[userCount].accessLevel[sizeof(users[userCount].accessLevel) - 1] = '\0';
      } 
      else {
        users[userCount].accessLevel[0] = '\0';
      }
      userCount++;
    }
  }

  File f = LittleFS.open(OPERATORS_FILE, "w");
  for (uint16_t i = 0; i < userCount; i++) {
    f.write((uint8_t *)&users[i], sizeof(User));
  }
  f.close();

  PrintDebug(String("Saved Operators to file: ") + String(userCount) + " users",
             PRINT_GENERAL_DEBUG);
  
}
void readOperatorsFromFile() {
  try {
    if (!LittleFS.exists(OPERATORS_FILE)) {
      PrintDebug("Operators file does not exist", PRINT_ERRORS);
      return;
    }

    File f = LittleFS.open(OPERATORS_FILE, "r");
    userCount = 0;

    while (f.available() && userCount < MAX_USERS) {
      f.read((uint8_t *)&users[userCount], sizeof(User));
      userCount++;
    }

    f.close();

    if (PRINT_FLASH_DEBUG) {
      printf("Read Operators(%i) from file\n", userCount);

      for (int i = 0; i < userCount; i++) {
        printf("User %i: %s, Tag: %s, Access: %s\n", 
          i, 
          users[i].name, 
          tagToString(users[i].tag).c_str(), 
          users[i].accessLevel
        );
      }
    }

  } catch (const std::exception &e) {
    PrintDebug(String("readOperatorsFromFile(): ") + e.what(), PRINT_ERRORS);
    return;
  }
}
void wrtieOperatorVerToFile(const char *ver) {
  File f = LittleFS.open(OPERATOR_VER_FILE, "w");
  f.print(ver);
  f.close();
  printf("Saved Operator Version to file: %s \n", ver);
}
String readOperatorVerFile() {
  if (LittleFS.exists(OPERATOR_VER_FILE)) {
    File f = LittleFS.open(OPERATOR_VER_FILE, "r");
    String ver = "0";

    if (f.available()) {
      ver = f.readString();
    }

    f.close();
    printf("Read Operator Version from file: %s \n", ver.c_str());
    return ver;
  }

  return "0";
}

// Read / Write File (IotData)
void writeIotDataToFile(IotData_Wheel iotdata) {
  Serial.println("writeIotDataToFile");

  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
    return;
  }

  File file = LittleFS.open(IOTDATA_FILE, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  const size_t written = file.write(reinterpret_cast<const uint8_t *>(&iotdata), sizeof(iotdata));
  
  if (written == sizeof(iotdata)) {
    Serial.printf("Measure written successfully");
    if (iotDataCount < MAX_MEASURE) {
      iotDataWheel[iotDataCount++] = iotdata;
    }
  } else {
    Serial.println("Measure write failed");
  }

  file.close();
}
void readIotDataFile() {
  resetIotData();

  if(!LittleFS.exists(IOTDATA_FILE)) {
      return;
  }

  File file = LittleFS.open(IOTDATA_FILE, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file");
    return;
  }

  while (file.available() && iotDataCount < MAX_MEASURE) {
    const size_t readLen =
        file.read(reinterpret_cast<uint8_t *>(&iotDataWheel[iotDataCount]),
                  sizeof(IotData_Wheel));
    
    if (readLen != sizeof(IotData_Wheel)) {
      break;
    }
    iotDataCount++;
  }

  file.close();

  if (PRINT_FLASH_DEBUG) {
    Serial.printf("Loaded %u iotData\n", static_cast<unsigned int>(iotDataCount));

    for (int i = 0; i < iotDataCount; i++) {
      printf("IOT%i: Operator: %s, Supervisor: %s, UserId: %s, MonId: %s, GPS: %s, Distance: %f, Lines: %i, Timestamp: %s\n", 
        i, 
        iotDataWheel[i].operatorName, 
        iotDataWheel[i].supervisorName, 
        iotDataWheel[i].userDocId, 
        iotDataWheel[i].monDocId, 
        iotDataWheel[i].gpsCoord, 
        iotDataWheel[i].distance, 
        iotDataWheel[i].lines,
        iotDataWheel[i].timestamp
        
      );
    }
  }
}
void deleteIotDataFile() {
  if (LittleFS.exists(IOTDATA_FILE)) {
    LittleFS.remove(IOTDATA_FILE);
    PrintDebug("Deleted IOT Data file", PRINT_FLASH_DEBUG);
  }
  
  resetIotData();
}
void resetIotData() {
  iotDataCount = 0;
  iotDataIndex = -1;
  memset(iotDataWheel, 0, sizeof(iotDataWheel));
}

void setup() {
  Serial.begin(9600);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(LED_BLUETOOTH, OUTPUT);
  pinMode(LED_WIFI_CONNECTED, OUTPUT);
  pinMode(LED_MQTT_CONNECTED, OUTPUT);
  pinMode(BATTERY_VIN, INPUT_PULLUP);
  pinMode(TAG_READER, INPUT);
  pinMode(TAG_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(SENSOR_WHEEL_1, INPUT);
  pinMode(SENSOR_WHEEL_2, INPUT);
  pinMode(KEY_ROW1, OUTPUT);
  pinMode(KEY_ROW2, OUTPUT);
  pinMode(KEY_ROW3, OUTPUT);
  pinMode(KEY_ROW4, OUTPUT);
  pinMode(KEY_COL1, INPUT_PULLUP);
  pinMode(KEY_COL2, INPUT_PULLUP);
  pinMode(KEY_COL3, INPUT_PULLUP);
  pinMode(KEY_COL4, INPUT_PULLUP);

  // Until board is fixed
  pinMode(39, INPUT);
  pinMode(35, INPUT);
  pinMode(34, INPUT);

  digitalWrite(BUZZER, LOW);
  digitalWrite(TAG_LED, HIGH);

  myDeviceId = getDeviceName();

  // timer 0, prescaler 80 → 1 tick = 1 µs
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimerOneSec, true);

  // 1,000,000 µs = 1 second
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmDisable(timer);

  // Wheel Sensor 1 Interrupt
  attachInterrupt(digitalPinToInterrupt(SENSOR_WHEEL_1), wheelSensor1ISR,
                  FALLING);

  // Wheel Sensor 2 Interrupt
  attachInterrupt(digitalPinToInterrupt(SENSOR_WHEEL_2), wheelSensor2ISR,
                  FALLING);

  xTaskCreatePinnedToCore(GeneralTask, 
    "GeneralTask",      // Task name
    3000,               // Stack size (bytes)
    NULL,               // Parameters
    1,                  // Priority
    &GeneralTaskHandle, // Task handle
    1                   // Core 1
  );

  xTaskCreatePinnedToCore(wifiConnectTask,
    "ConnectWiFiTask",      // Task name
    7000,                   // Stack size (bytes)
    NULL,                   // Parameters
    1,                      // Priority
    &ConnectWiFiTaskHandle, // Task handle
    1                       // Core 1
  );

  xTaskCreatePinnedToCore(IotTask, 
    "IotTask",      // Task name
    7000,           // Stack size (bytes)
    NULL,           // Parameters
    1,              // Priority
    &IotTaskHandle, // Task handle
    0               // Core 1
  );

  // vTaskResume(GeneralTaskHandle);
  // vTaskResume(IotTaskHandle);

  memset(&currentIotDataWheel, 0, sizeof(currentIotDataWheel));
 
  // Mount NVS Flash for users
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }

  wifiCasePtr = 0;
  mainCasePtr = 0;

  lcd_i2c.init(); // initialize the lcd
  lcd_i2c.backlight();
}
void loop() {
  vTaskDelay(1 / portTICK_PERIOD_MS);

  switch (mainCasePtr) {

  // Splash
  case 0: 
    lcdWrite("Wheel " + VERSION, "Connecting...");
    startTimout(5);
    readOperatorsFromFile();
    readIotDataFile();
    readSettingsFromFlash();
    mainCasePtr++;
   break;

  // Splash Screen timout
  case 1: 
  if (timeoutFlag) { 
    timeoutFlag = false;
      mainCasePtr++;
    }
   break;

  // Ready
  case 2: {
    if(sessionOpen) {
      lcdWrite(
        "Ready", 
        "(Start)(Stop)", 
        true
      );
    } 
    else {
      lcdWrite(
        "Ready", 
        "(Start)", 
        true
      );
    }

    oldDistance = 0;
    wheelDistance = 0;
    isRunning = 0;
    mainCasePtr++;
    subCasePtr = SUB_CASE_NONE;
  } break;

  // Home (Idle):
  // ----------------------------------------------------------------
  // - Wait for isPairing Button Press
  // - Wait for Connection request ----------------------------------------
  case 3: {
   
    // Keep MQTT Alive
    mqttServer.loop();

    // TESTING
    if (upKeyPressed) {
      //upKeyPressed = false;
      // saveWifiCredToFlash(ssid, password, serverIP);
      // readWifiCredFromFlash();
      //readIotDataFile();
    }
    if (downKeyPressed) {
      downKeyPressed = false;
      deleteIotDataFile();
      //deleteSessionNrFile();
    }

    // New Battery Reading
    if(newBatReadingAvailable){
      newBatReadingAvailable = false;
      lcdRefresh();
    }

    // isPairing
    if (startPairing) {
      startPairing = false;
      androidPaired = false;
      mainCasePtr = CASE_PAIR;
      break;
    }

    // TX Live Data
    if (androidConnected) {
      mainCasePtr = CASE_LIVE_WHEEL_DATA;
      break;
    }

    // New Settings Recieved
    if (newSettingsRecieved) {
      newSettingsRecieved = false;
      saveSettingsToFlash();
      lcdWrite("Settings Saved", "");
      startTimout(DISPLAY_TIMEOUT);
      mainCasePtr = CASE_DISPLAY_RETURN_HOME;
    }

    // IOT Type Error
    if (iotTypeError) {
      iotTypeError = false;
      lcdWrite("IOT Type Mismatch", "");
      startTimout(DISPLAY_TIMEOUT);
      mainCasePtr = CASE_DISPLAY_RETURN_HOME;
    }

    // startKeyPressed
    if (startKeyPressed) {
      startKeyPressed = false;
      wheelTicksCount = 0;
      wheelDistance = 0;
      mainCasePtr = 20;
      subCasePtr = 0;
      break;
    }

    // stopKeyPressed
    if (stopKeyPressed) {
      stopKeyPressed = false;
      if(sessionOpen) subCasePtr = 0;
      break;
    }

    // tag Presented
    if (tagPresented && !sessionOpen) {
      tagPresented = false;
      mainCasePtr = CASE_TAG_PRESENTED;
      PrintDebug("CasePtr: " + String(mainCasePtr), PRINT_GENERAL_DEBUG);
      break;
    }

    // New IOT Data Pushed
    if (newIotDataPushed) {
      newIotDataPushed = false;
      readIotDataFile();
      lcdWrite("Success", "");
      startTimout(DISPLAY_TIMEOUT);
      mainCasePtr++;
    }

    // Pushing IOT Data...
    if (isPushingIotData) {
      isPushingIotData = false;
      lcdWrite("Cloud Sync", "");
      startTimout(DISPLAY_TIMEOUT);
      mainCasePtr++;
    }

    // Pushing IOT Data (Timeout)
    if (isPushingIotDataTimeout) {
      isPushingIotDataTimeout = false;
      lcdWrite("TIMEOUT", "");
      startTimout(DISPLAY_TIMEOUT);
      mainCasePtr++;
    }

    // Max User Reached - TO DO _ FINISH THIS CODE
    if (maxUsersReached) {
      maxUsersReached = false;
      lcdWrite("Max Users Reached", "");
      startTimout(DISPLAY_TIMEOUT);
      mainCasePtr = CASE_DISPLAY_RETURN_HOME;
    }

    // Wifi Status
    if (WiFi.status() == WL_CONNECTED) {
      isWifiConnected = true;
    } else {
      isWifiConnected = false;
      isMqttServiceConnected = false;
    }

    // End Session
    switch (subCasePtr) {

      // (End Session) Confirm
      case 0:{
        lcdWrite(
          "End Session?", 
          "(Ent)(Back)", 
          true
        );
        subCasePtr++;
      } break;

      // (End Session) Confirm
      case 1:{
        if(backKeyPressed) {
          backKeyPressed = false;
          subCasePtr = SUB_CASE_NONE;
          mainCasePtr = CASE_HOME;
        }
        
        if(enterKeyPressed) {
          enterKeyPressed = false;
          lcdWrite(
            "Supervisor Tag", 
            "(Back)", 
            true
          );
          startTimout(2);
          subCasePtr++;
        }
      } break;

       // (End Session) Supervisor Tag
      case 2:{
          lcdWrite("Super Tag", "(Back)", true);
          subCasePtr++;
      } break;

      // (End Session) Confirm
      case 3: {
        if(tagPresented){
          tagPresented = false;
          
          User user = getUserNameByTag(tagCode);
          if(user.accessLevel[0]) {
            if(compareString(user.accessLevel, ACCESS_LEVEL_SUPERVISOR)) {
              strncpy(
                currentIotDataWheel.supervisorName, 
                user.name, 
                sizeof(currentIotDataWheel.supervisorName) - 1
              );
              sessionOpen = false;
              subCasePtr = 6;
            }
            else{
              subCasePtr++; // Invalid Tag
            }
          }
          
          if(backKeyPressed){       
            subCasePtr = SUB_CASE_NONE;
            mainCasePtr = CASE_HOME;
          };
        }
      } break;

       // (End Session) Invalid Tag
      case 4:{
        lcdWrite("Invalid Tag", "");
        startTimout(3);
        subCasePtr++;    
      } break;
      
      // (End Session) Back to Start
      case 5:{
        if (timeoutFlag) {
          timeoutFlag = false;
          subCasePtr = 2;    
        }
      } break;

        // (End Session) Invalid Tag
      case 6:{
          lcdWrite("Session Ended", currentIotDataWheel.supervisorName);
          startTimout(2);
          subCasePtr++;    
        } break;
        
        // (End Session) Back to Start
        case 7:{
          if (timeoutFlag) {
            timeoutFlag = false;
            subCasePtr = SUB_CASE_NONE;
            mainCasePtr = CASE_HOME ;   
          }
        } break;
    }
  } break;

  // Display Delay - Return Home
  case 4: {
    if (timeoutFlag) {
      timeoutFlag = false;
      mainCasePtr = 2;
    }
  } break;

  // (Pairing) Start ----------------------------------------
  
  case 10: {
    // Keep MQTT Alive
    mqttServer.loop();

    if (!isWifiConnected) {
      lcdWrite("Pairing ...", "No WiFi");
      startTimout(DISPLAY_TIMEOUT);
      mainCasePtr = 12;
      break;
    }

    if (!mqttServer.connected()) {
      lcdWrite("Pairing ...", "No MQTT");
      startTimout(DISPLAY_TIMEOUT);
      mainCasePtr = 12;
      break;
    }

    lcdWrite("Pairing...", "");

    isPairing = true;
    androidPaired = false;
    startTimout(PAIR_TIMEOUT);
    mainCasePtr++;
  } break;

  // (isPairing) Wait:
  // - Monitor Found
  // - Cancel isPairing button
  // - Timeout
  case 11: {
    // Monitor Found (From Android)
    if (androidPaired) {
      timerAlarmDisable(timer);
      digitalWrite(LED_MQTT_CONNECTED, HIGH);

      isPairing = false;

      PrintDebug("Monitor Found", PRINT_GENERAL_DEBUG);
      lcdWrite("Connected!", "");
      startTimout(DISPLAY_TIMEOUT);
      mainCasePtr++;
      break;
    }

    // User Cancelled
    if (btnPairPressed) {
      timerAlarmDisable(timer);
      digitalWrite(LED_MQTT_CONNECTED, HIGH);

      btnPairPressed = false;
      isPairing = false;
      androidPaired = false;

      PrintDebug("User Cancelled Advertising", PRINT_GENERAL_DEBUG);
      lcdWrite("Cancelled", "");
      startTimout(3);
      mainCasePtr++;
      break;
    }

    if (timeoutFlag) {
      timeoutFlag = false;
      isPairing = false;
      digitalWrite(LED_MQTT_CONNECTED, HIGH);

      lcdWrite("Timeout", "");
      startTimout(3);
      mainCasePtr++;
      break;
    }
  } break;

  // (isPairing) Message Delay
  case 12: {
    if (timeoutFlag) {
      timeoutFlag = false;
      mainCasePtr = CASE_HOME;
    }
  } break;

  // (Wheel) Start ----------------------------------------

  // (Wheel) Start - Present Tag
  case 20: {
    switch (subCasePtr) {
      // Lets Go
      case 0:{
        lcdWrite("Lets GO ...", "");
        startTimout(DISPLAY_TIMEOUT);
        subCasePtr++;    
      } break;
      
      // Start
      case 1:{
        if(timeoutFlag) {
          timeoutFlag = false;
          
          if(sessionOpen){
            isRunning = true;
            nrOfLanesToCut[0] = '\0';
            laneIndex = 0;
            tagPresented = false;
            mainCasePtr++;
            subCasePtr = 0;    
          }
          else{
            subCasePtr++;  
          }
        } 
      } break;
      
      // (Open Session) Start Session
      case 2:{
          lcdWrite("Supervisor Tag", "(Back)");
          subCasePtr++;
      } break;
      
      // (Open Session) Tag / Back
      case 3:{

        if (backKeyPressed) {
          backKeyPressed = false;
          mainCasePtr = CASE_HOME;
          subCasePtr = SUB_CASE_NONE;
        }
        
        // Supervisor Tag presented
        if (tagPresented) {
          tagPresented = false;
 
          User user = getUserNameByTag(tagCode);
          
          if(user.name[0]) {
            // Found User
            strncpy(
              currentIotDataWheel.supervisorName, 
              user.name, 
              sizeof(currentIotDataWheel.supervisorName) - 1
            );
            
            if(!compareString(user.accessLevel, ACCESS_LEVEL_SUPERVISOR)) {
              subCasePtr = 4;
              break;
            }
            
            sessionOpen = true;
            subCasePtr = 6;
          } 
          else{
            subCasePtr++; // Uknown Tag
          }
        }     
      } break;  
      
      // (Open Session) Invalid Tag
      case 4:{
        lcdWrite("Invalid Tag", "");
        startTimout(3);
        subCasePtr++;    
      } break;
      
      // (Open Session) Back to Start
      case 5:{
        if (timeoutFlag) {
          timeoutFlag = false;
          subCasePtr = 2;    
        }
      } break;

       // (Open Session) Session Open
      case 6:{
        lcdWrite("Session Open", currentIotDataWheel.supervisorName);
        startTimout(2);
        subCasePtr++;    
      } break;
      
      // (Open Session) Proceed
      case 7:{
        if (timeoutFlag) {
          timeoutFlag = false;
          subCasePtr = 0;
          mainCasePtr++;    
        }
      } break;

      default:
        break;
    }
  } break;

  // (Wheel) Operator Tag
  case 21: {
    switch (subCasePtr)
    {
      // Operator Tag
      case 0:{
        lcdWrite("Operator Tag", "(Back)");
        subCasePtr++;
      } break;

      // Wait Tag / Back
      case 1:{

        if (backKeyPressed) {
          backKeyPressed = false;
          mainCasePtr = CASE_HOME;
          subCasePtr = SUB_CASE_NONE;
        }
        
        // Tag presented
        if (tagPresented) {
          tagPresented = false;
          User user = getUserNameByTag(tagCode);
          
          if(user.name[0]) {
            // Found User
            strncpy(currentIotDataWheel.operatorName, user.name, sizeof(currentIotDataWheel.operatorName) - 1);
            subCasePtr = 4;
          } 
          else{
            subCasePtr++; // Uknown Tag
          }
        }     
      } break;  
      
      // Uknown Tag
      case 2:{
        lcdWrite("Unknown Tag", "");
        startTimout(3);
        subCasePtr++;    
      } break;
      
      // Back to Start
      case 3:{
        if (timeoutFlag) {
          timeoutFlag = false;
          subCasePtr = 0;    
        }
      } break;

       // Show Operator
      case 4:{
        lcdWrite(currentIotDataWheel.operatorName, "");
        startTimout(2);
        subCasePtr++;    
      } break;
      
      // Proceed
      case 5:{
        if (timeoutFlag) {
          timeoutFlag = false;
          subCasePtr = SUB_CASE_NONE;
          mainCasePtr++;    
        }
      } break;
      
      default:
        break;
    }
    
  } break;

  // (Wheel) Start
  case 22: {
    if (timeoutFlag) {
      timeoutFlag = false;
      lcdWrite("Dist: " + String(wheelDistance) + "m", "(Back)(Stop)");
      mainCasePtr++;

      //DEBUG
      simulateWheelDistance = true;
      startTimout(1);
      //DEBUG
    }
  } break;

  // (Wheel) Reading Distance ...
  case 23: {
    if (oldDistance != wheelDistance) {
      oldDistance = wheelDistance;
      lcdWrite("Dist: " + String(wheelDistance) + "m", "(Back)(Stop)");
    }

    // Cancel
    if (backKeyPressed) {
      backKeyPressed = false;
      subCasePtr = 0;
      mainCasePtr++;
    }

    // Stop
    if (stopKeyPressed) {
      stopKeyPressed = false;
      subCasePtr = 2;
      mainCasePtr++;
    }
  } break;

  // (Wheel) Confirm Cancel or Proceed
  case 24: {
    switch (subCasePtr) {
      case 0:{
        //Cancel  
        lcdWrite("Cancel?", "(Ent)(Back)");
        subCasePtr++;    
      } break;
      
      case 1:{
        // Cancel - Cancel
        if (backKeyPressed) {
          backKeyPressed = false;
          subCasePtr = SUB_CASE_NONE;
          mainCasePtr = 22;
        }

        // Cancel - Confirm
        if(enterKeyPressed) {
          enterKeyPressed = false;
          mainCasePtr = CASE_HOME;
          subCasePtr = SUB_CASE_NONE;
          simulateWheelDistance = false;    
        }
      } break;
      
      case 2:{
        // Stop
        lcdWrite("Stop?", "(Ent)(Back)");
        subCasePtr++;    
      } break;
      
      case 3:{
        // Stop - Cancel
        if (backKeyPressed) {
          backKeyPressed = false;
          subCasePtr = SUB_CASE_NONE;
          mainCasePtr = 22;
        }
        
        // Stop - Confirm
        if(enterKeyPressed) {
          enterKeyPressed = false; 
          simulateWheelDistance = false;

          laneIndex = 0;
          memset(nrOfLanesToCut, 0, sizeof(nrOfLanesToCut));
          lcdSetLanesPrompt();

          mainCasePtr++;
          subCasePtr = SUB_CASE_NONE;   
        }
      }  break;
     
      default:
        subCasePtr = SUB_CASE_NONE; 
      break;
    }
  } break;

  // (Wheel) Enter Lanes to count
  case 25: {

    // Back
    if (backKeyPressed) {
      backKeyPressed = false;
      newNumKeyPressed = false;
      key[0] = '\0';

      if (laneIndex > 0) {
        nrOfLanesToCut[--laneIndex] = '\0';
        lcdSetLanesPrompt();
      } else {
        mainCasePtr = 23;
      }
    }

    // Enter
    else if (enterKeyPressed) {
      enterKeyPressed = false;
      currentIotDataWheel.distance = wheelDistance;
      currentIotDataWheel.lines = strtol(nrOfLanesToCut, nullptr, 10);
      mainCasePtr++;
      subCasePtr = 0;
    }

    // Numbers
    else if (newNumKeyPressed && laneIndex < sizeof(nrOfLanesToCut) - 1) {
      newNumKeyPressed = false;
      bool isDigitKey = (key[1] == '\0' && key[0] >= '0' && key[0] <= '9');

      if (isDigitKey && !(laneIndex == 0 && strcmp(key, KEY_0) == 0)) {

        if (laneIndex < sizeof(nrOfLanesToCut) - 1) {
          nrOfLanesToCut[laneIndex++] = key[0];
          nrOfLanesToCut[laneIndex] = '\0';
        }
      }

      lcdSetLanesPrompt();
      key[0] = '\0'; // clear key
    }
  } break;
  
  
  // (Wheel) Save
  case 26: {
    switch (subCasePtr) {
      case 0:
        if (timeoutFlag) {
          timeoutFlag = false;
          
          // monId
          strncpy(
            currentIotDataWheel.monDocId, 
            settingsMonDocId,     
            sizeof(currentIotDataWheel.monDocId) - 1
          );
          
          // userDocId
          strncpy(
            currentIotDataWheel.userDocId, 
            settingsUserDocId,
            sizeof(currentIotDataWheel.userDocId) - 1
          );

          time_t t = getTimeStamp();
          String timestamp = String(t);
          
          // timestamp
          strncpy(
            currentIotDataWheel.timestamp, 
            timestamp.c_str(),
            sizeof(currentIotDataWheel.timestamp) - 1
          );
          
          currentIotDataWheel.monDocId[sizeof(currentIotDataWheel.monDocId) - 1] = '\0';
          currentIotDataWheel.userDocId[sizeof(currentIotDataWheel.userDocId) - 1] = '\0';
          currentIotDataWheel.timestamp[sizeof(currentIotDataWheel.timestamp) - 1] = '\0';
          currentIotDataWheel.gpsCoord[0] = '\0'; // TODO

          writeIotDataToFile(currentIotDataWheel);
          lcdWrite(String(wheelDistance) + "m X " + String(nrOfLanesToCut),currentIotDataWheel.operatorName);
          startTimout(3);
          subCasePtr++;
        }    
      break;
      
      case 1:
       if (timeoutFlag) {
            mainCasePtr = CASE_HOME;
            subCasePtr = SUB_CASE_NONE; 
          }
      break;
           
      default:
        subCasePtr = SUB_CASE_NONE; 
      break;
    }
    
  } break;


  // ----------------------------------------
  // Live Data
  // ----------------------------------------

  case 40: {
    lcdWrite("Distance: " + String(wheelDistance) + "m", "Live");
    isRunning = true;
    oldDistance = 0;
    wheelDistance = 0;
    mainCasePtr++;
  } break;

  case 41: {
    if (backKeyPressed) {
      backKeyPressed = false;
      isRunning = false;
      mainCasePtr = CASE_HOME;
    }

    // Wheel Moved
    if (oldDistance != wheelDistance) {
      oldDistance = wheelDistance;
      lcdWrite("Distance: " + String(wheelDistance) + "m", "Live", false);
      mqttReportLiveIotData();
    }

    // Android Disconnected
    if (!androidConnected) {
      mainCasePtr = CASE_HOME;
    }
  } break;

  // ----------------------------------------
  // Tag Presented
  // ----------------------------------------
  case 50: {
    if (tagCRCError) {
      tagCRCError = false;
      lcdWrite("Tag", "CRC Error");
    } else {
      PrintDebug("CasePtr: " + String(mainCasePtr), PRINT_GENERAL_DEBUG);

      String tag = tagToString(tagCode);
  
      User user = getUserNameByTag(tagCode);
      if(user.name[0]) {
        // Found User
        lcdWrite(
          user.name, 
          tag.substring(0, 10).c_str(),
          false
        );
      } 
      else{
        // Found User
        lcdWrite(
          "Unknown Tag", 
          tag.substring(0, 10).c_str(), 
          false
        );
      }   
    }

    if (isWifiConnected) {
      mqttReportTag();
    }

    startTimout(3);
    mainCasePtr++;
  } break;

  case 51: {
    if (timeoutFlag) {
      mainCasePtr = 2;
    }
  } break;

  default:
    break;
  }
}