// ESP32 GeoFence Monitor

#include <WiFi.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>

hw_timer_t *timer = NULL;
volatile bool timerFlag = false;

Preferences prefs;

// wifi
WiFiClient wifiClient;
PubSubClient mqttServer(wifiClient);
String ssid = "VoxFibre#59923";
String password = "8JudJNS2";

char serverIP[16];
const int port = 8080;

bool wifiCredentialsExist()
{
  prefs.begin("wifi", true);
  bool ok = prefs.isKey("ssid") && prefs.isKey("pw");
  prefs.end();
  return ok;
}

#define DEBOUNCE_DELAY 100
#define DEBOUNCE_DELAY_HELD 3000
#define MS_DELAY 10

int wifiCasePtr = 0;
int mainCasePtr = 0;
bool isBluetoothConnected = false;
bool isWifiConnected = false;
bool isMqttServiceConnected = false;
bool isRPiConnectingBusy = false;
bool isRPiConnected = false;
bool gotWifiCredentials = false;

// Commands
String CMD_SHARED_WIFI_CREDENTIALS = "wificred:"; // Format: wificred:ssid#password

// Button Debounce
volatile bool debPairFallEdge = false;
volatile bool debPairPressed = false;
volatile bool debPairHeld = false;
volatile int debPairLastTime = 0;

bool btnPairPressed = false;

// MQTT Command
const String MQTT_CMD_REQ_MONITOR = "#REQ_MONITOR";
const String MQTT_CMD_FOUND_MONITOR = "#FOUND_MONITOR";
const String MQTT_CMD_DEVICE_ID = "#DEVICE_ID";
const String MQTT_CMD_ACK = "#ACK";

// MQTT Topics
const String MQTT_TOPIC_FROM_IOT = "mqtt/from/iot";
const String MQTT_TOPIC_TO_IOT = "mqtt/to/iot";
String MQTT_TOPIC_TO_IOT_PRIVATE = "";
const String MQTT_TOPIC_CRED = "mqtt/credentials";

// MQTT JSON
const String MQTT_JSON_DEVICE_ID = "device_id";
const String MQTT_JSON_TOPIC = "topic";
const String MQTT_JSON_PAYLOAD = "payload";
const String MQTT_JSON_CMD = "command";

// IO
const int BUT_PAIR = 22;           // BT Pair button
const int LED_BLUETOOTH = 2;       // Blink (Pairing), Solid (Connected)
const int LED_WIFI_CONNECTED = 12; // Blink (Searching), Solid (Connected)
const int LED_MQTT_CONNECTED = 13; // Blink (Searching), Solid (Connected)
const int BUZZER = 23;             // Buzzer

// Declare task handle
TaskHandle_t BlinkTaskHandle = NULL;
TaskHandle_t ConnectWiFiTaskHandle = NULL;

// Bluetooth
#define SERVICE_UUID "f3a1c2d0-6b4e-4e9a-9f3e-8d2f1c9b7a1e"
#define CHAR_UUID "c7b2e3f4-1a5d-4c3b-8e2f-9a6b1d8c2f3a"
const String BT_NAME = "iOT"; // Bluetooth Name
NimBLECharacteristic *pChar = nullptr;
NimBLEServer* pServer = nullptr;
NimBLEService* pService = nullptr;

bool btConnected = false;
bool isAdvertising = false;
bool monitorFound = false;
bool monitorRequested = false;
const int ADVERTISE_TIMEOUT = 20; // seconds
int runningTime = 0; // seconds
bool wifiConnected = false;
bool time_1sec_Flag = false;
String DeviceName;

const bool PRINT_GENERAL_DEBUG = false;
const bool PRINT_DEBOUNCE_DEBUG = true;

// Declare functions
void DebouncePairBtn();
void SendHttpMsg(String msg);
void buttonISR();
void BT_StopServer();
void BT_StartServer();
void BT_StartAdvertising();
void MqttTX(String,String);

//------------------------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------------------------

// Timer
void IRAM_ATTR onTimerOneSec()
{
  timerFlag = true;
}


// General
void PrintDebug(String msg, bool print){
  if (print)
  {
    Serial.printf("%s\n", msg.c_str());
  }
}
void IRAM_ATTR buttonISR()
{
  debPairFallEdge = true;
}
void DebouncePairBtn()
{
  if (debPairFallEdge)
  {
    debPairLastTime = millis();
    debPairFallEdge = false;
  }

  if (!debPairPressed && !debPairHeld)
  {
    if (debPairLastTime > 0)
    {
      if (digitalRead(BUT_PAIR))
      {
        debPairLastTime = 0;
        return;
      }

      // Button Pressed
      uint32_t currentTime = millis();
      if (currentTime - debPairLastTime > DEBOUNCE_DELAY)
      {
        debPairPressed = true;
        btnPairPressed = true;
        PrintDebug("Button Pressed", PRINT_DEBOUNCE_DEBUG);
      }
    }
  }
  else if (debPairPressed && !debPairHeld)
  {
    uint32_t currentTime = millis();

    // Start Bluetooth Pairing
    // Button HELD
    if (currentTime - debPairLastTime > DEBOUNCE_DELAY_HELD)
    {
      debPairHeld = true;
      PrintDebug("Button HELD", PRINT_DEBOUNCE_DEBUG);
    }

    // Wait for release before HELD
    if (digitalRead(BUT_PAIR) == HIGH)
    {
      debPairPressed = false;
      debPairHeld = false;
      debPairLastTime = 0;
      PrintDebug("Button Released", PRINT_DEBOUNCE_DEBUG);
    }
  }
  else
  {
    // Button HELD (Wait for release)
    if (digitalRead(BUT_PAIR) == HIGH)
    {
      debPairPressed = false;
      debPairHeld = false;
      debPairLastTime = 0;
      PrintDebug("Button Released", PRINT_DEBOUNCE_DEBUG);
    }
  }
}
void LedBlinkTask(void *parameter)
{
  int cntBluetooth = 0;
  int cntWifi = 0;
  int cntMqtt = 0;
  int cntAdvertising = 0;

  bool ledBluetoothState = LOW;
  bool ledWifiState = LOW;
  bool ledMqttState = LOW;
  bool ledAdvertisingState = LOW;

  for (;;)
  {
    if (!isBluetoothConnected) {
      if(cntBluetooth++ >= 300) {
        cntBluetooth = 0;
        ledBluetoothState = !ledBluetoothState;

        if(ledBluetoothState) digitalWrite(LED_BLUETOOTH, HIGH);
        else digitalWrite(LED_BLUETOOTH, LOW);
      }
    }  

    if (!isWifiConnected) {
      if(cntWifi++ >= 500) {
        cntWifi = 0;
        ledWifiState = !ledWifiState;

        if(ledWifiState) digitalWrite(LED_WIFI_CONNECTED, HIGH);
        else digitalWrite(LED_WIFI_CONNECTED, LOW);
      }
    }
    
    if (!isMqttServiceConnected || isAdvertising) {
      if(cntMqtt++ >= 700) {
        cntMqtt = 0;
        ledMqttState = !ledMqttState;

        if(ledMqttState) digitalWrite(LED_MQTT_CONNECTED, HIGH);
        else digitalWrite(LED_MQTT_CONNECTED, LOW);
      }
    }
   
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
String getDeviceName() {
    // Read raw 48-bit efuse MAC (never modified)
    uint64_t mac = ESP.getEfuseMac();

    // Extract 6 bytes manually (correct order, no bit masking)
    uint8_t macBytes[6];
    for (int i = 0; i < 6; i++) {
        macBytes[i] = (mac >> (8 * (5 - i))) & 0xFF;
    }

    // Format as 12-char hex string (uppercase)
    char macStr[13];  // 12 hex chars + null
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X",
            macBytes[5], macBytes[4], macBytes[3],
            macBytes[2], macBytes[1], macBytes[0]);

    // Build device name
    String deviceName = BT_NAME + "_";
    deviceName += macStr;

    // Lowercase for BLE/MQTT safety (in-place)
    //deviceName.toLowerCase();

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

// MQTT
void mqttCallback(char *topic, byte *payload, unsigned int length){
  
  String msg = "";
  
  Serial.print("MQTT RX: [");
  Serial.print(topic);
  Serial.print("] ");
  
  for (int i = 0; i < length; i++)
    msg += (char)payload[i];
  Serial.println(msg);

  // 2️⃣ Parse JSON
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, msg);
 
  if (error) {
      Serial.print("JSON parse failed: ");
      Serial.println(error.c_str());
      return;
    }
    
  String _payload = doc[MQTT_JSON_PAYLOAD];
  String _topic = doc[MQTT_JSON_TOPIC];
  String _cmd = doc[MQTT_JSON_CMD];

  if(_topic == MQTT_TOPIC_TO_IOT){
  
    // Monitor Found
    if(_cmd == MQTT_CMD_FOUND_MONITOR){  
     monitorFound = true;
    }

    // Request Monitor 
    if(_cmd == MQTT_CMD_REQ_MONITOR){  
     monitorRequested = true;
    }
  }
}
void MqttTX(const JsonDocument& msg, const String& topic)
{
  if (!isWifiConnected) {
    Serial.println("WiFi not connected");
    return;
  }

  if (!mqttServer.connected()) {
    Serial.println("MQTT not connected");
    return;
  }

  String payload;
  serializeJson(msg, payload);

  bool ok = mqttServer.publish(topic.c_str(), payload.c_str());

  if (ok) {
    Serial.print("MQTT TX: ");
    Serial.println(payload);   // Safe, prints the JSON string
  } else {
    Serial.println("MQTT publish failed");
  }
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
void wifiConnectTask(void *parameter){
  int retry = 0;

  for (;;)
  { 
    switch (wifiCasePtr)
    {
      // Connecting Wifi
      case 0: 
      {
        Serial.printf("\nConnecting WiFi... %s:1883\n", serverIP);
        retry = 2;

        //wifiScanNetwork();
        WiFi.begin(ssid, password);
        mqttServer.setServer(serverIP, 1883);
        mqttServer.setCallback(mqttCallback);
        delay(1000);
        wifiCasePtr++;
      }
      break;
              
      // Wifi Connected
      case 1:
      {
        if (WiFi.status() == WL_CONNECTED)
        {
          Serial.println("\nConnected!");
          Serial.print("IP address: ");
          Serial.println(WiFi.SSID());
          Serial.println(WiFi.localIP());
          isWifiConnected = true;
          wifiCasePtr++;
        }
        else
        {
          Serial.printf("Wifi Stat: %d. Retry in %iS ...\n", WiFi.status, retry);
          delay(1000);
          if(retry-- == 0) wifiCasePtr = 0;
        }
      }
      break;

      // Connecting MQTT
      case 2:
      {
        Serial.println("\nConnecting MQQT ...");
        
        if (mqttServer.connect(DeviceName.c_str()))
        {
          Serial.println("MQTT Connected!");
          mqttServer.loop();
          wifiCasePtr++;
        }
        else
        {
          Serial.print("MQTT Connect failed, ERROR:");
          Serial.print(mqttServer.state());
          Serial.println(" try again in 5 seconds");
          delay(5000);
        }
      }
      break;

      // MQTT Connect
      case 3:
      {
        if (mqttServer.connected())
        {
          Serial.println("\nConnected!\n");
          Serial.print("IP address: ");
          Serial.println(WiFi.localIP());
          isMqttServiceConnected = true;
          wifiCasePtr++;
        }
      }
      break;

      // Subscribe MQTT Topics
      case 4:
      {
        // Broadcast Subscribe
        mqttServer.subscribe(MQTT_TOPIC_TO_IOT.c_str());
        Serial.printf("\nMQTT Subscribe ...%s", MQTT_TOPIC_TO_IOT.c_str());
        
        // Private Subscribe
        MQTT_TOPIC_TO_IOT_PRIVATE = MQTT_TOPIC_TO_IOT + '/' + DeviceName;
        mqttServer.subscribe(MQTT_TOPIC_TO_IOT_PRIVATE.c_str());
        Serial.printf("\nMQTT Subscribe ...%s\n", MQTT_TOPIC_TO_IOT_PRIVATE.c_str());
        
        wifiCasePtr++;
      }
      break;

      // Keep MQTT alive - Check Connection
      case 5:
      {
        mqttServer.loop();  // Keep MQTT Alive

        if(!mqttServer.connected()){
          mainCasePtr = 0;
        }

        if (WiFi.status() != WL_CONNECTED)isWifiConnected = false;
        else isWifiConnected = true;
        
        delay(500);
      }
      break;
      
      default:
        break;
    }
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

// Bluetooth
class BT_ServerCallbacks : public NimBLEServerCallbacks{
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo)
  {
    Serial.printf("BT Client connected: %s \n", connInfo.getAddress().toString().c_str());
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
    PrintDebug("Rx " + String(rxData), PRINT_GENERAL_DEBUG);
    
    // Recieved WiFi Credentials
    if (rxData.startsWith("wificred:"))
    {
      rxData.remove(0, 9); // remove "wificred:"

      int i1 = rxData.indexOf(">");
      int i2 = rxData.indexOf(">", i1 + 1);

      if (i1 != -1 && i2 != -1)
      {
        ssid = rxData.substring(0, i1);
        password = rxData.substring(i1 + 1, i2);

        // Set Global IP address part
        String ip = rxData.substring(i2 + 1);
        ip.toCharArray(serverIP, sizeof(serverIP));

        Serial.println("SSID: " + ssid);
        Serial.println("Password: " + password);
        Serial.println("ServerIP: " + String(serverIP)); // char*

        gotWifiCredentials = true;
      }
      else
      {
        Serial.println("Invalid WiFi credentials format: " + rxData);
      }
    }
  }

  void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override
  {
    // Raspberry PI Subscribed to notifications
    Serial.println("BT Client subscribed to notifications\n");
  }
};
void BT_StopServer() {
    if (pServer != nullptr) {
        Serial.println("Stopping existing BLE server...");
        NimBLEDevice::stopAdvertising();
        pServer->removeService(pService);
        pService = nullptr;
        pChar = nullptr;
        NimBLEDevice::deinit(true);   // fully shuts down BLE
        pServer = nullptr;
        delay(100);
    }
}
void BT_StartServer(){
  BT_StopServer();

  Serial.print("Starting BT Server...\n");

  NimBLEDevice::init(DeviceName.c_str());
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
  Serial.printf("BT Server Started: %s\n", DeviceName.c_str());
}
void BT_StartAdvertising(){
  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;

  advData.setFlags(0x06); // general discoverable + BR/EDR not supported
  advData.addServiceUUID(SERVICE_UUID);
  pAdv->setAdvertisementData(advData);

  NimBLEAdvertisementData scanResp;
  scanResp.setName(DeviceName.c_str()); // full name delivered in scan response
  pAdv->setScanResponseData(scanResp);

  NimBLEDevice::startAdvertising();
  Serial.printf("BT Advertising Started:  %s \n", DeviceName.c_str());
}

void setup(){
  Serial.begin(9600);
  pinMode(LED_BLUETOOTH, OUTPUT);
  pinMode(LED_WIFI_CONNECTED, OUTPUT);
  pinMode(LED_MQTT_CONNECTED, OUTPUT);
  pinMode(BUT_PAIR, INPUT_PULLUP);

  DeviceName = getDeviceName();

  // timer 0, prescaler 80 → 1 tick = 1 µs
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimerOneSec, true);

  // 1,000,000 µs = 1 second
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);

  attachInterrupt(
      digitalPinToInterrupt(BUT_PAIR),
      buttonISR,
      FALLING);

  xTaskCreatePinnedToCore(
      LedBlinkTask,        // Task function
      "BlinkTask",      // Task name
      10000,            // Stack size (bytes)
      NULL,             // Parameters
      1,                // Priority
      &BlinkTaskHandle, // Task handle
      1                 // Core 1
  );

  xTaskCreatePinnedToCore(
      wifiConnectTask,        // Task function
      "ConnectWiFiTask",      // Task name
      10000,                  // Stack size (bytes)
      NULL,                   // Parameters
      1,                      // Priority
      &ConnectWiFiTaskHandle, // Task handle
      1                       // Core 1
  );

  vTaskSuspend(BlinkTaskHandle);
  vTaskSuspend(ConnectWiFiTaskHandle);
  
  wifiCasePtr = 0;
  mainCasePtr = 0;
}

void loop()
{
  vTaskDelay(10 / portTICK_PERIOD_MS);
  DebouncePairBtn();

  switch (mainCasePtr) {
    // Start Bluetooth Server
    case 0:{
      isBluetoothConnected = false;
      isWifiConnected = false;
      isMqttServiceConnected = false;
      gotWifiCredentials = false;
      wifiCasePtr = 0;

      vTaskResume(BlinkTaskHandle);
      vTaskSuspend(ConnectWiFiTaskHandle);

      BT_StartServer();
      BT_StartAdvertising();
      mainCasePtr++;
    }
    break;

    // Wait for bluetooth Connection
    case 1:{
      if (isBluetoothConnected)
      {
        digitalWrite(LED_BLUETOOTH, HIGH); // Solid ON
        Serial.println("Waiting for Wifi Credentials ... ");
        mainCasePtr++;
      }
    }
    break;

    // Wait for Wifi Credentials via Bluetooth
    case 2:{
      if (gotWifiCredentials)
      {
        // Start WiFi Connection
        isWifiConnected = false;
        isMqttServiceConnected = false;

        saveWifiCredentials(ssid, password, serverIP);
        vTaskResume(ConnectWiFiTaskHandle);
        mainCasePtr++;
      }
    }
    break;

    // Check Wifi Connection
    case 3:{
      if (isWifiConnected)
      {
        digitalWrite(LED_WIFI_CONNECTED, HIGH); // Solid ON
        mainCasePtr++;
      }
    }
    break;

    // Check MQTT Service Connection
    case 4:{
      if (isMqttServiceConnected)
      {
        digitalWrite(LED_MQTT_CONNECTED, HIGH); // Solid ON
        mainCasePtr++;
      }
    }
    break;
  
    // Send Device Name
    case 5:{
      StaticJsonDocument<256> mqttPacket;
      mqttPacket[MQTT_JSON_DEVICE_ID] = DeviceName;
      mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
      mqttPacket[MQTT_JSON_PAYLOAD] = "";
      mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_DEVICE_ID;
      
      String payload;
      serializeJson(mqttPacket, payload);      
      MqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
      mainCasePtr++;
    }
    break;

    // Idle - Wait for Pair Button Press
    case 6:{
      if (btnPairPressed)
      {
        btnPairPressed = false;
        isAdvertising = true;
        monitorFound = false;
        monitorRequested = false;
        runningTime = 0;
  
        if(mqttServer.connected()){
          timerAlarmEnable(timer);
          Serial.printf("Advertising Device ID... %s\n", DeviceName.c_str());
          mainCasePtr++;
        }
        else{
          Serial.println("MQTT not connected, Cant advertise ID");
          mqttServer.connect(DeviceName.c_str());
        }
      }
    }
    break;
    
    // Waiting for Monitor to be Found
    case 7:{
      // Keep MQTT Alive
      mqttServer.loop();
      
      // Monitor Requested
      if(monitorRequested){
        monitorRequested = false;
        
        StaticJsonDocument<256> mqttPacket;
        mqttPacket[MQTT_JSON_DEVICE_ID] = DeviceName;
        mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
        mqttPacket[MQTT_JSON_PAYLOAD] = DeviceName;
        mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_REQ_MONITOR;

        String payload;
        serializeJson(mqttPacket, payload);      
        MqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);
      }

      // Monitor Found (From Android)
      if(monitorFound) { 
        timerAlarmDisable(timer);
        isAdvertising = false;
        
        // Send Confirmation MQTT
        StaticJsonDocument<256> mqttPacket;
        mqttPacket[MQTT_JSON_DEVICE_ID] = DeviceName;
        mqttPacket[MQTT_JSON_TOPIC] = MQTT_TOPIC_FROM_IOT;
        mqttPacket[MQTT_JSON_PAYLOAD] = DeviceName;
        mqttPacket[MQTT_JSON_CMD] = MQTT_CMD_FOUND_MONITOR;

        String payload;
        serializeJson(mqttPacket, payload);      
        MqttTX(mqttPacket, MQTT_TOPIC_FROM_IOT);

        digitalWrite(LED_MQTT_CONNECTED, HIGH);
        Serial.println("Monitor Found");  
        mainCasePtr = 6;
      }
      
      // User Cancelled
      if(btnPairPressed) {
          btnPairPressed = false;
          timerAlarmDisable(timer);
          mainCasePtr = 6;
          isAdvertising = false;
          monitorFound = false;
        
          digitalWrite(LED_MQTT_CONNECTED, HIGH);
          Serial.println("User Cancelled Advertising");
          mainCasePtr = 6;  
      }

      if (timerFlag) {
        timerFlag = false;

         // Timeout 20s
        if(runningTime++ >= ADVERTISE_TIMEOUT){
          timerAlarmDisable(timer);
          isAdvertising = false;
          Serial.println("Advertising Timeout");
          mainCasePtr = 6;
        }
         // Re-Advertise
         mainCasePtr = 7;
      }
    }
    break;
        
    default:
    break; 
  }
}