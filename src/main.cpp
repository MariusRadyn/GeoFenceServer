// ESP32 GeoFence Monitor

#include <WiFi.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <Preferences.h>

Preferences prefs;

// wifi
WiFiClient wifiClient;
PubSubClient wifiPublishClient(wifiClient);
String ssid = "VoxFibre#59923";
String password = "8JudJNS2";
// const char *ssid = "VoxFibre#59923";
// const char *password = "8JudJNS2";
char serverIP[16];
const int port = 8080;

bool wifiCredentialsExist() {
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
volatile bool btnPairFallEdge = false;
volatile bool btnPairPressed = false;
volatile bool btnPairHeld = false;
volatile int lastDebTime = 0;

//MQTT Topics
const String MQTT_TOPIC_REQ = "device/settings/request";
const String MQTT_TOPIC_RESPONSE = "device/settings/response";
const String MQTT_TOPIC_CRED ="device/settings/credentials";

// IO
const int BUT_PAIR = 22; // BT Pair button
const int LED_BLUETOOTH = 2; // Blink (Pairing), Solid (Connected) 
const int LED_WIFI_CONNECTED = 12; // Blink (Searching), Solid (Connected)
const int LED_MQTT_CONNECTED = 13; // Blink (Searching), Solid (Connected)
const int BUZZER = 23; // Buzzer

// Declare task handle
TaskHandle_t BlinkTaskHandle = NULL;
TaskHandle_t ConnectWiFiTaskHandle = NULL;

// Bluetooth
#define SERVICE_UUID "f3a1c2d0-6b4e-4e9a-9f3e-8d2f1c9b7a1e"
#define CHAR_UUID "c7b2e3f4-1a5d-4c3b-8e2f-9a6b1d8c2f3a"
const String BT_NAME = "geoserver"; // Bluetooth Name
NimBLECharacteristic* pChar;

bool btConnected = false;
bool wifiConnected = false;
bool time_1sec_Flag = false;
String deviceName;
bool printDebug = false;

// Declare functions
void DebouncePairBtn();
void SendHttpMsg(String msg);
void buttonISR();
void BT_StartServer();
void BT_StartAdvertising();

// General
void PrintDebug(String msg, bool print){
  if(print){
    Serial.printf("%s\n",msg.c_str());
  }
}
void IRAM_ATTR buttonISR()
{
  btnPairFallEdge = true;
}
void DebouncePairBtn()
{
  if (btnPairFallEdge)
  {
    lastDebTime = millis();
    btnPairFallEdge = false;
  }

  if (!btnPairPressed && !btnPairHeld)
  {
    if (lastDebTime > 0)
    {
      if (digitalRead(BUT_PAIR))
      {
        lastDebTime = 0;
        return;
      }

      // Button Pressed
      uint32_t currentTime = millis();
      if (currentTime - lastDebTime > DEBOUNCE_DELAY)
      {
        btnPairPressed = true;
        PrintDebug("Button Pressed", printDebug);
      }
    }
  }
  else if (btnPairPressed && !btnPairHeld)
  {
    uint32_t currentTime = millis();

    // Start Bluetooth Pairing
    // Button HELD 
    if (currentTime - lastDebTime > DEBOUNCE_DELAY_HELD)
    {
      btnPairHeld = true;     
      PrintDebug("Button HELD", printDebug);
    }

    // Wait for release before HELD
    if (digitalRead(BUT_PAIR) == HIGH)
    {
      btnPairPressed = false;
      btnPairHeld = false;
      lastDebTime = 0;
      PrintDebug("Button Released", printDebug);
    }
  }
  else
  {
    // Button HELD (Wait for release)
    if (digitalRead(BUT_PAIR) == HIGH)
    {
      btnPairPressed = false;
      btnPairHeld = false;
      lastDebTime = 0;
      PrintDebug("Button Released", printDebug);
    }
  }
}
void BlinkTask(void *parameter)
{
  for (;;)
  { 
    if(!isBluetoothConnected) digitalWrite(LED_BLUETOOTH, HIGH);
    if(!isWifiConnected) digitalWrite(LED_WIFI_CONNECTED, HIGH);
    if(!isMqttServiceConnected) digitalWrite(LED_MQTT_CONNECTED, HIGH);
    
    vTaskDelay(300 / portTICK_PERIOD_MS);

    if(!isBluetoothConnected)digitalWrite(LED_BLUETOOTH, LOW);
    if(!isWifiConnected)digitalWrite(LED_WIFI_CONNECTED, LOW);
    if(!isMqttServiceConnected)digitalWrite(LED_MQTT_CONNECTED, LOW);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
String getDeviceName(){
// create a short unique id from efuse MAC (last 4 hex digits)
  uint64_t mac = ESP.getEfuseMac();
  //printf("MAC: %llX\n", mac);

  char idbuf[9];
  snprintf(idbuf, sizeof(idbuf), "%08X", (uint32_t)(mac & 0xFFFFFFFF));

  String deviceName = BT_NAME + "_";
  deviceName += idbuf;
  deviceName.toLowerCase();
  return deviceName;
}
String GetMacAddress(){
  uint64_t mac = ESP.getEfuseMac();
  uint8_t macArr[6];
  for (int i = 0; i < 6; i++) {
    macArr[i] = (mac >> (8 * i)) & 0xFF;
  }

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
          macArr[5], macArr[4], macArr[3], macArr[2], macArr[1], macArr[0]);

  Serial.println(macStr);
  return macStr;
}
void saveWifiCredentials(String ssid, String password, String ip) {
  prefs.begin("wifi", false);  // namespace "wifi"
  prefs.putString("ssid", ssid);
  prefs.putString("pw",   password);
  prefs.putString("ip",   ip);
  prefs.end();
}
String loadSSID() {
  prefs.begin("wifi", true);
  String ssid = prefs.getString("ssid", "");
  prefs.end();
  return ssid;
}
String loadPassword() {
  prefs.begin("wifi", true);
  String pw = prefs.getString("pw", "");
  prefs.end();
  return pw;
}

// WiFi
void wifiCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println();
}
void wifiConnectTask(void *parameter)
{
  for (;;)
  { // Infinite loop

    switch (wifiCasePtr)
    {
    case 0:
      WiFi.begin(ssid, password);
      Serial.printf("\nConnecting to WiFi... %s:1883\n", serverIP);
      wifiPublishClient.setServer(serverIP, 1883);
      wifiPublishClient.setCallback(wifiCallback);
      wifiCasePtr++;
      break;

    case 1:
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.SSID());
        Serial.println(WiFi.localIP());
        isWifiConnected = true;
        wifiCasePtr++;
      }
      break;

    case 2:
        Serial.println("\nConnecting MQQT ...");

        if(wifiPublishClient.connect("geoPublisher")){
          Serial.println("MQTT Connected!");
          wifiPublishClient.loop();
          wifiCasePtr++;
        }
        else
        {
          Serial.print("MQTT Connect failed, ");
          Serial.print(wifiPublishClient.state());
          Serial.println(" try again in 5 seconds");
          delay(5000);
        }
      break;

    case 3:
      if (wifiPublishClient.connected()) {
        Serial.println("\nConnected!\n");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        isMqttServiceConnected = true;
        wifiCasePtr++;
      }
      break;

    case 4:
        wifiPublishClient.publish(MQTT_TOPIC_RESPONSE.c_str(), "online");
        Serial.printf("\nPublish topic...%s", MQTT_TOPIC_RESPONSE.c_str());
        wifiCasePtr++;
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

    Serial.println("Sent: " + msg);
  }
  else
  {
    Serial.println("Connection failed");
  }
}

//Bluetooth
class BT_ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    Serial.printf("BT Client connected: %s \n", connInfo.getAddress().toString().c_str());
    isBluetoothConnected = true;
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    Serial.printf("BT Client disconnected: %s \n", connInfo.getAddress().toString().c_str()); 
    NimBLEDevice::startAdvertising();   
    isBluetoothConnected = false;
  }
};
class BT_HandshakeCallbacks : public NimBLECharacteristicCallbacks {
  
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
    
    String val = pCharacteristic->getValue();
    Serial.print("RX: ");
    Serial.println(val.c_str());

    // Recieved WiFi Credentials
    if (val.startsWith("wificred:")) {
      val.remove(0, 9);  // remove "wificred:"

      int i1 = val.indexOf(">");
      int i2 = val.indexOf(">", i1 + 1);

      if (i1 != -1 && i2 != -1) {

        ssid     = val.substring(0, i1);
        password = val.substring(i1 + 1, i2);
        
        // IP address part
        String ip = val.substring(i2 + 1);
        ip.toCharArray(serverIP, sizeof(serverIP));  
        
        Serial.println("SSID: " + ssid);
        Serial.println("Password: " + password);
        Serial.println("ServerIP: " + String(serverIP));   // char*

        gotWifiCredentials = true;
      }
      else {
        Serial.println("Invalid WiFi credentials format: " + val);
      }
    }
  } 


  void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    // Raspberry PI Subscribed to notifications
    Serial.println("BT Client subscribed to notifications\n");
  }
};
void BT_StartServer(){
  Serial.print("Starting BT Server...\n"); 

  String deviceName = getDeviceName();
  
  NimBLEDevice::init(deviceName.c_str());
  NimBLEDevice::setSecurityAuth(false, false, true);
  
  // Create Server
  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new BT_ServerCallbacks());

  // Create Comms Service
  NimBLEService *pService = pServer->createService(SERVICE_UUID);
  pChar = pService->createCharacteristic(
    CHAR_UUID,
    NIMBLE_PROPERTY::READ | 
    NIMBLE_PROPERTY::WRITE | 
    NIMBLE_PROPERTY::NOTIFY
  );
  pChar->setCallbacks(new BT_HandshakeCallbacks());
  pChar->setValue("IDLE");
  pService->start();
  Serial.printf("BT Server Started: %s\n", deviceName.c_str());
}
void BT_StartAdvertising() {
  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;
  
  advData.setFlags(0x06);                    // general discoverable + BR/EDR not supported
  advData.addServiceUUID(SERVICE_UUID);
  pAdv->setAdvertisementData(advData);

  NimBLEAdvertisementData scanResp;
  scanResp.setName(deviceName.c_str());              // full name delivered in scan response
  pAdv->setScanResponseData(scanResp);

  NimBLEDevice::startAdvertising();
  Serial.printf("BT Advertising Started:  %s \n", deviceName.c_str());
}

void setup(){
  Serial.begin(9600);
  pinMode(LED_BLUETOOTH, OUTPUT);
  pinMode(LED_WIFI_CONNECTED, OUTPUT);
  pinMode(LED_MQTT_CONNECTED, OUTPUT);
  pinMode(BUT_PAIR, INPUT_PULLUP);

  deviceName = getDeviceName();

  attachInterrupt(
      digitalPinToInterrupt(BUT_PAIR),
      buttonISR,
      FALLING);

  xTaskCreatePinnedToCore(
      BlinkTask,        // Task function
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

  //vTaskResume(ConnectWiFiTaskHandle);
  wifiCasePtr = 0;
  mainCasePtr = 0;
}

void loop()
{
  vTaskDelay(10 / portTICK_PERIOD_MS);
  DebouncePairBtn();

  switch (mainCasePtr)
  {
    // Start Bluetooth Server
    case 0:
      isBluetoothConnected = false;
      isWifiConnected = false;
      isMqttServiceConnected = false;

      vTaskResume(BlinkTaskHandle);
      vTaskSuspend(ConnectWiFiTaskHandle);

      BT_StartServer();
      BT_StartAdvertising();
      mainCasePtr++;
    break;
    
    // Wait for bluetooth Connection
    case 1:
      if (isBluetoothConnected)
      {
        digitalWrite(LED_BLUETOOTH, HIGH); // Solid ON
        saveWifiCredentials(ssid, password, serverIP);
        mainCasePtr++;
      }
    break;
    
    // Wait for Wifi Credentials via Bluetooth
    case 2:
     if(gotWifiCredentials)
     {
        // Start WiFi Connection
        isWifiConnected = false;
        isMqttServiceConnected = false;

        vTaskResume(ConnectWiFiTaskHandle);
        mainCasePtr++;
     }
    break;

    // Check Wifi Connection
    case 3:
      if (isWifiConnected)
      {
        digitalWrite(LED_WIFI_CONNECTED, HIGH); // Solid ON
        mainCasePtr++;
      } 
   
      // Check MQTT Service Connection
    case 4:
      if (isMqttServiceConnected)
      {
        vTaskSuspend(ConnectWiFiTaskHandle);
        digitalWrite(LED_MQTT_CONNECTED, HIGH); // Solid ON
        mainCasePtr++;
      } 
    break;
      case 5:
      if(btnPairPressed){
        btnPairPressed = false;
        SendHttpMsg("Button Pressed");
        Serial.println("Send HTTP: Button Pressed");
      }
      break;
  default:
    break;
  }
}