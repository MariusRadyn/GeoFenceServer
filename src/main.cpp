// ESP32 GeoFence Monitor

#include <WiFi.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <Arduino.h>
#include <PubSubClient.h>

const char *ssid = "VoxFibre#59923";
const char *password = "8JudJNS2";
const char *serverIP = "192.168.1.108"; // <-- replace with your Pi's IP
const int port = 8080;

#define DEBOUNCE_DELAY 100
#define DEBOUNCE_DELAY_HELD 3000
#define MS_DELAY 10
#define SERVICE_UUID "f3a1c2d0-6b4e-4e9a-9f3e-8d2f1c9b7a1e"
#define CHAR_UUID "c7b2e3f4-1a5d-4c3b-8e2f-9a6b1d8c2f3a"

WiFiClient wifiClient;
PubSubClient pubClient(wifiClient);

int CasePtr = 0;
bool isDiscoverable = false;

// Button Pair
const int BUT_PAIR = 22; // GPIO22 button
volatile bool btnPairFallEdge = false;
volatile bool btnPairPressed = false;
volatile bool btnPairHeld = false;
volatile int lastDebTime = 0;

const int LED_ONBOARD = 2; // Built-in LED

void SendHttpMsg(String msg);
void buttonISR();

// Declare task handle
TaskHandle_t BlinkTaskHandle = NULL;
TaskHandle_t ConnectWiFiTaskHandle = NULL;

// Bluetooth
const char* BT_NAME = "GeoServer"; // Bluetooth Name
NimBLECharacteristic* pChar;

bool btConnected = false;
bool wifiConnected = false;
bool time_1sec_Flag = false;
String deviceName;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
     Serial.printf("Client connected: %s \n", connInfo.getAddress().toString().c_str());
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    Serial.printf("Client disconnected: %s \n", connInfo.getAddress().toString().c_str()); 
     NimBLEDevice::startAdvertising();   
  };
};
class HandshakeCallbacks : public NimBLECharacteristicCallbacks {
  
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
    std::string val = pCharacteristic->getValue();
    Serial.print("RX: ");
    Serial.println(val.c_str());

    // Expected handshake from client
    std::string ackClient = "ACK_FROM_CLIENT";  //From Raspberry Pi
    std::string ackServer = "ACK_FROM_SERVER";  //From ESP32

    if (val == ackClient.c_str()) {
      // reply to client
      pChar->setValue(ackServer);
      pChar->notify(true);
      Serial.printf("TX: %s\n", ackServer.c_str() );
    } else {
      // optional: echo back
      std::string reply = "RX (unknown): ";
      reply += val;
      pChar->setValue(reply);
      pChar->notify(true);
    }
  }

  void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    // Raspberry PI Subscribed to notifications
    Serial.println("Client subscribed to notifications\n");
  }

};

// General
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
        isDiscoverable = false;
        vTaskSuspend(BlinkTaskHandle);
        digitalWrite(LED_ONBOARD, LOW);
        Serial.println("Button Pressed");
      }
    }
  }
  else if (btnPairPressed && !btnPairHeld)
  {
    uint32_t currentTime = millis();

    // Button HELD
    if (currentTime - lastDebTime > DEBOUNCE_DELAY_HELD)
    {
      btnPairHeld = true;
      isDiscoverable = true;
      vTaskResume(BlinkTaskHandle);
      Serial.println("Button Held");
    }

    // Wait for release
    if (digitalRead(BUT_PAIR) == HIGH)
    {
      btnPairPressed = false;
      btnPairHeld = false;
      lastDebTime = 0;
      Serial.println("Button Released");
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
      Serial.println("Button Released");
    }
  }
}
void BlinkTask(void *parameter)
{
  for (;;)
  { // Infinite loop
    digitalWrite(LED_ONBOARD, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000ms

    digitalWrite(LED_ONBOARD, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
String getDeviceName(){
// create a short unique id from efuse MAC (last 4 hex digits)
  uint64_t mac = ESP.getEfuseMac();
  char idbuf[9];
  snprintf(idbuf, sizeof(idbuf), "%04X", (uint16_t)(mac & 0xFFFF));

  String deviceName = BT_NAME;
  deviceName += idbuf;
  return deviceName;
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

    switch (CasePtr)
    {
    case 0:
      WiFi.begin(ssid, password);
      Serial.println("\nConnecting to WiFi...");
      CasePtr++;
      break;

    case 1:
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.SSID());
        Serial.println(WiFi.localIP());
        CasePtr++;
      }
      break;

    case 2:
        //SendHttpMsg("Button Pressed");
        Serial.println("\nConnecting MQQT ...");
        pubClient.connect("geoPublisher");
        pubClient.loop();

      break;

    case 3:
      if (pubClient.connected()) {
        Serial.println("\nConnected!\n");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        CasePtr++;
      }
      break;

    case 4:
        pubClient.publish("devices/esp32_1/status", "online");
        delay(2000);
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
void BT_StartServer(){
  Serial.print("Starting BT Server...\n"); 

  String deviceName = getDeviceName();
  
  NimBLEDevice::init(deviceName.c_str());
  NimBLEDevice::setSecurityAuth(false, false, true);
  
  // Create Server
  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create Comms Service
  NimBLEService *pService = pServer->createService(SERVICE_UUID);
  pChar = pService->createCharacteristic(
    CHAR_UUID,
    NIMBLE_PROPERTY::READ | 
    NIMBLE_PROPERTY::WRITE | 
    NIMBLE_PROPERTY::NOTIFY
  );
  pChar->setCallbacks(new HandshakeCallbacks());
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

 // NimBLEDevice::startAdvertising();
 // Serial.printf("BT Advertising Started:  %s \n", deviceName.c_str());
}

void setup(){
  Serial.begin(9600);
  pinMode(LED_ONBOARD, OUTPUT);
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

  vTaskSuspend(BlinkTaskHandle);
  vTaskSuspend(ConnectWiFiTaskHandle);

  BT_StartServer();
  BT_StartAdvertising();

  pubClient.setServer(serverIP, 1883);
  pubClient.setCallback(wifiCallback);
}

void loop()
{
  DebouncePairBtn();
  
  if (isDiscoverable) {
    // Start Bluetooth Discovery
    isDiscoverable = false;
    NimBLEDevice::startAdvertising();
    Serial.printf("BT Advertising Started:  %s \n", deviceName.c_str());
  }
}