/*
  ESP32 + RS485 Converter: PV Inverter Prod Home Assistant Integration (RS485(Modbus)/MQTT)
*/

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <ESP32.h>

#include <Chrono.h>
#include <WiFi.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialPro.h>

#include <ModbusMaster.h>
#include <MQTT.h>

#include "ESP32_Sofar_KTLX-G3_Modbus_MQTT.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define SERIAL_BAUDRATE 115200  // default serial interface bauderate
#define SERIAL_DELAY 200        // serial communication delay [ms]

#define RS485 Serial2              // rename serial interface name
#define RS485_BAUDRATE 9600        // serial interface bauderate
#define RS485_MODE SERIAL_8N1      // serial mode (1 start / 8 data / 1 stop / no parity)
#define RS485_IO_RX 16             // GPIO16:RX2 (ESP32)
#define RS485_IO_TX 17             // GPIO17:TX2 (ESP32)
#define RS485_DELAY 200            // RS485 communication delay [ms]
#define RS485_RX_BUFFER_SIZE 2048  // RS485 RX buffer size [byte]
#define RS485_TX_BUFFER_SIZE 2048  // RS485 RX buffer size [byte]

#define MODBUS_SLAVE_ID 0x01  // Modbus Slave Address ID: 0x01 Inverter Sofar KTLX-G3
#define MODBUS_DELAY 20       // Modbus communication delay [ms]

#define CONNECTION_DELAY 1000        // WiFi / MQTT connection delay [ms]
#define CONNECTION_RETRY_DELAY 1000  // WiFi / MQTT retry connection delay [ms]

#define MQTT_LOOP_DELAY 10       // MQTT loop communication delay [ms]
#define MQTT_SUBSCRIBE_DELAY 10  // MQTT subscribe communication delay [ms]
#define MQTT_PUBLISH_DELAY 10    // MQTT publish communication delay [ms]

#define MQTT_IO_BUFFER_SIZE 512  // MQTT I/O buffer size [byte]

#define MQTT_KEEP_ALIVE_INTERVAL_CHRONO 60  // chrono MQTT keep alive interval [s] / MQTT_KEEP_ALIVE_INTERVAL_CHRONO > MQTT_KEEP_ALIVE_DELAY_CHRONO!
#define MQTT_KEEP_ALIVE_DELAY_CHRONO 15     // chrono MQTT keep alive delay [s]

#define INVERTER_DATA_HANDLE_INTERVAL_CHRONO 2  // chrono inverter register data handle interval [s]

#define STATE_OUTPUT_REFRESH_INTERVAL_CHRONO 10  // chrono pv inverter + converter states output refresh interval [s]

#define INVERTER_STATE_OFFLINE_CHANGE_LIMIT 5  // pv inverter state "Offline" change limit
#define INVERTER_STATE_OFFLINE_STAGE_LIMIT 10  // pv inverter state "Offline" stage limit

#define DEBUG_OUTPUT  // default serial interface output active

#define OTA_OUTPUT  // over the air serial monitor output active

#define ESP32_RESTART_DELAY 5000  // ESP32 restart command delay [ms]

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// instantiate chrono objects
Chrono mqttKeepAliveChrono(Chrono::SECONDS);
Chrono mqttKeepAliveDelayChrono(Chrono::SECONDS);
Chrono inverterDataHandleChrono(Chrono::SECONDS);
Chrono refreshOutputStateChrono(Chrono::SECONDS);

// instantiate WiFi+MQTT objects
WiFiClient wifiConnectionID;
MQTTClient mqttClient(MQTT_IO_BUFFER_SIZE);

// instantiate Modbus object
ModbusMaster modbusNode;
/* ModbudMaster Error Codes
  ku8MBSuccess          = 0x00;
  ku8MBInvalidSlaveID   = 0xE0;
  ku8MBInvalidFunction  = 0xE1;
  ku8MBResponseTimedOut = 0xE2;
  ku8MBInvalidCRC       = 0xE3;
 */

// instantiate over the air serial monitor
#ifdef OTA_OUTPUT
  AsyncWebServer serialOTA(80);
#endif

// WiFi definitions
const char wifiSSID[] = "WLAN_Belp";  // WiFi network SSID
const char wifiPassword[] = "family.ruch@belp";  // WiFi network password

unsigned int wifiReconnectionCounter = 0;
const unsigned int wifiMaxReconnections = 10;

// Modbus definitions
unsigned int resultModbus;
unsigned long modbusErrorCounter = 0;
unsigned long modbusErrorCodes[4];

// MQTT definitions
const char mqttServer[] = "192.168.1.150";
int mqttServerPort = 1883;
const char mqttUser[] = "mqtt-admin";
const char mqttPassword[] = "mqtt-admin";
const char mqttClientID[] = "pv-inverter_mqtt";
bool mqttDisconnectSkip = false;

String mqttTopic;
String mqttTopicID;
String mqttTopicEntity;
String mqttPayload;
unsigned long mqttPayloadData;
unsigned int mqttPayloadInt;
float mqttPayloadFloat;
bool mqttRetained = false;
const unsigned int mqttQoS = 0;

uint16_t inverterRegisterBlockHead;
uint16_t inverterRegisterBlockSize;
uint16_t inverterRegisterBlockElements;
uint16_t inverterRegisterBlockBaseAddress;
uint16_t inverterEntityRegisterAddress;
uint16_t inverterEntityResponseBufferOffset;
char *inverterEntityDesriptor;
char *inverterEntityDataType;
float inverterEntityDataFactor;
char *inverterEntityUnit;
bool inverterEntityActiveMQTT;
char bufferValueChar;
uint16_t bufferValueUInt16;
uint32_t bufferValueUInt32;
uint16_t inverterStateID = INVERTER_STATE_OFFLINE;
char *inverterState = "Offline";

unsigned int inverterStateOfflineChangeCounter = 0;
unsigned long inverterStateOfflineStageCounter = 0;

bool mqttSuccess;

unsigned int mqttReconnectionCounter = 0;
const unsigned int mqttMaxReconnections = 10;

unsigned long mqttKeepAliveCounter = 0;
unsigned long mqttKeepAliveReceivedCounter = 0;
unsigned long mqttKeepAliveDelayedCounter = 0;

// common definitions
char msgString[256];  // message char array

bool connectionsEstablished = 0;

String inverterRegisterBlockName;
unsigned int inverterRegisterBlockIndex = 0;
unsigned int systenTimeBlockIndex = 0;

unsigned int inverterTimeArray[6];  // [year | month | day | hour | minutue | second]
String inverterTimeYear, inverterTimeMonth, inverterTimeDay, inverterTimeHour, inverterTimeMinute, inverterTimeSecond;
String inverterTime = "1970-01-01 00:00:00";

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  // initialize chronos
  mqttKeepAliveChrono.stop();
  mqttKeepAliveDelayChrono.stop();
  inverterDataHandleChrono.stop();
  refreshOutputStateChrono.stop();
  
  // initialize default serial interface
  Serial.begin(SERIAL_BAUDRATE);
  Serial.flush();
  delay(SERIAL_DELAY);
  
  // initialize serial interface (RS485)
  RS485.begin(RS485_BAUDRATE, RS485_MODE, RS485_IO_RX, RS485_IO_TX);
  RS485.setRxBufferSize(RS485_RX_BUFFER_SIZE);
  RS485.setTxBufferSize(RS485_TX_BUFFER_SIZE);
  RS485.flush();
  delay(RS485_DELAY);
  
  // output infoline
  Serial.println("Inverter Sofar KTLX-G3 Prod [Init Modbus + WiFi + MQTT]");
  Serial.println();
  
  // initalize modbus slave <ID>
  modbusNode.begin(MODBUS_SLAVE_ID, RS485);
  delay(MODBUS_DELAY);
  modbusNode.clearResponseBuffer();
  
  // initialize WiFi network connection
  Serial.print("WiFi connecting...");
  WiFi.mode(WIFI_STA);  // WiFi station mode
  WiFi.begin(wifiSSID, wifiPassword);
  delay(CONNECTION_DELAY);
  wifiReconnectionCounter = 0;
  while((WiFi.status() != WL_CONNECTED) && (wifiReconnectionCounter < wifiMaxReconnections)) {
    wifiReconnectionCounter++;
    WiFi.reconnect();
    Serial.print("-");
    delay(CONNECTION_RETRY_DELAY);
  }
  if(wifiReconnectionCounter >= wifiMaxReconnections) {
    WiFi.disconnect();
    sprintf(msgString, "\nWiFi Error: WiFi not connected after <%u> retries", wifiReconnectionCounter);
    Serial.println(msgString);
    
    esp32Restart();
  }
  Serial.print("\nWiFi <");
  Serial.print(wifiSSID);
  Serial.print("> / <");
  Serial.print(WiFi.localIP());
  Serial.println("> connected.\n");
  
  // initialize over the air serial monitor
  #ifdef OTA_OUTPUT
    WebSerialPro.begin(&serialOTA);
    WebSerialPro.msgCallback(serialOTAReceiver);
    serialOTA.begin();
    WebSerialPro.setID("PV Inverter Prod Home Assistant Integration");
  #endif
  
  // initialize MQTT server connection
  Serial.print("MQTT server <");
  Serial.print(mqttServer);
  Serial.print("> connecting...");
  mqttClient.begin(mqttServer, mqttServerPort, wifiConnectionID);
  delay(CONNECTION_DELAY);
  mqttClient.onMessage(mqttKeepAliveReceiver);
  mqttReconnectionCounter = 0;
  while(!mqttClient.connected() && (mqttReconnectionCounter < mqttMaxReconnections)) {
    mqttReconnectionCounter++;
    mqttClient.connect(mqttClientID, mqttUser, mqttPassword, mqttDisconnectSkip);
    Serial.print("-");
    delay(CONNECTION_RETRY_DELAY);
  }
  if(mqttReconnectionCounter >= mqttMaxReconnections) {
    mqttClient.disconnect();
    sprintf(msgString, "\nMQTT Error: MQTT server not connected after <%u> retries", mqttReconnectionCounter);
    Serial.println(msgString);
    
    esp32Restart();
  }
  Serial.println("\nMQTT server connected.");
  Serial.println();
  
  if((WiFi.status() == WL_CONNECTED) && mqttClient.connected()) {
    connectionsEstablished = 1;
    
    // MQTT output converter states
    mqttOutputConverterState();
    
    // MQTT subscriptions
    #ifdef DEBUG_OUTPUT
      Serial.println("Inverter Sofar KTLX-G3 Prod [MQTT Subscriptions]");
    #endif
    
    // MQTT ESP32 subscription
    mqttTopicID = "ESP32";
    mqttTopicEntity = "Restart";
    mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
    mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
    #ifdef DEBUG_OUTPUT
      Serial.println("ESP32");
      sprintf(msgString, "%s", String(mqttTopicID + "/" + mqttTopicEntity).c_str());
      Serial.println(msgString);
    #endif
    delay(MQTT_SUBSCRIBE_DELAY);
    
    // MQTT inverter modbus converter subscriptions
    #ifdef DEBUG_OUTPUT
      Serial.println("InverterModbusConverterStates");
    #endif
    for(unsigned short element = 0; element < SOFAR_MODBUS_CONVERTER_MQTT_SUBSCRIPTION_ELEMENTS; element++) {
      mqttTopicID = "PVConverter";
      mqttTopicEntity = String(inverterModbusConverterMQTTSubscriptionsArray[element]).c_str();
      mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
      mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
      #ifdef DEBUG_OUTPUT
        sprintf(msgString, "%s", String(mqttTopicID + "/" + mqttTopicEntity).c_str());
        Serial.println(msgString);
      #endif
      delay(MQTT_SUBSCRIBE_DELAY);
    }
    
    // MQTT keep alive subscription
    mqttTopicID = "MQTT";
    mqttTopicEntity = "Keep_Alive_PVInverter";
    mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
    mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
    #ifdef DEBUG_OUTPUT
      Serial.println("MQTT");
      sprintf(msgString, "%s", String(mqttTopicID + "/" + mqttTopicEntity).c_str());
      Serial.println(msgString);
    #endif
    delay(MQTT_SUBSCRIBE_DELAY);
    
    // MQTT inverter system time subscription
    mqttTopicID = "PVInverter";
    mqttTopicEntity = "System_Time";
    mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
    mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
    #ifdef DEBUG_OUTPUT
      Serial.println("SysTime");
      sprintf(msgString, "%s", String(mqttTopicID + "/" + mqttTopicEntity).c_str());
      Serial.println(msgString);
    #endif
    delay(MQTT_SUBSCRIBE_DELAY);
    
    // MQTT inverter register block subscriptions
    for(inverterRegisterBlockIndex = 0; inverterRegisterBlockIndex < INVERTER_REGISTER_BLOCK_COUNT; inverterRegisterBlockIndex++) {
      switch(inverterRegisterBlockIndex) {
        case 0: {  // SysInfo1
          inverterRegisterBlockName = "SysInfo1";
          if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterEntityDesriptor = inverterRegisterSysInfo1[element].inverterRegisterDescriptor;
              inverterEntityActiveMQTT = inverterRegisterSysInfo1[element].inverterRegisterActiveMQTT;
              
              if(inverterEntityActiveMQTT) {
                mqttTopicID = "PVInverter";
                mqttTopic = String(mqttTopicID + "/" + inverterEntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + inverterEntityDesriptor).c_str(), inverterRegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if inverterRegisterBlockActive
          break;
        }  // end case
        case 1: {  // SysInfo2
          inverterRegisterBlockName = "SysInfo2";
          if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterEntityDesriptor = inverterRegisterSysInfo2[element].inverterRegisterDescriptor;
              inverterEntityActiveMQTT = inverterRegisterSysInfo2[element].inverterRegisterActiveMQTT;
              
              if(inverterEntityActiveMQTT) {
                mqttTopicID = "PVInverter";
                mqttTopic = String(mqttTopicID + "/" + inverterEntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + inverterEntityDesriptor).c_str(), inverterRegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if inverterRegisterBlockActive
          break;
        }  // end case
        case 2: {  // SysGridOutput1
          inverterRegisterBlockName = "SysGridOutput1";
          if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterEntityDesriptor = inverterRegisterSysGridOutput1[element].inverterRegisterDescriptor;
              inverterEntityActiveMQTT = inverterRegisterSysGridOutput1[element].inverterRegisterActiveMQTT;
              
              if(inverterEntityActiveMQTT) {
                mqttTopicID = "PVInverter";
                mqttTopic = String(mqttTopicID + "/" + inverterEntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + inverterEntityDesriptor).c_str(), inverterRegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if inverterRegisterBlockActive
          break;
        }  // end case
        case 3: {  // EmergencyOutput1
          inverterRegisterBlockName = "EmergencyOutput1";
          if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterEntityDesriptor = inverterRegisterEmergencyOutput1[element].inverterRegisterDescriptor;
              inverterEntityActiveMQTT = inverterRegisterEmergencyOutput1[element].inverterRegisterActiveMQTT;
              
              if(inverterEntityActiveMQTT) {
                mqttTopicID = "PVInverter";
                mqttTopic = String(mqttTopicID + "/" + inverterEntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + inverterEntityDesriptor).c_str(), inverterRegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if inverterRegisterBlockActive
          break;
        }  // end case
        case 4: {  // InputPV1
          inverterRegisterBlockName = "InputPV1";
          if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterEntityDesriptor = inverterRegisterInputPV1[element].inverterRegisterDescriptor;
              inverterEntityActiveMQTT = inverterRegisterInputPV1[element].inverterRegisterActiveMQTT;
              
              if(inverterEntityActiveMQTT) {
                mqttTopicID = "PVInverter";
                mqttTopic = String(mqttTopicID + "/" + inverterEntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + inverterEntityDesriptor).c_str(), inverterRegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if inverterRegisterBlockActive
          break;
        }  // end case  
        case 5: {  // InputPV2
          inverterRegisterBlockName = "InputPV2";
          if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterEntityDesriptor = inverterRegisterInputPV2[element].inverterRegisterDescriptor;
              inverterEntityActiveMQTT = inverterRegisterInputPV2[element].inverterRegisterActiveMQTT;
              
              if(inverterEntityActiveMQTT) {
                mqttTopicID = "PVInverter";
                mqttTopic = String(mqttTopicID + "/" + inverterEntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + inverterEntityDesriptor).c_str(), inverterRegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if inverterRegisterBlockActive
          break;
        }  // end case
        case 6: {  // InputBat1
          inverterRegisterBlockName = "InputBat1";
          if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterEntityDesriptor = inverterRegisterInputBat1[element].inverterRegisterDescriptor;
              inverterEntityActiveMQTT = inverterRegisterInputBat1[element].inverterRegisterActiveMQTT;
              
              if(inverterEntityActiveMQTT) {
                mqttTopicID = "PVInverter";
                mqttTopic = String(mqttTopicID + "/" + inverterEntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + inverterEntityDesriptor).c_str(), inverterRegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if inverterRegisterBlockActive
          break;
        }  // end case
        case 7: {  // InputBat2
          inverterRegisterBlockName = "InputBat2";
          if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterEntityDesriptor = inverterRegisterInputBat2[element].inverterRegisterDescriptor;
              inverterEntityActiveMQTT = inverterRegisterInputBat2[element].inverterRegisterActiveMQTT;
              
              if(inverterEntityActiveMQTT) {
                mqttTopicID = "PVInverter";
                mqttTopic = String(mqttTopicID + "/" + inverterEntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + inverterEntityDesriptor).c_str(), inverterRegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if inverterRegisterBlockActive
          break;
        }  // end case
        case 8: {  // ElectricityStatistics1
          inverterRegisterBlockName = "ElectricityStatistics1";
          if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterEntityDesriptor = inverterRegisterElectricityStatistics1[element].inverterRegisterDescriptor;
              inverterEntityActiveMQTT = inverterRegisterElectricityStatistics1[element].inverterRegisterActiveMQTT;
              
              if(inverterEntityActiveMQTT) {
                mqttTopicID = "PVInverter";
                mqttTopic = String(mqttTopicID + "/" + inverterEntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + inverterEntityDesriptor).c_str(), inverterRegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if inverterRegisterBlockActive
          break;
        }  // end case
        case 9: {  // UnbalancedSupportControl
          inverterRegisterBlockName = "UnbalancedSupportControl";
          if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterEntityDesriptor = inverterRegisterUnbalancedSupportControl[element].inverterRegisterDescriptor;
              inverterEntityActiveMQTT = inverterRegisterUnbalancedSupportControl[element].inverterRegisterActiveMQTT;
              
              if(inverterEntityActiveMQTT) {
                mqttTopicID = "PVInverter";
                mqttTopic = String(mqttTopicID + "/" + inverterEntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + inverterEntityDesriptor).c_str(), inverterRegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if inverterRegisterBlockActive
          break;
        }  // end case
        default: {
          inverterRegisterBlockIndex = 0;  // reset inverter register block index
          break;
        }  // end default
      }  // end switch inverterRegisterBlockIndex
    }  // end for inverterRegisterBlockIndex
    
    inverterRegisterBlockIndex = 0;  // reset inverter register block index
  }
  
  // reset reconnection counters
  wifiReconnectionCounter = 0;
  mqttReconnectionCounter = 0;
  modbusErrorCounter = 0;
  
  // start chronos
  mqttKeepAliveChrono.start();
  inverterDataHandleChrono.start();
  refreshOutputStateChrono.start();
  
  #ifdef DEBUG_OUTPUT
    Serial.println();
  #endif
  
  // output pv inverter + converter states 
  mqttOutputConverterState();
  outputInfoline();
}  // end setup()

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
   #ifdef LED_ACTIVE
    if(LEDFlashChrono.hasPassed(LEDFlashInterval)) {
      // flash LED to indicate activity
      stateLED = !(stateLED);
      digitalWrite(LED_BUILTIN, stateLED);
      LEDFlashChrono.restart();
    }
  #endif

  // handle MQTT loop
  mqttClient.loop();
  delay(MQTT_LOOP_DELAY);
  
  // output pv inverter + converter states
  if(refreshOutputStateChrono.hasPassed(STATE_OUTPUT_REFRESH_INTERVAL_CHRONO)) {
    mqttOutputConverterState();
    outputInfoline();
    refreshOutputStateChrono.restart();
  }
  
  // handle MQTT keep alive
  if(mqttKeepAliveChrono.hasPassed(MQTT_KEEP_ALIVE_INTERVAL_CHRONO) && connectionsEstablished) {
    mqttKeepAliveCounter++;
    
    mqttTopicID = "MQTT";
    mqttTopicEntity = "Keep_Alive_PVInverter";
    mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
    mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttKeepAliveCounter) + "\":\"" + String(mqttKeepAliveReceivedCounter) + "\":\"" + String(mqttKeepAliveDelayedCounter) + "\"}").c_str();
    mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
    
    mqttKeepAliveChrono.restart();
    if(! mqttKeepAliveDelayChrono.isRunning()) {
      mqttKeepAliveDelayChrono.start();
    } else {
      mqttKeepAliveDelayChrono.restart();
    }
  }  // end if mqttKeepAliveChrono.hasPassed
  
  // handle MQTT keep alive delayes
  if(mqttKeepAliveDelayChrono.hasPassed(MQTT_KEEP_ALIVE_DELAY_CHRONO) && (mqttKeepAliveCounter != 0)) {
    mqttKeepAliveDelayedCounter++;
    
    sprintf(msgString, "MQTT Connection delayed %lux!\n", mqttKeepAliveDelayedCounter);
    Serial.println(msgString);
    #ifdef OTA_OUTPUT
      sprintf(msgString, "MQTT Connection delayed %lux!\n", mqttKeepAliveDelayedCounter);
      WebSerialPro.println(msgString);
    #endif
    
    // ESP32 restart at <mqttMaxReconnections> keep alive delayes
    if((mqttKeepAliveDelayedCounter % 10) == 0) {
      esp32Restart();
    }
  }  // end if mqttKeepAliveDelayChrono.hasPassed
  
  if(inverterDataHandleChrono.hasPassed(INVERTER_DATA_HANDLE_INTERVAL_CHRONO)) {
    if(inverterRegisterBlockIndex == 0) {  // only once per inverter register block cycle
      // handle pv inverter system time
      inverterRegisterBlockName = "SysTime";
      resultModbus = modbusNode.readHoldingRegisters(inverterRegisterBlockSysTime[systenTimeBlockIndex].inverterRegisterBlockHead, inverterRegisterBlockSysTime[systenTimeBlockIndex].inverterRegisterBlockSize);
      delay(MODBUS_DELAY);
      RS485.flush();
      if(resultModbus == modbusNode.ku8MBSuccess) {
        inverterRegisterBlockHead = inverterRegisterBlockSysTime[systenTimeBlockIndex].inverterRegisterBlockHead;
        inverterRegisterBlockSize = inverterRegisterBlockSysTime[systenTimeBlockIndex].inverterRegisterBlockSize;
        inverterRegisterBlockElements = inverterRegisterBlockSysTime[systenTimeBlockIndex].inverterRegisterBlockElements;
        mqttTopicEntity = "System_Time";
        
        // handle modbus holding register data
        #ifdef DEBUG_OUTPUT
          sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
          Serial.println(msgString);
        #endif
        #ifdef OTA_OUTPUT
          sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
          WebSerialPro.println(msgString);
        #endif
        for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
          inverterRegisterBlockBaseAddress = inverterRegisterSysInfo1SysTime[0].inverterRegisterAddress;
          inverterEntityRegisterAddress = inverterRegisterSysInfo1SysTime[element].inverterRegisterAddress;           // 0x042C
          inverterEntityResponseBufferOffset = (inverterEntityRegisterAddress - inverterRegisterBlockBaseAddress);  // (0x042C - 0x0431) = 0x06
          inverterTimeArray[element-2] = modbusNode.getResponseBuffer(inverterEntityResponseBufferOffset);  // update system time array
          delay(MODBUS_DELAY);
        }
        
        mqttTopicID = "PVInverter";
        mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
        
        // adjust date + time format
        inverterTimeArray[0] += 2000;
        sprintf(msgString, "%04u", inverterTimeArray[0]);
        inverterTimeYear = String(msgString);
        sprintf(msgString, "%02u", inverterTimeArray[1]);
        inverterTimeMonth = String(msgString);
        sprintf(msgString, "%02u", inverterTimeArray[2]);
        inverterTimeDay = String(msgString);
        sprintf(msgString, "%02u", inverterTimeArray[3]);
        inverterTimeHour = String(msgString);
        sprintf(msgString, "%02u", inverterTimeArray[4]);
        inverterTimeMinute = String(msgString);
        sprintf(msgString, "%02u", inverterTimeArray[5]);
        inverterTimeSecond = String(msgString);
        inverterTime = String(inverterTimeYear + "-" + inverterTimeMonth + "-" + inverterTimeDay + " / " + inverterTimeHour + ":" + inverterTimeMinute + ":" + inverterTimeSecond).c_str();
        mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(inverterTime) + "\"}").c_str();
        mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
      
        #ifdef DEBUG_OUTPUT
          sprintf(msgString, "%s: ", String(mqttTopicEntity).c_str());
          Serial.print(msgString);
          Serial.println(inverterTime);
        #endif
        #ifdef OTA_OUTPUT
          sprintf(msgString, "%s: ", String(mqttTopicEntity).c_str());
          WebSerialPro.print(msgString);
          WebSerialPro.println(inverterTime);
        #endif
      } else {
        modbusErrorCounter++;
        switch(resultModbus) {
          case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
          case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
          case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
          case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
        }
        #ifdef DEBUG_OUTPUT
          sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, inverterRegisterBlockSysTime[systenTimeBlockIndex].inverterRegisterBlockHead, inverterRegisterBlockSysTime[systenTimeBlockIndex].inverterRegisterBlockSize);
          Serial.println(msgString);
        #endif
      }  // end if resultModbus
      modbusNode.clearResponseBuffer();

      #ifdef DEBUG_OUTPUT
        Serial.println();
      #endif
      #ifdef OTA_OUTPUT
        WebSerialPro.println();
      #endif
    }
    
    // handle pv inverter data
    switch(inverterRegisterBlockIndex) {
      case 0: {  // SysInfo1
        inverterRegisterBlockName = "SysInfo1";
        if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            inverterRegisterBlockHead = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead;
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            
            mqttTopicID = "PVInverter";
            
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterRegisterBlockBaseAddress = inverterRegisterSysInfo1[0].inverterRegisterAddress;
              inverterEntityRegisterAddress = inverterRegisterSysInfo1[element].inverterRegisterAddress;                // 0x0400
              inverterEntityResponseBufferOffset = (inverterEntityRegisterAddress - inverterRegisterBlockBaseAddress);  // (0x0404 - 0x0400) = 0x04
              inverterEntityDesriptor = inverterRegisterSysInfo1[element].inverterRegisterDescriptor;
              inverterEntityDataType = inverterRegisterSysInfo1[element].inverterRegisterDataType;
              inverterEntityDataFactor = inverterRegisterSysInfo1[element].inverterRegisterDataFactor;
              inverterEntityUnit = inverterRegisterSysInfo1[element].inverterRegisterUnit;
              inverterEntityActiveMQTT = inverterRegisterSysInfo1[element].inverterRegisterActiveMQTT;
              
              if(inverterEntityRegisterAddress == INVERTER_REGISTER_SYSINFO1_SYSSTATE) {  // special case sysinfo1 state
                bufferValueUInt16 = modbusNode.getResponseBuffer(inverterEntityResponseBufferOffset);
                delay(MODBUS_DELAY);
                
                inverterStateID = bufferValueUInt16;  // global inverter state ID
                inverterState = inverterRegisterSysInfo1State[inverterStateID].inverterRegisterStateDescriptor;  // global inverter state
                
                if(((inverterState == "Stand By") || (inverterState == "Normal")) && (inverterStateOfflineStageCounter >= INVERTER_STATE_OFFLINE_STAGE_LIMIT)) {  // inverter state stage "Offline" changes after at least <INVERTER_STATE_OFFLINE_STAGE_LIMIT> to "STand By" or "Normal"
                  delay(ESP32_RESTART_DELAY);
                  esp32Restart();
                } else {
                  inverterStateOfflineStageCounter = 0;  // reset counter
                }
                
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s: %u (%s)", String(inverterEntityDesriptor).c_str(), inverterStateID, String(inverterState).c_str());
                  Serial.println(msgString);
                #endif
                #ifdef OTA_OUTPUT
                  sprintf(msgString, "%s: %u (%s)", String(inverterEntityDesriptor).c_str(), inverterStateID, String(inverterState).c_str());
                  WebSerialPro.println(msgString);
                #endif
                if(inverterEntityActiveMQTT) {
                  mqttSuccess = mqttPublishTopicPayloadUInt16(mqttTopicID, inverterEntityDesriptor, inverterStateID, mqttRetained, mqttQoS);
                }
                if((inverterStateID != INVERTER_STATE_FAULT) && (inverterStateID != INVERTER_STATE_PERMANENT_FAULT)) {  // inverter state not 'Fault' and not 'Permanent Fault'
                  element = 20;  // skip fault registers #3...#20
                }
              } else {
                outputInverterData(inverterEntityResponseBufferOffset, inverterEntityDesriptor, inverterEntityDataType, inverterEntityDataFactor, inverterEntityUnit, inverterEntityActiveMQTT, mqttRetained, mqttQoS);
              }
            }
            inverterStateOfflineChangeCounter = 0;
            inverterRegisterBlockIndex++;
          } else if(resultModbus == modbusNode.ku8MBResponseTimedOut) {  // modbus communication error 0xE2 (timeout) set system state "Offline"
            if(inverterStateOfflineChangeCounter < INVERTER_STATE_OFFLINE_CHANGE_LIMIT) {
              // handle pv inverter modbus error 0xE2
              modbusErrorCounter++;
              inverterStateOfflineChangeCounter++;
              modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++;
              inverterStateOfflineStageCounter++;
              
            } else {  
              // handle pv inverter system state "Offline"
              modbusErrorCounter++;
              modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++;
              inverterStateOfflineStageCounter++;
              
              inverterStateOfflineChangeCounter = 0;  // reset modbus error timeout counter
              
              #ifdef DEBUG_OUTPUT
                sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
                Serial.println(msgString);
              #endif
              #ifdef OTA_OUTPUT
                sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
                WebSerialPro.println(msgString);
              #endif
              
              inverterEntityDesriptor = inverterRegisterSysInfo1[INVERTER_REGISTER_SYSINFO1_SYSSTATE_INDEX].inverterRegisterDescriptor;
              inverterStateID = INVERTER_STATE_OFFLINE;
              inverterState = inverterRegisterSysInfo1State[inverterStateID].inverterRegisterStateDescriptor;  // global inverter state
              
              mqttTopicID = "PVInverter";
              mqttTopic = String(mqttTopicID + "/" + inverterEntityDesriptor).c_str();
              mqttPayload = ("{\"" + String(inverterEntityDesriptor) + "\":\"" + inverterStateID + "\"}").c_str();
              mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
              
              #ifdef DEBUG_OUTPUT
                sprintf(msgString, "%s: %u (%s)", String(inverterEntityDesriptor).c_str(), inverterStateID, String(inverterState).c_str());
                Serial.println(msgString);
              #endif
              #ifdef OTA_OUTPUT
                sprintf(msgString, "%s: %u (%s)", String(inverterEntityDesriptor).c_str(), inverterStateID, String(inverterState).c_str());
                WebSerialPro.println(msgString);
              #endif
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          inverterDataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if inverterRegisterBlockActive
        break;
      }  // end case
      case 1: {  // SysInfo2
        if(inverterState == "Offline") {  // pv inverter modbus not responding
          inverterRegisterBlockIndex = 0;  // reset inverter register block index
          break;
        }
        inverterRegisterBlockName = "SysInfo2";
        if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            inverterRegisterBlockHead = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead;
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterRegisterBlockBaseAddress = inverterRegisterSysInfo2[0].inverterRegisterAddress;
              inverterEntityRegisterAddress = inverterRegisterSysInfo2[element].inverterRegisterAddress;
              inverterEntityResponseBufferOffset = (inverterEntityRegisterAddress - inverterRegisterBlockBaseAddress);
              inverterEntityDesriptor = inverterRegisterSysInfo2[element].inverterRegisterDescriptor;
              inverterEntityDataType = inverterRegisterSysInfo2[element].inverterRegisterDataType;
              inverterEntityDataFactor = inverterRegisterSysInfo2[element].inverterRegisterDataFactor;
              inverterEntityUnit = inverterRegisterSysInfo2[element].inverterRegisterUnit;
              inverterEntityActiveMQTT = inverterRegisterSysInfo2[element].inverterRegisterActiveMQTT;
              
              outputInverterData(inverterEntityResponseBufferOffset, inverterEntityDesriptor, inverterEntityDataType, inverterEntityDataFactor, inverterEntityUnit, inverterEntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          inverterDataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if inverterRegisterBlockActive
        inverterRegisterBlockIndex++;
        break;
      }  // end case
      case 2: {  // SysGridOutput1
        if(inverterState == "Offline") {  // pv inverter modbus not responding
          inverterRegisterBlockIndex = 0;  // reset inverter register block index
          break;
        }
        inverterRegisterBlockName = "SysGridOutput1";
        if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            inverterRegisterBlockHead = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead;
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterRegisterBlockBaseAddress = inverterRegisterSysGridOutput1[0].inverterRegisterAddress;
              inverterEntityRegisterAddress = inverterRegisterSysGridOutput1[element].inverterRegisterAddress;
              inverterEntityResponseBufferOffset = (inverterEntityRegisterAddress - inverterRegisterBlockBaseAddress);
              inverterEntityDesriptor = inverterRegisterSysGridOutput1[element].inverterRegisterDescriptor;
              inverterEntityDataType = inverterRegisterSysGridOutput1[element].inverterRegisterDataType;
              inverterEntityDataFactor = inverterRegisterSysGridOutput1[element].inverterRegisterDataFactor;
              inverterEntityUnit = inverterRegisterSysGridOutput1[element].inverterRegisterUnit;
              inverterEntityActiveMQTT = inverterRegisterSysGridOutput1[element].inverterRegisterActiveMQTT;
              
              outputInverterData(inverterEntityResponseBufferOffset, inverterEntityDesriptor, inverterEntityDataType, inverterEntityDataFactor, inverterEntityUnit, inverterEntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          inverterDataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if inverterRegisterBlockActive
        inverterRegisterBlockIndex++;
        break;
      }  // end case
      case 3: {  // EmergencyOutput1
        if(inverterState == "Offline") {  // pv inverter modbus not responding
          inverterRegisterBlockIndex = 0;  // reset inverter register block index
          break;
        }
        inverterRegisterBlockName = "EmergencyOutput1";
        if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            inverterRegisterBlockHead = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead;
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterRegisterBlockBaseAddress = inverterRegisterEmergencyOutput1[0].inverterRegisterAddress;
              inverterEntityRegisterAddress = inverterRegisterEmergencyOutput1[element].inverterRegisterAddress;
              inverterEntityResponseBufferOffset = (inverterEntityRegisterAddress - inverterRegisterBlockBaseAddress);
              inverterEntityDesriptor = inverterRegisterEmergencyOutput1[element].inverterRegisterDescriptor;
              inverterEntityDataType = inverterRegisterEmergencyOutput1[element].inverterRegisterDataType;
              inverterEntityDataFactor = inverterRegisterEmergencyOutput1[element].inverterRegisterDataFactor;
              inverterEntityUnit = inverterRegisterEmergencyOutput1[element].inverterRegisterUnit;
              inverterEntityActiveMQTT = inverterRegisterEmergencyOutput1[element].inverterRegisterActiveMQTT;
              
              outputInverterData(inverterEntityResponseBufferOffset, inverterEntityDesriptor, inverterEntityDataType, inverterEntityDataFactor, inverterEntityUnit, inverterEntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          inverterDataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if inverterRegisterBlockActive
        inverterRegisterBlockIndex++;
        break;
      }  // end case
      case 4: {  // InputPV1
        if(inverterState == "Offline") {  // pv inverter modbus not responding
          inverterRegisterBlockIndex = 0;  // reset inverter register block index
          break;
        }
        inverterRegisterBlockName = "InputPV1";
        if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            inverterRegisterBlockHead = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead;
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterRegisterBlockBaseAddress = inverterRegisterInputPV1[0].inverterRegisterAddress;
              inverterEntityRegisterAddress = inverterRegisterInputPV1[element].inverterRegisterAddress;
              inverterEntityResponseBufferOffset = (inverterEntityRegisterAddress - inverterRegisterBlockBaseAddress);
              inverterEntityDesriptor = inverterRegisterInputPV1[element].inverterRegisterDescriptor;
              inverterEntityDataType = inverterRegisterInputPV1[element].inverterRegisterDataType;
              inverterEntityDataFactor = inverterRegisterInputPV1[element].inverterRegisterDataFactor;
              inverterEntityUnit = inverterRegisterInputPV1[element].inverterRegisterUnit;
              inverterEntityActiveMQTT = inverterRegisterInputPV1[element].inverterRegisterActiveMQTT;
              
              outputInverterData(inverterEntityResponseBufferOffset, inverterEntityDesriptor, inverterEntityDataType, inverterEntityDataFactor, inverterEntityUnit, inverterEntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          inverterDataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if inverterRegisterBlockActive
        inverterRegisterBlockIndex++;
        break;
      }  // end case  
      case 5: {  // InputPV2
        if(inverterState == "Offline") {  // pv inverter modbus not responding
          inverterRegisterBlockIndex = 0;  // reset inverter register block index
          break;
        }
        inverterRegisterBlockName = "InputPV2";
        if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            inverterRegisterBlockHead = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead;
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterRegisterBlockBaseAddress = inverterRegisterInputPV2[0].inverterRegisterAddress;
              inverterEntityRegisterAddress = inverterRegisterInputPV2[element].inverterRegisterAddress;
              inverterEntityResponseBufferOffset = (inverterEntityRegisterAddress - inverterRegisterBlockBaseAddress);
              inverterEntityDesriptor = inverterRegisterInputPV2[element].inverterRegisterDescriptor;
              inverterEntityDataType = inverterRegisterInputPV2[element].inverterRegisterDataType;
              inverterEntityDataFactor = inverterRegisterInputPV2[element].inverterRegisterDataFactor;
              inverterEntityUnit = inverterRegisterInputPV2[element].inverterRegisterUnit;
              inverterEntityActiveMQTT = inverterRegisterInputPV2[element].inverterRegisterActiveMQTT;
              
              outputInverterData(inverterEntityResponseBufferOffset, inverterEntityDesriptor, inverterEntityDataType, inverterEntityDataFactor, inverterEntityUnit, inverterEntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          inverterDataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if inverterRegisterBlockActive
        inverterRegisterBlockIndex++;
        break;
      }  // end case
      case 6: {  // InputBat1
        if(inverterState == "Offline") {  // pv inverter modbus not responding
          inverterRegisterBlockIndex = 0;  // reset inverter register block index
          break;
        }
        inverterRegisterBlockName = "InputBat1";
        if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            inverterRegisterBlockHead = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead;
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterRegisterBlockBaseAddress = inverterRegisterInputBat1[0].inverterRegisterAddress;
              inverterEntityRegisterAddress = inverterRegisterInputBat1[element].inverterRegisterAddress;
              inverterEntityResponseBufferOffset = (inverterEntityRegisterAddress - inverterRegisterBlockBaseAddress);
              inverterEntityDesriptor = inverterRegisterInputBat1[element].inverterRegisterDescriptor;
              inverterEntityDataType = inverterRegisterInputBat1[element].inverterRegisterDataType;
              inverterEntityDataFactor = inverterRegisterInputBat1[element].inverterRegisterDataFactor;
              inverterEntityUnit = inverterRegisterInputBat1[element].inverterRegisterUnit;
              inverterEntityActiveMQTT = inverterRegisterInputBat1[element].inverterRegisterActiveMQTT;
              
              outputInverterData(inverterEntityResponseBufferOffset, inverterEntityDesriptor, inverterEntityDataType, inverterEntityDataFactor, inverterEntityUnit, inverterEntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          inverterDataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if inverterRegisterBlockActive
        inverterRegisterBlockIndex++;
        break;
      }  // end case
      case 7: {  // InputBat2
        if(inverterState == "Offline") {  // pv inverter modbus not responding
          inverterRegisterBlockIndex = 0;  // reset inverter register block index
          break;
        }
        inverterRegisterBlockName = "InputBat2";
        if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            inverterRegisterBlockHead = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead;
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterRegisterBlockBaseAddress = inverterRegisterInputBat2[0].inverterRegisterAddress;
              inverterEntityRegisterAddress = inverterRegisterInputBat2[element].inverterRegisterAddress;
              inverterEntityResponseBufferOffset = (inverterEntityRegisterAddress - inverterRegisterBlockBaseAddress);
              inverterEntityDesriptor = inverterRegisterInputBat2[element].inverterRegisterDescriptor;
              inverterEntityDataType = inverterRegisterInputBat2[element].inverterRegisterDataType;
              inverterEntityDataFactor = inverterRegisterInputBat2[element].inverterRegisterDataFactor;
              inverterEntityUnit = inverterRegisterInputBat2[element].inverterRegisterUnit;
              inverterEntityActiveMQTT = inverterRegisterInputBat2[element].inverterRegisterActiveMQTT;
              
              outputInverterData(inverterEntityResponseBufferOffset, inverterEntityDesriptor, inverterEntityDataType, inverterEntityDataFactor, inverterEntityUnit, inverterEntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          inverterDataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if inverterRegisterBlockActive
        inverterRegisterBlockIndex++;
        break;
      }  // end case
      case 8: {  // ElectricityStatistics1
        if(inverterState == "Offline") {  // pv inverter modbus not responding
          inverterRegisterBlockIndex = 0;  // reset inverter register block index
          break;
        }
        inverterRegisterBlockName = "ElectricityStatistics1";
        if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            inverterRegisterBlockHead = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead;
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterRegisterBlockBaseAddress = inverterRegisterElectricityStatistics1[0].inverterRegisterAddress;
              inverterEntityRegisterAddress = inverterRegisterElectricityStatistics1[element].inverterRegisterAddress;
              inverterEntityResponseBufferOffset = (inverterEntityRegisterAddress - inverterRegisterBlockBaseAddress);
              inverterEntityDesriptor = inverterRegisterElectricityStatistics1[element].inverterRegisterDescriptor;
              inverterEntityDataType = inverterRegisterElectricityStatistics1[element].inverterRegisterDataType;
              inverterEntityDataFactor = inverterRegisterElectricityStatistics1[element].inverterRegisterDataFactor;
              inverterEntityUnit = inverterRegisterElectricityStatistics1[element].inverterRegisterUnit;
              inverterEntityActiveMQTT = inverterRegisterElectricityStatistics1[element].inverterRegisterActiveMQTT;
              
              outputInverterData(inverterEntityResponseBufferOffset, inverterEntityDesriptor, inverterEntityDataType, inverterEntityDataFactor, inverterEntityUnit, inverterEntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          inverterDataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if inverterRegisterBlockActive
        inverterRegisterBlockIndex++;
        break;
      }  // end case
      case 9: {  // UnbalancedSupportControl
        if(inverterState == "Offline") {  // pv inverter modbus not responding
          inverterRegisterBlockIndex = 0;  // reset inverter register block index
          break;
        }
        inverterRegisterBlockName = "UnbalancedSupportControl";
        if(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            inverterRegisterBlockHead = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead;
            inverterRegisterBlockSize = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize;
            inverterRegisterBlockElements = inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(inverterRegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < inverterRegisterBlockElements; element++) {
              inverterRegisterBlockBaseAddress = inverterRegisterUnbalancedSupportControl[0].inverterRegisterAddress;
              inverterEntityRegisterAddress = inverterRegisterUnbalancedSupportControl[element].inverterRegisterAddress;
              inverterEntityResponseBufferOffset = (inverterEntityRegisterAddress - inverterRegisterBlockBaseAddress);
              inverterEntityDesriptor = inverterRegisterUnbalancedSupportControl[element].inverterRegisterDescriptor;
              inverterEntityDataType = inverterRegisterUnbalancedSupportControl[element].inverterRegisterDataType;
              inverterEntityDataFactor = inverterRegisterUnbalancedSupportControl[element].inverterRegisterDataFactor;
              inverterEntityUnit = inverterRegisterUnbalancedSupportControl[element].inverterRegisterUnit;
              inverterEntityActiveMQTT = inverterRegisterElectricityStatistics1[element].inverterRegisterActiveMQTT;
              
              outputInverterData(inverterEntityResponseBufferOffset, inverterEntityDesriptor, inverterEntityDataType, inverterEntityDataFactor, inverterEntityUnit, inverterEntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockHead, inverterRegisterBlocks[inverterRegisterBlockIndex].inverterRegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          inverterDataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if inverterRegisterBlockActive
        inverterRegisterBlockIndex = 0;  // reset inverter register block index
        break;
      }  // end case
      default: {
        inverterRegisterBlockIndex = 0;  // reset inverter register block index
        break;
      }  // end default
    }  // end switch inverterRegisterBlockIndex
  }  // end if inverterDataHandleChrono.hasPassed
}  // end loop()

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void esp32Restart() {
  String mqttTopicID = "ESP32";
  String mqttTopicEntity = "Restart";
  String mqttPayload="1";
  bool mqttSuccess;
  
  Serial.println("ESP32 Restarting...");
  #ifdef OTA_OUTPUT
    WebSerialPro.println("ESP32 Restarting...");
  #endif
  
  // MQTT publish
  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayload) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  
  // reset connections
  RS485.end();
  delay(CONNECTION_DELAY);
  mqttClient.disconnect();
  delay(CONNECTION_DELAY);
  WiFi.disconnect();
  delay(ESP32_RESTART_DELAY);
  
  ESP.restart();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void serialOTAReceiver(uint8_t *data, size_t length) {
  String buffer = "";
  
  for(uint8_t i = 0; i < length; i++) {
    buffer += char(data[i]);
  }
  
  if(buffer == "Restart") {
    WebSerialPro.println("ESP32 Restarting...");
    delay(ESP32_RESTART_DELAY);
    esp32Restart();
  } else if(buffer == "Halt") {
    WebSerialPro.println("ESP32 Halt...");
    delay(ESP32_RESTART_DELAY);
    while(1);
  } else {
    WebSerialPro.println("Usage: \'Restart\' | \'Halt\'");
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void mqttOutputConverterState() {
  for(unsigned short element = 0; element < SOFAR_MODBUS_CONVERTER_MQTT_SUBSCRIPTION_ELEMENTS; element++) {
    switch(element) {
      case 0: {  // Inverter_Modbus_Converter_Modbus_Errors_0xE0
        mqttTopicEntity = String(inverterModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = modbusErrorCodes[0];
        break;
      }  // end case
      case 1: {  // Inverter_Modbus_Converter_Modbus_Errors_0xE1
        mqttTopicEntity = String(inverterModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = modbusErrorCodes[1];
        break;
      }  // end case
      case 2: {  // Inverter_Modbus_Converter_Modbus_Errors_0xE2
        mqttTopicEntity = String(inverterModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = modbusErrorCodes[2];
        break;
      }  // end case
      case 3: {  // Inverter_Modbus_Converter_Modbus_Errors_0xE3
        mqttTopicEntity = String(inverterModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = modbusErrorCodes[3];
        break;
      }  // end case
      case 4: {  // Inverter_Modbus_Converter_MQTT_Keep_Alive
        mqttTopicEntity = String(inverterModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = mqttKeepAliveCounter;
        break;
      }  // end case
      case 5: {  // Inverter_Modbus_Converter_MQTT_Keep_Alive_Received
        mqttTopicEntity = String(inverterModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = mqttKeepAliveReceivedCounter;
        break;
      }  // end case
      case 6: {  // Inverter_Modbus_Converter_MQTT_Keep_Alive_Delayed
        mqttTopicEntity = String(inverterModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = mqttKeepAliveDelayedCounter;
        break;
      }  // end case
      default: {
        break;
      }  // end default
    }  // end switch element

    mqttTopicID = "PVConverter";
    mqttSuccess = mqttPublishTopicPayloadUInt32(mqttTopicID, mqttTopicEntity, mqttPayloadData, mqttRetained, mqttQoS);
  }  // end for element
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void outputInfoline() {
  #ifdef DEBUG_OUTPUT
    sprintf(msgString, "Inverter Sofar KTLX-G3 Prod [%s] - %s", String(inverterState).c_str(), String(inverterTime).c_str());
    Serial.println(msgString);
    
    sprintf(msgString, "Modbus Errors 0xE0|0xE1|0xE2|0xE3 [%lu | %lu | %lu | %lu]", modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX], modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX], modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX], modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]);
    Serial.println(msgString);
    
    sprintf(msgString, "MQTT KeepAlive|Received|Delayed [%lu | %lu | %lu]", mqttKeepAliveCounter, mqttKeepAliveReceivedCounter, mqttKeepAliveDelayedCounter);
    Serial.println(msgString);
    
    Serial.println();
  #endif
  
  #ifdef OTA_OUTPUT
    sprintf(msgString, "Inverter Sofar KTLX-G3 Prod [%s] - %s", String(inverterState).c_str(), String(inverterTime).c_str());
    WebSerialPro.println(msgString);
    
    sprintf(msgString, "Modbus Errors 0xE0|0xE1|0xE2|0xE3 [%lu | %lu | %lu | %lu]", modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX], modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX], modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX], modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]);
    WebSerialPro.println(msgString);
    
    sprintf(msgString, "MQTT KeepAlive|Received|Delayed [%lu | %lu | %lu]", mqttKeepAliveCounter, mqttKeepAliveReceivedCounter, mqttKeepAliveDelayedCounter);
    WebSerialPro.println(msgString);
    
    WebSerialPro.println();
  #endif
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void mqttKeepAliveReceiver(String &inputTopic, String &inputPayload) {
  String mqttKeepAliveTopic;
  String mqttTopicID = "MQTT";
  String mqttTopicEntity = "Keep_Alive_PVInverter";
  
  mqttKeepAliveTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  if(inputTopic == mqttKeepAliveTopic) {
    mqttKeepAliveReceivedCounter++;
    mqttKeepAliveDelayChrono.stop();
    
    #ifdef DEBUG_OUTPUT
      Serial.println("MQTT Message Receiver: " + inputTopic + " - " + inputPayload);
      Serial.println();
    #endif
    #ifdef OTA_OUTPUT
      WebSerialPro.println("MQTT Message Receiver: " + inputTopic + " - " + inputPayload);
      WebSerialPro.println();
    #endif
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void outputInverterData(uint16_t inverterEntityResponseBufferOffset, char *inverterEntityDesriptor, char *inverterEntityDataType, float inverterEntityDataFactor, char *inverterEntityUnit, bool mqttActive, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopicID = "PVInverter";
  bool mqttSuccess;
  
  if(inverterEntityDataType == "CHAR") {
    bufferValueUInt16 = modbusNode.getResponseBuffer(inverterEntityResponseBufferOffset);
    bufferValueChar = (bufferValueUInt16 & 0x00FF);  // 8 bit conversion
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %c", String(inverterEntityDesriptor).c_str(), bufferValueChar);
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %c", String(inverterEntityDesriptor).c_str(), bufferValueChar);
      WebSerialPro.println(msgString);
    #endif
    
    if(mqttActive) {
      //mqttSuccess = mqttPublishTopicPayloadChar(mqttTopicID, inverterEntityDesriptor, bufferValueChar, mqttRetained, mqttQoS);
    }
  } else if(inverterEntityDataType == "U16") {
    bufferValueUInt16 = modbusNode.getResponseBuffer(inverterEntityResponseBufferOffset);
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %u%s", String(inverterEntityDesriptor).c_str(), bufferValueUInt16, String(inverterEntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %u%s", String(inverterEntityDesriptor).c_str(), bufferValueUInt16, String(inverterEntityUnit).c_str());
      WebSerialPro.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadUInt16(mqttTopicID, inverterEntityDesriptor, bufferValueUInt16, mqttRetained, mqttQoS);
    }
  } else if(inverterEntityDataType == "U32") {
    bufferValueUInt32 = ((modbusNode.getResponseBuffer(inverterEntityResponseBufferOffset) << 16) + modbusNode.getResponseBuffer(inverterEntityResponseBufferOffset + 1));
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %u%s", String(inverterEntityDesriptor).c_str(), bufferValueUInt32, String(inverterEntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %u%s", String(inverterEntityDesriptor).c_str(), bufferValueUInt32, String(inverterEntityUnit).c_str());
      WebSerialPro.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadUInt32(mqttTopicID, inverterEntityDesriptor, bufferValueUInt32, mqttRetained, mqttQoS);
    }
  } else if(inverterEntityDataType == "F16") {
    float bufferValueFloat16 = (modbusNode.getResponseBuffer(inverterEntityResponseBufferOffset) * inverterEntityDataFactor);
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %0.2f%s", String(inverterEntityDesriptor).c_str(), bufferValueFloat16, String(inverterEntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %0.2f%s", String(inverterEntityDesriptor).c_str(), bufferValueFloat16, String(inverterEntityUnit).c_str());
      WebSerialPro.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadFloat16(mqttTopicID, inverterEntityDesriptor, bufferValueFloat16, mqttRetained, mqttQoS);
    }
  } else if(inverterEntityDataType == "F32") {
    float bufferValueFloat32 = (((modbusNode.getResponseBuffer(inverterEntityResponseBufferOffset) << 16) + modbusNode.getResponseBuffer(inverterEntityResponseBufferOffset + 1)) * inverterEntityDataFactor);
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %0.2f%s", String(inverterEntityDesriptor).c_str(), bufferValueFloat32, String(inverterEntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %0.2f%s", String(inverterEntityDesriptor).c_str(), bufferValueFloat32, String(inverterEntityUnit).c_str());
      WebSerialPro.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadFloat32(mqttTopicID, inverterEntityDesriptor, bufferValueFloat32, mqttRetained, mqttQoS);
    }
  } else {
    #ifdef DEBUG_OUTPUT
      Serial.println("Unknown Type!");
    #endif
    #ifdef OTA_OUTPUT
      WebSerialPro.println("Unknown Type!");
    #endif
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
bool mqttPublishTopicPayloadCHAR(String mqttTopicID, String mqttTopicEntity, char mqttPayloadCHAR, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopic;
  String mqttPayload;
  bool mqttSuccess;
  
  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayloadCHAR) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  return(mqttSuccess);
}

bool mqttPublishTopicPayloadUInt16(String mqttTopicID, String mqttTopicEntity, uint16_t mqttPayloadUInt16, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopic;
  String mqttPayload;
  bool mqttSuccess;
  
  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayloadUInt16) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  return(mqttSuccess);
}

bool mqttPublishTopicPayloadUInt32(String mqttTopicID, String mqttTopicEntity, uint32_t mqttPayloadUInt32, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopic;
  String mqttPayload;
  bool mqttSuccess;
  
  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayloadUInt32) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  return(mqttSuccess);
}

bool mqttPublishTopicPayloadFloat16(String mqttTopicID, String mqttTopicEntity, float mqttPayloadFloat16, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopic;
  String mqttPayload;
  bool mqttSuccess;
  
  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayloadFloat16, 2) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  return(mqttSuccess);
}

bool mqttPublishTopicPayloadFloat32(String mqttTopicID, String mqttTopicEntity, float mqttPayloadFloat32, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopic;
  String mqttPayload;
  bool mqttSuccess;

  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayloadFloat32, 2) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  return(mqttSuccess);
}

//EOF