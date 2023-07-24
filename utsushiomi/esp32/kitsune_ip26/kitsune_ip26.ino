// EEPROMでIP設定
//  本番プログラムにはIP書かない
//  あらかじめEEPROMに書き込んでおいて、本番プログラムはEEPROM読むだけに
//
// OSC受信
//  OSC library
//  https://github.com/CNMAT/OSC
//
// DMX送信
//  esp_dmx library 3.0.0-beta
//  https://www.arduino.cc/reference/en/libraries/esp_dmx/
//
// FreeRTOS
//  それぞれのTaskに、ArduinoOTA.handle();

#include "EEPROM.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// OSC
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>

// #define EEPROM_WRITE

WiFiUDP Udp;
const char *raspberrypi = "192.168.11.108";  //音を出したいラズパイへ
const int port1 = 32700;
const int port2 = 32701;
const int port3 = 32702;

int RX_PIN = 16;
int TX_PIN = 17;
int RS485 = 18;
int slider = 0;
int last_slider = 0;
OSCErrorCode error;

// WiFi
int address = 0;
String ssid = "";
String password = "";
String local_ip_str = "";
String gateway_ip_str = "";
String subnet_ip_str = "";

IPAddress _local_ip;
IPAddress _gateway;
IPAddress _subnet;


// FreeRTOS
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

void TaskOsc(void *pvParameters);
void TaskSerial(void *pvParameters);


// --------------------------------------------
// SETUP
void setup() {
  Serial.begin(115200);

  if (!EEPROM.begin(1000)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }

#if defined(EEPROM_WRITE)
  EEPROM_write();
  Serial.println("EEPROM write done!");
  return;
#else
  EEPROM_read();
  _local_ip.fromString(local_ip_str);
  _gateway.fromString(gateway_ip_str);
  _subnet.fromString(subnet_ip_str);
#endif

  Serial2.begin(57600);

  pinMode(RS485, OUTPUT);
  delay(10);
  // delete old config
  WiFi.disconnect(true);

  WiFi.mode(WIFI_STA);
  if (!WiFi.config(_local_ip, _gateway, _subnet)) {
    Serial.println("STA Failed to configure");
  }
  
  bool done = true;
  WiFi.begin(ssid.c_str(), password.c_str());
  Serial.println();
  while (done) {
    Serial.print("WiFi connecting");
    auto last = millis();
    while (WiFi.status() != WL_CONNECTED && last + 5000 > millis()) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      done = false;
    } else {
      Serial.println("retry");
      WiFi.disconnect();
      WiFi.reconnect();
    }
  }

  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  Udp.begin(port3);
  Serial.print("Local port: ");
  Serial.println(port3);

  xTaskCreateUniversal(
    TaskSerial, "TaskSerial", 8192, NULL, 2, NULL, ARDUINO_RUNNING_CORE);

  xTaskCreateUniversal(
    TaskOsc, "TaskOsc", 8192, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
}




// --------------------------------------------
// TASK
// --------------------------------------------
// シリアル受信タスク
void TaskSerial(void *pvParameters) {
  (void)pvParameters;
  while (1) {
    digitalWrite(RS485, LOW);
    while (Serial2.available()) {
      slider = Serial2.read();
      Serial.println(slider);
    }
    vTaskDelay(1);
  }
}
// OSC送信タスク
void TaskOsc(void *pvParameters) {
  (void)pvParameters;
  while (1) {
    if (last_slider != slider) {
      //the message wants an OSC address as first argument
      OSCMessage msg("/pd");
      msg.add((int32_t)slider);

      Udp.beginPacket(raspberrypi, port3);
      msg.send(Udp);    // send the bytes to the SLIP stream
      Udp.endPacket();  // mark the end of the OSC Packet
      msg.empty();      // free space occupied by message
    }
    last_slider = slider;
    vTaskDelay(1);
  }
}

// --------------------------------------------
// LOOP
void loop() {
  delay(1);
}
