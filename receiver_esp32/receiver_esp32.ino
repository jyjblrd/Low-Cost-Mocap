#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <stdint.h>
#include "sbus.h"

#define GAIN 1.0

unsigned long lastPing;

bfs::SbusTx sbus_tx(&Serial1, 33, 32, true, false);
bfs::SbusData data;

bool armed = false;
unsigned long timeArmed = 0;

StaticJsonDocument<200> json;

double xSetpoint, xPos, xOutput;
double ySetpoint, yPos, yOutput;
double zSetpoint, zPos, zOutput;

int xTrim=0, yTrim=0, zTrim=0, yawTrim=0;

double xKp=2, xKi=5, xKd=1;
double yKp=2, yKi=5, yKd=1;
double zKp=2, zKi=5, zKd=1;

PID xPID(&xPos, &xOutput, &xSetpoint, xKp, xKi, xKd, DIRECT);
PID yPID(&yPos, &yOutput, &ySetpoint, yKp, yKi, yKd, DIRECT);
PID zPID(&zPos, &zOutput, &zSetpoint, zKp, zKi, zKd, DIRECT);

unsigned long lastLoopTime = micros();
unsigned long lastSbusSend = micros();
float loopFrequency = 2000.0;
float sbusFrequency = 50.0;

uint8_t newMACAddress[] = { 0xC0, 0x4E, 0x30, 0x4B, 0x61, 0x3A };

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Serial.println((char*)incomingData);
  DeserializationError err = deserializeJson(json, (char*)incomingData);
  
  if (err) {
    Serial.print("failed to parse json");
    return;
  }

  if (json.containsKey("pos")) {
    xPos = json["pos"][0];
    yPos = -((double)json["pos"][2]);
    zPos = json["pos"][1];
  }
  else if (json.containsKey("armed")) {
    if (json["armed"] != armed && json["armed"]) {
      timeArmed = millis();
    }
    armed = json["armed"];
  }
  else if (json.containsKey("setpoint")) {
    xSetpoint = json["setpoint"][0];
    ySetpoint = json["setpoint"][1];
    zSetpoint = json["setpoint"][2];
  }
  else if (json.containsKey("pid")) {
    xPID.SetTunings(json["pid"][0], json["pid"][1], json["pid"][2]);
    yPID.SetTunings(json["pid"][3], json["pid"][4], json["pid"][5]);
    zPID.SetTunings(json["pid"][6], json["pid"][7], json["pid"][8]);
  }
  else if (json.containsKey("trim")) {
    xTrim = json["trim"][0];
    yTrim = json["trim"][1];
    zTrim = json["trim"][2];
    yawTrim = json["trim"][3];
  }

  lastPing = micros();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  sbus_tx.Begin();
  data.failsafe = false;
  data.ch17 = true;
  data.ch18 = true;
  data.lost_frame = false;
  for (int i = 500; i > 172; i--) {
    for (int j = 0; j < 16; j++) {
      data.ch[j] = i;
    }
    Serial.println(i);
    sbus_tx.data(data);
    sbus_tx.Write();
  }
  
  // Set device as a Wi-Fi Station  
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_ps(WIFI_PS_NONE);
  //esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_start();
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_24M);
  esp_wifi_start();
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);


  xPID.SetMode(AUTOMATIC);
  yPID.SetMode(AUTOMATIC);
  zPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-1*GAIN, 1*GAIN);
  yPID.SetOutputLimits(-1*GAIN, 1*GAIN);
  zPID.SetOutputLimits(-1*GAIN, 1*GAIN);

  lastPing = micros();
  lastLoopTime = micros();
  lastSbusSend = micros();
}

void loop() {
  while (micros() - lastLoopTime < 1e6/loopFrequency) { yield(); }
  lastLoopTime = micros();

  if (micros() - lastPing > 2e6) {
    armed = false;
  }

  if (armed) {
    data.ch[4] = 1800;
  }
  else {
    data.ch[4] = 172;
    xPos = xSetpoint;
    yPos = ySetpoint;
    zPos = zSetpoint;
  }

  xPID.Compute();
  yPID.Compute();
  zPID.Compute();
  int xPWM = 1000 + (xOutput*811) + xTrim;
  int yPWM = 1000 + (-yOutput*811) + yTrim;
  int zPWM = 1000 + (zOutput*811) + zTrim;
  //int zPWM = 900 + zTrim;
  zPWM = armed && millis() - timeArmed > 100 ? zPWM : 172;
  data.ch[0] = yPWM;
  data.ch[1] = xPWM;
  data.ch[2] = zPWM;
  data.ch[3] = 1000 + yawTrim;

  if (micros() - lastSbusSend > 1e6/sbusFrequency) {
    lastSbusSend = micros();
    // Serial.printf("PWM x: %d, y: %d, z: %d\nPos x: %f, y: %f, z: %f\n", xPWM, yPWM, zPWM, xPos, yPos, zPos);
    // Serial.printf("Setpoint x: %f, y: %f, z: %f\n", xSetpoint, ySetpoint, zSetpoint);
    // Serial.printf("Pos x: %f, y: %f, z: %f\n", xPos, yPos, zPos);
    //Serial.printf("Output x: %f, y: %f, z: %f\n", xOutput, yOutput, zOutput);

    sbus_tx.data(data);
    sbus_tx.Write();
  }
}

