#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <stdint.h>
#include "sbus.h"

#define GAIN 1.0
#define batVoltagePin 34

unsigned long lastPing;

bfs::SbusTx sbus_tx(&Serial1, 33, 32, true, false);
bfs::SbusData data;

bool armed = false;
unsigned long timeArmed = 0;

StaticJsonDocument<1024> json;

// https://github.com/iNavFlight/inav/wiki/Developer-info
double xPosSetpoint = 0, xVelSetpoint, xVel, xPos, xOutput;
double yPosSetpoint = 0, yVelSetpoint, yVel, yPos, yOutput;
double zPosSetpoint = 0, zVelSetpoint, zVel, zPos, zOutput;
double yawSetpoint = 0, yawPos, yawOutput;

int xTrim = 0, yTrim = 0, zTrim = 0, yawTrim = 0;

double xKp = 0.2, xKi = 0.03, xKd = 0.05;
double yKp = 0.2, yKi = 0.03, yKd = 0.05;
double zKp = 0.6, zKi = 0.3, zKd = 0.1;
double yawKp = 0.3, yawKi = 0.1, yawKd = 0.05;

PID xPID(&xVel, &xOutput, &xVelSetpoint, xKp, xKi, xKd, DIRECT);
PID yPID(&yVel, &yOutput, &yVelSetpoint, yKp, yKi, yKd, DIRECT);
PID zPID(&zVel, &zOutput, &zVelSetpoint, zKp, zKi, zKd, DIRECT);
PID yawPID(&yawPos, &yawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);

unsigned long lastLoopTime = micros();
unsigned long lastSbusSend = micros();
float loopFrequency = 2000.0;
float sbusFrequency = 50.0;

uint8_t newMACAddress[] = { 0xC0, 0x4E, 0x30, 0x4B, 0x61, 0x3A };

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Serial.println((char*)incomingData);
  DeserializationError err = deserializeJson(json, (char *)incomingData);

  if (err) {
    Serial.print("failed to parse json");
    return;
  }

  if (json.containsKey("pos") && json.containsKey("vel")) {
    xPos = json["pos"][0];
    yPos = json["pos"][1];
    zPos = json["pos"][2];
    yawPos = json["pos"][3];

    xVelSetpoint = xPosSetpoint - xPos;
    yVelSetpoint = yPosSetpoint - yPos;
    zVelSetpoint = zPosSetpoint - zPos;

    xVel = json["vel"][0];
    yVel = json["vel"][1];
    zVel = json["vel"][2];
  } else if (json.containsKey("armed")) {
    if (json["armed"] != armed && json["armed"]) {
      timeArmed = millis();
    }
    armed = json["armed"];
  } else if (json.containsKey("setpoint")) {
    xPosSetpoint = json["setpoint"][0];
    yPosSetpoint = json["setpoint"][1];
    zPosSetpoint = json["setpoint"][2];
  } else if (json.containsKey("pid")) {
    xPID.SetTunings(json["pid"][0], json["pid"][1], json["pid"][2]);
    yPID.SetTunings(json["pid"][3], json["pid"][4], json["pid"][5]);
    zPID.SetTunings(json["pid"][6], json["pid"][7], json["pid"][8]);
    yawPID.SetTunings(json["pid"][9], json["pid"][10], json["pid"][11]);
    Serial.println((double)json["pid"][9]);
  } else if (json.containsKey("trim")) {
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
  yawPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-1 * GAIN, 1 * GAIN);
  yPID.SetOutputLimits(-1 * GAIN, 1 * GAIN);
  zPID.SetOutputLimits(-1 * GAIN, 1 * GAIN);
  yawPID.SetOutputLimits(-1 * GAIN, 1 * GAIN);

  lastPing = micros();
  lastLoopTime = micros();
  lastSbusSend = micros();
}

void loop() {
  while (micros() - lastLoopTime < 1e6 / loopFrequency) { yield(); }
  lastLoopTime = micros();

  if (micros() - lastPing > 2e6) {
    armed = false;
  }

  if (armed) {
    data.ch[4] = 1800;
  } else {
    data.ch[4] = 172;
    xVel = xVelSetpoint;
    yVel = yVelSetpoint;
    zVel = zVelSetpoint;
    yawPos = yawSetpoint;
  }

  xPID.Compute();
  yPID.Compute();
  zPID.Compute();
  yawPID.Compute();
  int xPWM = 992 + (xOutput * 811) + xTrim;
  int yPWM = 992 + (yOutput * 811) + yTrim;
  int zPWM = 992 + (zOutput * 811) + zTrim;
  int yawPWM = 992 + (yawOutput * 811) + yawTrim;
  //int zPWM = 900 + zTrim;
  zPWM = armed && millis() - timeArmed > 100 ? zPWM : 172;
  data.ch[0] = -yPWM;
  data.ch[1] = xPWM;
  data.ch[2] = zPWM;
  data.ch[3] = yawPWM;

  if (micros() - lastSbusSend > 1e6 / sbusFrequency) {
    lastSbusSend = micros();
    // Serial.printf("PWM x: %d, y: %d, z: %d, yaw: %d\nPos x: %f, y: %f, z: %f, yaw: %f\n", xPWM, yPWM, zPWM, yawPWM, xVel, yVel, zPos, yawPos);
    // Serial.printf("Setpoint x: %f, y: %f, z: %f\n", xVelSetpoint, yVelSetpoint, zVelSetpoint);
    // Serial.printf("Pos x: %f, y: %f, z: %f\n", xVel, yVel, zPos);
    //Serial.printf("Output x: %f, y: %f, z: %f\n", xOutput, yOutput, zOutput);

    sbus_tx.data(data);
    sbus_tx.Write();
  }
}
