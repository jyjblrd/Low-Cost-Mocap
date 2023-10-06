#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <stdint.h>
#include <EEPROM.h>
#include "sbus.h"

#define batVoltagePin 34
#define MAX_VEL 100
#define ROTOR_RADIUS 0.0225
#define Z_GAIN 0.7

#define DRONE_INDEX 1

#define EEPROM_SIZE 4

unsigned long lastPing;

bfs::SbusTx sbus_tx(&Serial1, 33, 32, true, false);
bfs::SbusData data;

bool armed = false;
unsigned long timeArmed = 0;

StaticJsonDocument<1024> json;

int xTrim = 0, yTrim = 0, zTrim = 0, yawTrim = 0;

double groundEffectCoef = 28, groundEffectOffset = -0.035;

// nested pid loops
// outer: position pid loop
// inner: velocity pid loop
// velocity pid loop sends accel setpoint to flight controller
double xPosSetpoint = 0, xPos = 0;
double yPosSetpoint = 0, yPos = 0;
double zPosSetpoint = 0, zPos = 0;
double yawPosSetpoint = 0, yawPos, yawPosOutput;

double xyPosKp = 1, xyPosKi = 0, xyPosKd = 0;
double zPosKp = 1.5, zPosKi = 0, zPosKd = 0;
double yawPosKp = 0.3, yawPosKi = 0.1, yawPosKd = 0.05;

double xVelSetpoint, xVel, xVelOutput;
double yVelSetpoint, yVel, yVelOutput;
double zVelSetpoint, zVel, zVelOutput;

double xyVelKp = 0.2, xyVelKi = 0.03, xyVelKd = 0.05;
double zVelKp = 0.3, zVelKi = 0.1, zVelKd = 0.05;

PID xPosPID(&xPos, &xVelSetpoint, &xPosSetpoint, xyPosKp, xyPosKi, xyPosKd, DIRECT);
PID yPosPID(&yPos, &yVelSetpoint, &yPosSetpoint, xyPosKp, xyPosKi, xyPosKd, DIRECT);
PID zPosPID(&zPos, &zVelSetpoint, &zPosSetpoint, zPosKp, zPosKi, zPosKd, DIRECT);
PID yawPosPID(&yawPos, &yawPosOutput, &yawPosSetpoint, yawPosKp, yawPosKi, yawPosKd, DIRECT);

PID xVelPID(&xVel, &xVelOutput, &xVelSetpoint, xyVelKp, xyVelKi, xyVelKd, DIRECT);
PID yVelPID(&yVel, &yVelOutput, &yVelSetpoint, xyVelKp, xyVelKi, xyVelKd, DIRECT);
PID zVelPID(&zVel, &zVelOutput, &zVelSetpoint, zVelKp, zVelKi, zVelKd, DIRECT);


unsigned long lastLoopTime = micros();
unsigned long lastSbusSend = micros();
float loopFrequency = 2000.0;
float sbusFrequency = 50.0;

#if DRONE_INDEX == 0
  uint8_t newMACAddress[] = { 0xC0, 0x4E, 0x30, 0x4B, 0x61, 0x3A };
#elif DRONE_INDEX == 1
  uint8_t newMACAddress[] = { 0xC0, 0x4E, 0x30, 0x4B, 0x80, 0x3B };
#endif

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
    xPosPID.SetTunings(json["pid"][0], json["pid"][1], json["pid"][2]);
    yPosPID.SetTunings(json["pid"][0], json["pid"][1], json["pid"][2]);
    zPosPID.SetTunings(json["pid"][3], json["pid"][4], json["pid"][5]);
    yawPosPID.SetTunings(json["pid"][6], json["pid"][7], json["pid"][8]);

    xVelPID.SetTunings(json["pid"][9], json["pid"][10], json["pid"][11]);
    yVelPID.SetTunings(json["pid"][9], json["pid"][10], json["pid"][11]);
    zVelPID.SetTunings(json["pid"][12], json["pid"][13], json["pid"][14]);

    groundEffectCoef = json["pid"][15];
    groundEffectOffset = json["pid"][16];
  } else if (json.containsKey("trim")) {
    xTrim = json["trim"][0];
    yTrim = json["trim"][1];
    zTrim = json["trim"][2];
    yawTrim = json["trim"][3];
  }

  lastPing = micros();
}

void resetPid(PID &pid, double min, double max) {
  pid.SetOutputLimits(0.0, 1.0); 
  pid.SetOutputLimits(-1.0, 0.0);
  pid.SetOutputLimits(min, max);
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


  xPosPID.SetMode(AUTOMATIC);
  yPosPID.SetMode(AUTOMATIC);
  zPosPID.SetMode(AUTOMATIC);
  yawPosPID.SetMode(AUTOMATIC);
  xVelPID.SetMode(AUTOMATIC);
  yVelPID.SetMode(AUTOMATIC);
  zVelPID.SetMode(AUTOMATIC);
  
  // Sample rate is determined by main loop
  xPosPID.SetSampleTime(0);
  yPosPID.SetSampleTime(0);
  zPosPID.SetSampleTime(0);
  yawPosPID.SetSampleTime(0);
  xVelPID.SetSampleTime(0);
  yVelPID.SetSampleTime(0);
  zVelPID.SetSampleTime(0);

  xPosPID.SetOutputLimits(-MAX_VEL, MAX_VEL);
  yPosPID.SetOutputLimits(-MAX_VEL, MAX_VEL);
  zPosPID.SetOutputLimits(-MAX_VEL, MAX_VEL);
  yawPosPID.SetOutputLimits(-1, 1);
  xVelPID.SetOutputLimits(-1, 1);
  yVelPID.SetOutputLimits(-1, 1);
  zVelPID.SetOutputLimits(-1, 1);

  EEPROM.begin(EEPROM_SIZE);

  // xTrim = EEPROM.read(0);
  // yTrim = EEPROM.read(1);
  // zTrim = EEPROM.read(2);
  // yawTrim = EEPROM.read(3);

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
    resetPid(xPosPID, -MAX_VEL, MAX_VEL);
    resetPid(yPosPID, -MAX_VEL, MAX_VEL);
    resetPid(zPosPID, -MAX_VEL, MAX_VEL);
    resetPid(yawPosPID, -1, 1);
    resetPid(xVelPID, -1, 1);
    resetPid(yVelPID, -1, 1);
    resetPid(zVelPID, -1, 1);
  }

  xPosPID.Compute();
  yPosPID.Compute();
  zPosPID.Compute();
  yawPosPID.Compute();

  xVelPID.Compute();
  yVelPID.Compute();
  zVelPID.Compute();
  int xPWM = 992 + (xVelOutput * 811) + xTrim;
  int yPWM = 992 + (yVelOutput * 811) + yTrim;
  int zPWM = 992 + (Z_GAIN * zVelOutput * 811) + zTrim;
  int yawPWM = 992 + (yawPosOutput * 811) + yawTrim;
  double groundEffectMultiplier = 1 - groundEffectCoef*pow(((2*ROTOR_RADIUS) / (4*(zPos-groundEffectOffset))), 2);
  zPWM *= max(0., groundEffectMultiplier);
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
    //Serial.printf("Output x: %f, y: %f, z: %f\n", xVelOutput, yVelOutput, zVelOutput);

    sbus_tx.data(data);
    sbus_tx.Write();
  }
}
