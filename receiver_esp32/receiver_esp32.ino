#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <stdint.h>

#define PPM_FRAME_LENGTH 22500
#define PPM_PULSE_LENGTH 300
#define PPM_CHANNELS 8
#define DEFAULT_CHANNEL_VALUE 1500
#define OUTPUT_PIN 39

uint16_t channelValue[PPM_CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

enum ppmState_e
{
    PPM_STATE_IDLE,
    PPM_STATE_PULSE,
    PPM_STATE_FILL,
    PPM_STATE_SYNC
};

void IRAM_ATTR onPpmTimer()
{

    static uint8_t ppmState = PPM_STATE_IDLE;
    static uint8_t ppmChannel = 0;
    static uint8_t ppmOutput = LOW;
    static int usedFrameLength = 0;
    int currentChannelValue;

    portENTER_CRITICAL(&timerMux);

    if (ppmState == PPM_STATE_IDLE)
    {
        ppmState = PPM_STATE_PULSE;
        ppmChannel = 0;
        usedFrameLength = 0;
        ppmOutput = LOW;
    }

    if (ppmState == PPM_STATE_PULSE)
    {
        ppmOutput = HIGH;
        usedFrameLength += PPM_PULSE_LENGTH;
        ppmState = PPM_STATE_FILL;

        timerAlarmWrite(timer, PPM_PULSE_LENGTH, true);
    }
    else if (ppmState == PPM_STATE_FILL)
    {
        ppmOutput = LOW;
        currentChannelValue = channelValue[ppmChannel];

        ppmChannel++;
        ppmState = PPM_STATE_PULSE;

        if (ppmChannel >= PPM_CHANNELS)
        {
            ppmChannel = 0;
            timerAlarmWrite(timer, PPM_FRAME_LENGTH - usedFrameLength, true);
            usedFrameLength = 0;
        }
        else
        {
            usedFrameLength += currentChannelValue - PPM_PULSE_LENGTH;
            timerAlarmWrite(timer, currentChannelValue - PPM_PULSE_LENGTH, true);
        }
    }
    portEXIT_CRITICAL(&timerMux);
    digitalWrite(OUTPUT_PIN, ppmOutput);
}

StaticJsonDocument<1024> json;

double xSetpoint, xPos, xOutput;
double ySetpoint, yPos, yOutput;
double zSetpoint, zPos, zOutput;

double xKp=2, xKi=5, xKd=1;
double yKp=2, yKi=5, yKd=1;
double zKp=2, zKi=5, zKd=1;

PID xPID(&xPos, &xOutput, &xSetpoint, xKp, xKi, xKd, DIRECT);
PID yPID(&yPos, &yOutput, &ySetpoint, yKp, yKi, yKd, DIRECT);
PID zPID(&zPos, &zOutput, &zSetpoint, zKp, zKi, zKd, DIRECT);

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.println(len);
  Serial.println(strlen((char*) incomingData));
  Serial.println((char*)incomingData);
  deserializeJson(json, (char*)incomingData);

  xSetpoint = json["setpoint"][0];
  ySetpoint = json["setpoint"][1];
  zSetpoint = json["setpoint"][2];

  xPos = json["pos"][0];
  yPos = json["pos"][1];
  zPos = json["pos"][2];

  if (json.containsKey("xPID")) {
    xPID.SetTunings(json["xPID"][0], json["xPID"][1], json["xPID"][2]);
  }
  if (json.containsKey("yPID")) {
    yPID.SetTunings(json["yPID"][0], json["yPID"][1], json["yPID"][2]);
  }
  if (json.containsKey("zPID")) {
    zPID.SetTunings(json["zPID"][0], json["zPID"][1], json["zPID"][2]);
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);


  xPID.SetMode(AUTOMATIC);
  yPID.SetMode(AUTOMATIC);
  zPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-1, 1);
  yPID.SetOutputLimits(-1, 1);
  zPID.SetOutputLimits(-1, 1);

  // Init 
  pinMode(OUTPUT_PIN, OUTPUT);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onPpmTimer, true);
  timerAlarmWrite(timer, 12000, true);
  timerAlarmEnable(timer);
}
 
void loop() {
  xPID.Compute();
  yPID.Compute();
  zPID.Compute();
  int xPWM = 1500 + (xOutput*500);
  int yPWM = 1500 + (yOutput*500);
  int zPWM = 1500 + (zOutput*500);
  channelValue[0] = xPWM;
  channelValue[1] = yPWM;
  channelValue[2] = zPWM;
  Serial.printf("PWM x: %d, y: %d, z: %d\n", xPWM, yPWM, zPWM);
  // Serial.printf("Setpoint x: %f, y: %f, z: %f\n", xSetpoint, ySetpoint, zSetpoint);
  // Serial.printf("Pos x: %f, y: %f, z: %f\n", xPos, yPos, zPos);
  //Serial.printf("Output x: %f, y: %f, z: %f\n", xOutput, yOutput, zOutput);
  delay(1);
}

