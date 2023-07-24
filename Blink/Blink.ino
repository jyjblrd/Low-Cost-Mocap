/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

int hertz = 146;
#define OUTPUT_PIN 33
int count = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}


void loop() {
  int delayMicros = 1E6/hertz;
  int timer = micros();
  digitalWrite(OUTPUT_PIN, HIGH);
  if (count == 2) {
    //hertz -= 1;
    digitalWrite(LED_BUILTIN, HIGH);
  }
  delayMicroseconds(delayMicros/2);
  digitalWrite(OUTPUT_PIN, LOW); 
  count++;
  if (count == 4) {
    //hertz -= 1;
    digitalWrite(LED_BUILTIN, LOW);
    count = 0;
  }
  while (micros() - timer < delayMicros) {delayMicroseconds(10);}
}
