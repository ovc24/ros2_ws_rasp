#include <ESP32Servo.h>

Servo servo1;
const int servoPin = 18;
const int minPulseWidth = 1200;
const int maxPulseWidth = 1800;

void setup() {
  Serial.begin(115200);
  servo1.attach(servoPin, minPulseWidth, maxPulseWidth);
  servo1.setPeriodHertz(50);
  servo1.writeMicroseconds(maxPulseWidth);
  delay(100);
  //Motor 1
  pinMode(15, OUTPUT);
  //digitalWrite(15, HIGH);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  //Motor 2
  pinMode(4, OUTPUT);
  //digitalWrite(4, LOW);
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);
  //Motor 3
  pinMode(17, OUTPUT);
  //digitalWrite(17, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  //Motor 4
  pinMode(32, OUTPUT);
  //digitalWrite(32, LOW);
  pinMode(33, OUTPUT);
  digitalWrite(33, LOW);
  //Motor 5
  pinMode(25, OUTPUT);
  //digitalWrite(25, LOW);
  pinMode(26, OUTPUT);
  digitalWrite(26, LOW);
  //Motor 6
  pinMode(27, OUTPUT);
  //digitalWrite(27, LOW);
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);

}

void loop() {
  if(Serial.available()){
    String input = Serial.readStringUntil('\n');
    Serial.println(input);
    //int pulseWidth = map(, 0, 150, minPulseWidth, maxPulseWidth);
    if (input == "close"){
      servo1.writeMicroseconds(minPulseWidth);
    }
    if (input == "open"){
      servo1.writeMicroseconds(maxPulseWidth);
    }
    if (input == "disable"){
      digitalWrite(15, HIGH);
      digitalWrite(4, HIGH);
      digitalWrite(17, HIGH);
      digitalWrite(32, HIGH);
      digitalWrite(25, HIGH);
      digitalWrite(27, HIGH);
    }
    if (input == "enable"){
      digitalWrite(15, LOW);
      digitalWrite(4, LOW);
      digitalWrite(17, LOW);
      digitalWrite(32, LOW);
      digitalWrite(25, LOW);
      digitalWrite(27, LOW);
    }
  }
}