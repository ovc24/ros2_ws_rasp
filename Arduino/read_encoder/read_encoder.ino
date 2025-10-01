#include <ModbusMaster.h>
//RX2_PIN 16
//TX2_PIN 17
#define RE_DE_PIN 4
#define MAX_ENCODERS 6
float valores[MAX_ENCODERS]={0,0,0,0,0,0};
ModbusMaster encoder[MAX_ENCODERS];

/* ======= RS485 Transmission Control ======= */
void preTransmission() {
  digitalWrite(RE_DE_PIN, HIGH);
}

void postTransmission() {
  digitalWrite(RE_DE_PIN, LOW);
}

float convertAngle(float angle_deg){
  if(angle_deg > 180.0) angle_deg -= 360.0;
  return angle_deg;
}
/* ======= Setup ======= */
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW);

  for (int i=0; i<MAX_ENCODERS;i++){
    encoder[i].begin(i+1, Serial2);
    encoder[i].preTransmission(preTransmission);
    encoder[i].postTransmission(postTransmission);
  }

  Serial.println("Iniciando lectura del encoder BRITER...");
}

void loop() 
{
  Serial.print("{");
  for (int i=0;i<MAX_ENCODERS;i++){
    uint8_t result = encoder[i].readHoldingRegisters(0x0000, 1);
    if(result==encoder[i].ku8MBSuccess){
      float angle = ((encoder[i].getResponseBuffer(0))*360.0)/1024.0;
      valores[i] = convertAngle(angle);
      Serial.print(valores[i]);
      if(i<MAX_ENCODERS-1) Serial.print(",");
    } else {
      Serial.println(result,HEX);
      if(i<MAX_ENCODERS-1) Serial.print(",");
    }
  }
  Serial.println("}");
  delay(10);
}
