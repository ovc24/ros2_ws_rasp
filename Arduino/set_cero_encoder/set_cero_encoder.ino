#include <ModbusMaster.h>

//RX2_PIN 16
//TX2_PIN 17
#define RE_DE_PIN 5

ModbusMaster node;

/* ======= RS485 Transmission Control ======= */
void preTransmission() {
  digitalWrite(RE_DE_PIN, HIGH);
}

void postTransmission() {
  digitalWrite(RE_DE_PIN, LOW);
}
  
/* ======= Setup ======= */
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW);
  node.begin(3, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  Serial.println("Reiniciando posición del encoder (posición actual = 0)");
  uint8_t result = node.writeSingleRegister(0x0008, 0x0001);
  //uint8_t result = node.writeSingleRegister(0x0009, 0x0000);
  //uint8_t result = node.writeSingleRegister(0x0009, 0x0001);
    if (result == node.ku8MBSuccess) {
    Serial.println("✔️ Encoder reseteado con éxito.");
  } else {
    Serial.print(" Error al resetear encoder: ");
    Serial.println(result, HEX);
  }

  delay(1000);  // Espera antes de comenzar a leer
}

void loop() {
  uint8_t result = node.readHoldingRegisters(0x0000, 1);  // leer registro 0x0000

  if (result == node.ku8MBSuccess) {
    uint16_t raw = node.getResponseBuffer(0);
    float angulo = (raw * 360.0) / 1024.0;
    Serial.print("Ángulo actual: ");
    Serial.println(angulo, 2);
  } else {
    Serial.print("Error Modbus al leer ángulo: ");
    Serial.println(result, HEX);
  }

  delay(100);  // ajusta según la frecuencia deseada
}