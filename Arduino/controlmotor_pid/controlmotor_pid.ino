#include <FastAccelStepper.h>
#include <ModbusMaster.h>

#define MAX_MOTORES 5
#define RE_DE_PIN 5

float valores[MAX_MOTORES] = {0,0,0,0,0};
float encoders_val[MAX_MOTORES];

int cantidad = 0;

float kp[MAX_MOTORES] = {0.1, 0.06, 0.045, 0.1, 0.1};
float ki[MAX_MOTORES] = {0.0, 0.0, 0.0, 0.0, 0.0};
float kd[MAX_MOTORES] = {0.0, 0.0, 0.0, 0.0013, 0.003};

float angMin[MAX_MOTORES] = {-50, -60, -60, -90, -25};
float angMax[MAX_MOTORES] = {50, 60, 80, 90, 120};

float errorPrev[MAX_MOTORES] = {0};
float integral[MAX_MOTORES] = {0};
float dt = 0.1;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *motors[MAX_MOTORES];

ModbusMaster encoders[MAX_MOTORES];

// Pines Motores
int PUL[MAX_MOTORES] = {15, 27, 18, 22, 33};
int DIR_POS[MAX_MOTORES] = {2, 14, 19, 23, 25};
int DIR_NEG[MAX_MOTORES] = {4, 12, 21, 32, 26};

// Configuracion motor
int PASOS_POR_VUELTA[MAX_MOTORES] = {4100, 10000, 10000, 4100, 4000};
int SPEEDHZ[MAX_MOTORES] = {500,400,500,500,500};
int ACCE[MAX_MOTORES] = {1000,500,800,1000,1000};
//#define PASOS_POR_VUELTA6 1000

void preTransmission() {digitalWrite(RE_DE_PIN, HIGH);}

void postTransmission() {digitalWrite(RE_DE_PIN, LOW);}

float convertAngle(float angle_deg){
  if(angle_deg > 180.0) angle_deg -= 360.0;
  return angle_deg;
}

void setup() {
  Serial.begin(115200);
  
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  
  engine.init();

  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW);

  for (int i = 0; i< MAX_MOTORES; i++){
    motors[i] = engine.stepperConnectToPin(PUL[i]);
    motors[i]->setAutoEnable(true);
    motors[i]->setSpeedInHz(SPEEDHZ[i]);
    motors[i]->setAcceleration(ACCE[i]);

    encoders[i].begin(i + 1, Serial2);
    encoders[i].preTransmission(preTransmission);
    encoders[i].postTransmission(postTransmission);

    pinMode(DIR_POS[i], OUTPUT);
    pinMode(DIR_NEG[i], OUTPUT);
    
  }

  Serial.println("Ingrese ángulos de giro en grados para los 6 motores, separados por coma.");
  Serial.println("Ejemplo: 90,-45, 90, 45, 90, 90");
}

float readEncoderAngle(int i){
  uint8_t result = encoders[i].readHoldingRegisters(0x0000, 1);
  if (result == encoders[i].ku8MBSuccess){
    float angle = ((encoders[i].getResponseBuffer(0))*360.0)/1024.0;
    return convertAngle(angle);
  }
  return encoders_val[i];
}

float computePID(int i, float target, float actual){
  float error = (target - actual);
  if (fabs(error) < 3) error = 0;
  integral[i] += error*dt;
  float derivative = (error - errorPrev[i])/dt;
  errorPrev[i] = error;
  return kp[i]*error + ki[i]*integral[i] + kd[i]*derivative;
}

void moveMotor(int i, float control){
  long pasos = round(abs((control / 360.0)*PASOS_POR_VUELTA[i]));
  bool sentido = (control >= 0);
  //motors[i]->setCurrentPosition(0);
  digitalWrite(DIR_POS[i], sentido ? HIGH : LOW);
  digitalWrite(DIR_NEG[i], sentido ? LOW : HIGH);
  motors[i]->move(pasos);
  
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // quitar espacios y saltos de línea
    cantidad = 0; // reset cantidad de valores

    int inicio = 0;
    int sep = input.indexOf(',');

    // Mientras haya comas en la trama
    while (sep != -1 && cantidad < MAX_MOTORES) {
      valores[cantidad] = input.substring(inicio, sep).toFloat();
      valores[cantidad] = constrain(valores[cantidad], angMin[cantidad], angMax[cantidad]);
       //= token.toFloat();  // convertir a float
      cantidad++;
      inicio = sep + 1;
      sep = input.indexOf(',', inicio); // buscar la siguiente coma
    }
        // // Guardar el último valor después de la última coma
    if (cantidad < MAX_MOTORES) {
      valores[cantidad] = input.substring(inicio).toFloat();
      valores[cantidad] = constrain(valores[cantidad], angMin[cantidad], angMax[cantidad]);
       //= token.toFloat();  // convertir a float
      cantidad++;
    }
  }

  //Serial.print("{");
  for (int i =0; i<MAX_MOTORES; i++){
    encoders_val[i] = readEncoderAngle(i);
    float control = computePID(i, valores[i], encoders_val[i]);
    //moveMotor(i, valores[i]);
    moveMotor(i, control);
    // Serial.print("[");
    // Serial.print(control);
    // Serial.print("]");

  }
  // Serial.println("}");
  // delay(1);
  Serial.print("{");
  for (int i = 0; i<MAX_MOTORES; i++){
    Serial.print(encoders_val[i]);
    if (i<MAX_MOTORES-1) Serial.print(",");
  }
  Serial.println("}");
  delay(10);

}
