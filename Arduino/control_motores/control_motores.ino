#include <FastAccelStepper.h>
#define MAX_MOTORS 6

float valores[MAX_MOTORS];
int cantidad = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *motors[MAX_MOTORS];

int PUL[MAX_MOTORS] = {15, 16, 18, 22, 33, 27};
int DIR_POS[MAX_MOTORS] = {2, 17, 19, 23, 25, 14};
int DIR_NEG[MAX_MOTORS] = {4, 5, 21, 32, 26, 12};

// Configuracion motor
int PASOS_POR_VUELTA[MAX_MOTORS] = {4100, 10000, 10000, 4100, 4000, 1000};
int SPEEDHZ[MAX_MOTORS] = {500, 400, 500, 500, 500, 500};
int ACCELERATION[MAX_MOTORS] = {1000, 500, 800, 1000, 1000, 1000};

float angMin[MAX_MOTORS] = {-50, -60, -60, -90, -25, -180};
float angMax[MAX_MOTORS] = {50, 60, 80, 90, 120, 180};

void setup() {
  Serial.begin(115200);
  engine.init();

  // Motor 1, 2, 3, 4, 5 y 6
  for(int i=0; i<MAX_MOTORS;i++){
    motors[i] = engine.stepperConnectToPin(PUL[i]);
    motors[i]->setAutoEnable(true);
    motors[i]->setSpeedInHz(SPEEDHZ[i]);
    motors[i]->setAcceleration(ACCELERATION[i]);
    pinMode(DIR_POS[i], OUTPUT); pinMode(DIR_NEG[i], OUTPUT);
  }
  Serial.println("Ingrese ángulos de giro en grados para los 6 motores, separados por coma.");
  Serial.println("Ejemplo: 90,-45, 90, 45, 90, 90");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // quitar espacios y saltos de línea
    cantidad = 0; // reset cantidad de valores

    int start = 0;
    int sep = input.indexOf(',');

    // Mientras haya comas en el string
    while (sep != -1 && cantidad < MAX_MOTORS) {
      valores[cantidad] = input.substring(start, sep).toFloat();
      valores[cantidad] = constrain(valores[cantidad], angMin[cantidad], angMax[cantidad]);
      cantidad++;
      start = sep + 1;
      sep = input.indexOf(',', start); // buscar la siguiente coma
    }

    // Guardar el último valor después de la última coma
    if (cantidad < MAX_MOTORS) {
      valores[cantidad] = input.substring(start, sep).toFloat();
      valores[cantidad] = constrain(valores[cantidad], angMin[cantidad], angMax[cantidad]);
      cantidad++;
    }
    for (int i = 0; i<MAX_MOTORS;i++){
      long pasos = round(abs((valores[i] / 360.0)*PASOS_POR_VUELTA[i]));
      bool sentido = (valores[i] >= 0);
      //motors[i]->setCurrentPosition(0);
      digitalWrite(DIR_POS[i], sentido ? HIGH : LOW);
      digitalWrite(DIR_NEG[i], sentido ? LOW : HIGH);
      motors[i]->move(pasos);
    }
    // Serial.printf("Motor2: %ld pasos, direccion %s\n", valores[1], sentido2 ? "CW" : "CCW");
    // Serial.printf("Motor3: %ld pasos, direccion %s\n", pasos3, sentido3 ? "CW" : "CCW");
    // Serial.printf("Motor4: %ld pasos, direccion %s\n", pasos4, sentido4 ? "CW" : "CCW");
    // Serial.printf("Motor5: %ld pasos, direccion %s\n", pasos5, sentido5 ? "CW" : "CCW");
    // Serial.printf("Motor6: %ld pasos, direccion %s\n", pasos6, sentido6 ? "CW" : "CCW");
    //}

    // Limpiar buffer
    // while (Serial.available()) Serial.read();
  }


}
